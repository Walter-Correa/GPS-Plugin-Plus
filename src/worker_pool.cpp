#include "worker_pool.h"
#include "pathfinder.h"
#include "container.h"
#include "amx_handler.h"
#include <algorithm>

namespace WorkerPool
{
	namespace
	{
		std::vector<std::thread> workers_;
		std::deque<PathJob> jobs_;
		std::vector<PathJobResult> results_;
		std::mutex jobs_mutex_;
		std::mutex results_mutex_;
		std::condition_variable cv_jobs_;
		std::atomic<bool> running_{ false };
		int thread_count_{ 0 };

		constexpr size_t MAX_QUEUED_JOBS = 1024;

		void WorkerThreadFunc()
		{
			while (running_.load(std::memory_order_relaxed))
			{
				PathJob job;
				{
					std::unique_lock<std::mutex> lock(jobs_mutex_);
					cv_jobs_.wait(lock, [] { return !running_.load(std::memory_order_relaxed) || !jobs_.empty(); });

					if (!running_.load(std::memory_order_relaxed))
					{
						break;
					}

					if (jobs_.empty())
					{
						continue;
					}

					job = std::move(jobs_.front());
					jobs_.pop_front();
				}

				PathJobResult result;
				result.amx = job.amx;
				result.amx_serial = job.amx_serial;
				result.callback = job.callback;
				result.success = false;
				result.length = 0.0f;

				const auto now = std::chrono::steady_clock::now();
				if (job.first_attempt_time == std::chrono::steady_clock::time_point{})
				{
					job.first_attempt_time = now;
				}

				// After 3 yields or 100ms since first processing attempt, complete without yielding
				const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - job.first_attempt_time).count();
				const bool should_force_completion = (job.yield_count >= 3 || elapsed_ms >= 100);
				const bool allow_yield = !should_force_completion;

				try
				{
					std::shared_lock<std::shared_mutex> g_lock(Container::GetGraphMutex());
					Node* start = Container::Nodes::Find(job.start_id);
					Node* target = Container::Nodes::Find(job.target_id);

					if (start != nullptr && target != nullptr && !start->isSetForDeletion() && !target->isSetForDeletion())
					{
						std::vector<Node*> nodes;
						float length = 0.0f;
						if (Pathfinder::FindPathNodes(start, target, nodes, length, job.type, job.pref, job.cat, 0.0f, 0.0f, false, false, 0.0f, allow_yield))
						{
							result.success = true;
							result.length = length;
							result.node_ids.reserve(nodes.size());
							for (auto n : nodes)
							{
								result.node_ids.push_back(n->getID());
							}
						}
					}
				}
				catch (const PathfinderYieldException&)
				{
					job.yield_count++;

					// Wait until writers are done or shutdown requested
					Container::WaitForWritersDone([&]() {
						return !running_.load(std::memory_order_relaxed);
					});

					if (!running_.load(std::memory_order_relaxed))
					{
						delete job.callback;
						job.callback = nullptr;
						continue;
					}

					{
						std::lock_guard<std::mutex> lock(jobs_mutex_);
						if (!running_.load(std::memory_order_relaxed))
						{
							delete job.callback;
							job.callback = nullptr;
							continue;
						}
						jobs_.push_front(std::move(job));
					}
					continue;
				}
				catch (...)
				{
					result.success = false;
				}

				{
					std::lock_guard<std::mutex> lock(results_mutex_);
					results_.push_back(std::move(result));
				}
			}
		}
	}

	void Init(int thread_count)
	{
		if (running_.load(std::memory_order_relaxed))
		{
			return;
		}

		if (thread_count <= 0)
		{
			const int hc = static_cast<int>(std::thread::hardware_concurrency());
			thread_count = std::clamp(hc - 1, 1, 4);
		}
		else
		{
			thread_count = std::clamp(thread_count, 1, 16);
		}

		thread_count_ = thread_count;
		running_.store(true, std::memory_order_relaxed);

		for (int i = 0; i < thread_count_; ++i)
		{
			workers_.emplace_back(WorkerThreadFunc);
		}
	}

	void Shutdown()
	{
		if (!running_.load(std::memory_order_relaxed))
		{
			return;
		}

		running_.store(false, std::memory_order_relaxed);

		// Discard pending queued jobs immediately so workers don't execute them during shutdown
		{
			std::lock_guard<std::mutex> lock(jobs_mutex_);
			for (auto& job : jobs_)
			{
				delete job.callback;
			}
			jobs_.clear();
		}

		cv_jobs_.notify_all();

		for (auto& t : workers_)
		{
			if (t.joinable())
			{
				t.join();
			}
		}
		workers_.clear();

		// Ensure jobs queue is empty after threads have joined
		{
			std::lock_guard<std::mutex> lock(jobs_mutex_);
			for (auto& job : jobs_)
			{
				delete job.callback;
			}
			jobs_.clear();
		}

		{
			std::lock_guard<std::mutex> lock(results_mutex_);
			for (auto& res : results_)
			{
				delete res.callback;
			}
			results_.clear();
		}
	}

	bool EnqueueJob(PathJob job)
	{
		if (!running_.load(std::memory_order_relaxed))
		{
			return false;
		}

		{
			std::lock_guard<std::mutex> lock(jobs_mutex_);
			if (jobs_.size() >= MAX_QUEUED_JOBS)
			{
				return false;
			}
			jobs_.push_back(std::move(job));
		}
		cv_jobs_.notify_one();
		return true;
	}

	void ProcessResults()
	{
		std::vector<PathJobResult> local_results;
		{
			std::lock_guard<std::mutex> lock(results_mutex_);
			if (results_.empty())
			{
				return;
			}
			local_results.swap(results_);
		}

		for (auto& res : local_results)
		{
			// Check if AMX instance is still loaded and has matching load serial
			if (!AmxHandler::IsAmxValid(res.amx, res.amx_serial))
			{
				delete res.callback;
				res.callback = nullptr;
				continue;
			}

			int path_id = INVALID_PATH_ID;
			if (res.success && !res.node_ids.empty())
			{
				std::shared_lock<std::shared_mutex> g_lock(Container::GetGraphMutex());
				bool valid = true;
				std::vector<Node*> valid_nodes;
				valid_nodes.reserve(res.node_ids.size());

				for (int nid : res.node_ids)
				{
					Node* n = Container::Nodes::FindIncludingPending(nid);
					if (n == nullptr || n->isSetForDeletion())
					{
						valid = false;
						break;
					}
					valid_nodes.push_back(n);
				}

				if (valid)
				{
					Path* path = Container::Paths::Register(valid_nodes, res.length, res.amx);
					path_id = path->getID();
				}
			}

			if (res.callback != nullptr)
			{
				res.callback->setResult(path_id);
				res.callback->call();
				delete res.callback;
				res.callback = nullptr;
			}
		}
	}

	void DiscardForAmx(AMX* amx)
	{
		{
			std::lock_guard<std::mutex> lock(jobs_mutex_);
			auto it = jobs_.begin();
			while (it != jobs_.end())
			{
				if (it->amx == amx)
				{
					delete it->callback;
					it = jobs_.erase(it);
				}
				else
				{
					++it;
				}
			}
		}

		{
			std::lock_guard<std::mutex> lock(results_mutex_);
			auto it = results_.begin();
			while (it != results_.end())
			{
				if (it->amx == amx)
				{
					delete it->callback;
					it = results_.erase(it);
				}
				else
				{
					++it;
				}
			}
		}
	}

	int GetThreadCount()
	{
		return thread_count_;
	}
}
