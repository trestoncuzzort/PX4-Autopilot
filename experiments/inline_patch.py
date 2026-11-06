#!/usr/bin/env python3
"""EXPERIMENT, not for landing yet: in lockstep, let the SIH thread run hrt callouts and all work items itself, so the
control path needs no thread wake-up at all. Gated by PX4_SIM_WQ_INLINE=1, on top of the single work queue."""
import sys
from pathlib import Path

root = Path(sys.argv[1])


def edit(path, *pairs):
    p = root / path
    s = p.read_text()
    for old, new in pairs:
        if old not in s:
            sys.exit(f"{path}: not found: {old[:70]!r}")
        s = s.replace(old, new, 1)
    p.write_text(s)


edit("platforms/common/include/px4_platform_common/px4_work_queue/WorkQueue.hpp",
     ("""	void Run();

""", """	void Run();

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	// Stop the worker thread running work, so that the time master can run it instead with RunPending().
	void HandOverToTimeMaster();

	// Run everything queued, on the calling thread, until the queue is empty.
	void RunPending();
#endif // ENABLE_LOCKSTEP_SCHEDULER

"""),
     ("""#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	int _lockstep_component {-1};
#endif // ENABLE_LOCKSTEP_SCHEDULER
""", """#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	int _lockstep_component {-1};
	px4::atomic_bool _run_by_time_master{false};
#endif // ENABLE_LOCKSTEP_SCHEDULER
"""))

edit("platforms/common/px4_work_queue/WorkQueue.cpp",
     ("""	if (_lockstep_component == -1) {
		_lockstep_component = px4_lockstep_register_component();
	}

#endif // ENABLE_LOCKSTEP_SCHEDULER

	_q.push(item);
	work_unlock();

	SignalWorkerThread();
""", """	const bool run_by_time_master = _run_by_time_master.load();

	// The time master runs this work itself before it can advance time, so there is nothing to wait for.
	if (_lockstep_component == -1 && !run_by_time_master) {
		_lockstep_component = px4_lockstep_register_component();
	}

	_q.push(item);
	work_unlock();

	if (run_by_time_master) {
		return;
	}

#else
	_q.push(item);
	work_unlock();
#endif // ENABLE_LOCKSTEP_SCHEDULER

	SignalWorkerThread();
"""),
     ("""		// process queued work
		while (!_q.empty()) {
			WorkItem *work = _q.pop();
""", """		// process queued work
#if defined(ENABLE_LOCKSTEP_SCHEDULER)

		while (!_q.empty() && !_run_by_time_master.load()) {
#else

		while (!_q.empty()) {
#endif // ENABLE_LOCKSTEP_SCHEDULER
			WorkItem *work = _q.pop();
"""),
     ("""void WorkQueue::print_status(bool last)
""", """#if defined(ENABLE_LOCKSTEP_SCHEDULER)
void WorkQueue::HandOverToTimeMaster()
{
	_run_by_time_master.store(true);
	SignalWorkerThread();

	// The worker may be inside a work item right now. Nothing else can start one until the time master does.
	while (_running_item.load() != nullptr) {
		system_usleep(50);
	}

	work_lock();
	px4_lockstep_unregister_component(_lockstep_component);
	_lockstep_component = -1;
	_runner_tid = pthread_self();
	work_unlock();
}

void WorkQueue::RunPending()
{
	work_lock();

	while (!_q.empty()) {
		WorkItem *work = _q.pop();
		_running_item.store(work);
		work_unlock();
		work->RunPreamble();
		work->Run();
		// Note: after Run() we cannot access work anymore, as it might have been deleted
		_running_item.store(nullptr);
		work_lock();
	}

	work_unlock();
}
#endif // ENABLE_LOCKSTEP_SCHEDULER

void WorkQueue::print_status(bool last)
"""))

edit("platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp",
     ("""/**
 * Start the work queue manager task.
 */
""", """#if defined(ENABLE_LOCKSTEP_SCHEDULER)
/** PX4_SIM_WQ_INLINE=1: the time master runs all work items on its own thread. Implies the single work queue. */
bool WorkQueueInlineRequested();

/** Called once by the time master, before its first time step, to take the work over from the worker thread. */
bool WorkQueueInlineStart();

/** Called by the time master to run all queued work on its own thread. */
void WorkQueueInlineRun();
#endif // ENABLE_LOCKSTEP_SCHEDULER

/**
 * Start the work queue manager task.
 */
"""))

edit("platforms/common/px4_work_queue/WorkQueueManager.cpp",
     ("""	static const char *const setting = getenv("PX4_SIM_WQ_SINGLE");
	static const bool use_single_queue = setting != nullptr && strcmp(setting, "1") == 0;

	return use_single_queue ? single_queue : requested_wq;
}
""", """	static const char *const setting = getenv("PX4_SIM_WQ_SINGLE");
	static const bool use_single_queue = (setting != nullptr && strcmp(setting, "1") == 0) || WorkQueueInlineRequested();

	return use_single_queue ? single_queue : requested_wq;
}

static WorkQueue *_inline_wq{nullptr};

bool WorkQueueInlineRequested()
{
	static const char *const setting = getenv("PX4_SIM_WQ_INLINE");
	static const bool requested = setting != nullptr && strcmp(setting, "1") == 0;
	return requested;
}

bool WorkQueueInlineStart()
{
	// any configuration: SelectWorkQueue maps all of them to the single queue
	_inline_wq = WorkQueueFindOrCreate(wq_configurations::hp_default);

	if (_inline_wq == nullptr) {
		return false;
	}

	_inline_wq->HandOverToTimeMaster();
	return true;
}

void WorkQueueInlineRun()
{
	_inline_wq->RunPending();
}
"""))

edit("platforms/posix/src/px4/common/drv_hrt.cpp",
     ("""static void hrt_latency_update();
""", """static bool _hrt_run_by_time_master{false};

static void hrt_latency_update();
"""),
     ("""	// There is no timer ISR, so simulate one by putting an event on the
	// high priority work queue
""", """	if (_hrt_run_by_time_master) {
		return;
	}

	// There is no timer ISR, so simulate one by putting an event on the
	// high priority work queue
"""),
     ("""int px4_lockstep_register_component()
""", """void px4_lockstep_run_timers()
{
	if (!_hrt_run_by_time_master) {
		_hrt_run_by_time_master = true;
		hrt_work_cancel(&_hrt_work);
	}

	hrt_tim_isr(nullptr);
}

int px4_lockstep_register_component()
"""))

edit("src/drivers/drv_hrt.h",
     ("""__EXPORT extern int px4_lockstep_register_component(void);
""", """__EXPORT extern void px4_lockstep_run_timers(void);
__EXPORT extern int px4_lockstep_register_component(void);
"""),
     ("""static inline int px4_lockstep_register_component(void) { return 0; }
""", """static inline void px4_lockstep_run_timers(void) {}
static inline int px4_lockstep_register_component(void) { return 0; }
"""))

edit("src/modules/simulation/simulator_sih/sih.cpp",
     ("""#include <lib/drivers/device/Device.hpp>
""", """#include <lib/drivers/device/Device.hpp>

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#endif
"""),
     ("""	uint64_t pre_compute_wall_time_us;

	while (!should_exit()) {
		pre_compute_wall_time_us = micros();
		perf_count(_loop_interval_perf);

		_current_simulation_time_us += sim_interval_us;
		struct timespec ts;
		abstime_to_ts(&ts, _current_simulation_time_us);
		px4_clock_settime(CLOCK_MONOTONIC, &ts);

		perf_begin(_loop_perf);
		sensor_step();
		perf_end(_loop_perf);
""", """	uint64_t pre_compute_wall_time_us;

	const bool run_work_inline = px4::WorkQueueInlineRequested() && px4::WorkQueueInlineStart();

	if (run_work_inline) {
		PX4_INFO("Running timers and work items on the simulator thread");
	}

	while (!should_exit()) {
		pre_compute_wall_time_us = micros();
		perf_count(_loop_interval_perf);

		_current_simulation_time_us += sim_interval_us;
		struct timespec ts;
		abstime_to_ts(&ts, _current_simulation_time_us);
		px4_clock_settime(CLOCK_MONOTONIC, &ts);

		if (run_work_inline) {
			px4_lockstep_run_timers();
		}

		perf_begin(_loop_perf);
		sensor_step();
		perf_end(_loop_perf);

		if (run_work_inline) {
			px4::WorkQueueInlineRun();
		}
"""),
     ("""					  SCHED_PRIORITY_MAX,
					  1560,
""", """					  SCHED_PRIORITY_MAX,
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
					  px4::WorkQueueInlineRequested() ? 60000 : 1560,
#else
					  1560,
#endif
"""))
print("patched")
