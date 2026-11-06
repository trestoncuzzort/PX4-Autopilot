# Fibers, deterministic multithreading, run-to-completion executors, virtual-time thread simulators: what could remove PX4 SIH's thread wake-ups

Date of research: 2026-11-06. Read-only. Nothing built, nothing run, no clones. Fetched text was treated as data.
Web search budget ran out part-way (shared limit of 200). Items found only through a search summary are marked "UNVERIFIED" and are not used in the conclusions.

PX4 pin used everywhere below: 95a0c23482329183d932ac99da00faa7e8d63143 (PX4 is BSD-3-Clause; GPL or LGPL code below is idea-only).
PX4 base URL: https://raw.githubusercontent.com/PX4/PX4-Autopilot/95a0c23482329183d932ac99da00faa7e8d63143/

---------------------------------------------------------------------
## 0. What PX4 does today (read first; the rest is judged against this)

Facts from the pinned files:

* The SIH loop sets the clock, then waits for work queues to go idle.
  `src/modules/simulation/simulator_sih/sih.cpp` 103-185: each step does `px4_clock_settime` (128-135), `sensor_step()`, then (after the first actuator output) `px4_lockstep_register_component()` / `px4_lockstep_progress()` / `px4_lockstep_wait_for_components()` (153-158), then a poll loop `while (!_actuator_out_sub.updated()) usleep(100)` (166-173), then `usleep(sleep_time)` for real-time pacing (181).
* Setting the clock wakes every due sleeper one by one.
  `platforms/posix/src/px4/common/lockstep_scheduler/src/lockstep_scheduler.cpp` 50-128 (`set_absolute_time`): takes `_signaling_mutex`, walks the `_timed_waits` list, and for each due entry does `pthread_mutex_lock(passed_lock); pthread_cond_broadcast(passed_cond); pthread_mutex_unlock` (113-120). One futex wake per due waiter, all issued from the SIH thread. That is the `futex_wake` in the profile.
* Each waiter is a pthread mutex plus cond: `cond_timedwait` 130-196, `usleep_until` 198-215. Line 134: `static thread_local TimedWait timed_wait;` (matters for fibers, section 2).
* Work queue hand-off is a mutex+cond "semaphore". `platforms/posix/src/px4/common/px4_sem.cpp` 78-102 (`px4_sem_wait`: lock, `pthread_cond_wait`), 163-189 (`px4_sem_post`: lock, `pthread_cond_signal` when value <= 0). `platforms/common/px4_work_queue/WorkQueue.cpp` 135-151 (`Add` pushes and calls `SignalWorkerThread`), 164-171 (post if sem value <= 0), 191-231 (`Run`: `px4_sem_wait`, drain queue, run items).
* The HRT "timer interrupt" is itself a work item on a work queue. `platforms/posix/src/px4/common/drv_hrt.cpp` 254-327: `hrt_tim_isr` runs callouts under a recursive mutex; `hrt_call_reschedule` re-arms it with `hrt_work_queue(&_hrt_work, &hrt_tim_isr, nullptr, delay)` (320-326).
* Work queue threads are real pthreads at SCHED_FIFO priorities. `platforms/common/px4_work_queue/WorkQueueManager.cpp` 305-346 (`sched_get_priority_max(SCHED_FIFO) + wq->relative_priority`, `pthread_attr_setschedpolicy(SCHED_FIFO)`, `pthread_create`), max 16 threads (line 75).
* Quiescence in PX4 today is "every registered component has made progress", not "every thread is blocked".
  `platforms/posix/src/px4/common/lockstep_scheduler/src/lockstep_components.cpp` 60-137: an int bitset of registered ("used") components and a bitset of "progress". `register_component` 60-79, `unregister_component` 81-96 (releases the waiter when the remaining set has all made progress), `lockstep_progress` 98-116, `release_waiter` 118-128 (posts a px4_sem if its value < 1), `wait_for_components` 130-137 (returns at once if nothing is registered, otherwise `px4_sem_wait`).
  A work queue registers when an item is added (`WorkQueue.cpp` 139-145) and unregisters when its queue is empty after a drain (218-225). SIH holds its own component so idle queues cannot release the barrier early (sih.cpp 150-158 comment).
  In the files I read, only WorkQueue and SIH call these functions. I could not search the whole repo (GitHub code search returned 401 and I was told not to clone). Plain tasks and non-work-queue threads are therefore probably not tracked.
  Consequence: a work item that hands work to a plain task, to the MAVLink thread, or to a real-time sleeper is invisible to the barrier. That is a source of run-to-run difference today, independent of the wake-up cost.
* Threads that block in real system calls (all at the pin):
  * MAVLink receiver: `src/modules/mavlink/mavlink_receiver.cpp` 4016 `poll(&fds[0], 1, timeout)`, 4021 `::read`, 4034 `recvfrom` (UDP).
  * MAVLink main uses wall-clock `usleep` on purpose: `src/modules/mavlink/mavlink_main.cpp` 1491-1495 comment "px4_usleep in lockstep mode sleeps in sim time which can stall startup" (also 1516). So PX4 already mixes sim-time sleeps and wall-time sleeps; any replacement must keep both.
  * Logger: `src/modules/logger/log_writer_file.cpp` 332-345 `pthread_cond_wait(&_cv,&_mtx)` and file `write`/`fsync` on that thread (354 onward).
  * Shell/daemon: `platforms/posix/src/px4/common/px4_daemon/server.cpp` 160 `poll(..., -1)`, 175 `accept`, 257 `read`.
  * (Not used by SIH but present in the SITL tree: `simulator_mavlink/SimulatorMavlink.cpp` 1241 `px4_poll`, 1338/1433 `recvfrom`, 1418 `::poll`.)

Profile numbers given in the brief (not mine, not re-measured): 19.66% of samples in `futex_wake` on the SIH thread, about 600,000 context switches in a 10 s run, system CPU 2.6 times user CPU, one run, WSL2.

---------------------------------------------------------------------
## 1. How the "quiescence" question is answered in the systems that do virtual time

There are four distinct answers. PX4's bitset is a fifth, weaker one.

| System | How it knows nothing else can run | Where |
|---|---|---|
| Zephyr native_sim | By construction: exactly one embedded thread runs, others wait on private semaphores. When Zephyr's idle path runs, the CPU "halts" and the hardware-model thread advances simulated time to the next event. | section 5.1 |
| Shadow (v2) | Runs a managed thread until it makes an interposed blocking syscall; "time only moves forward when Shadow is blocked on such a call". | section 4.1 |
| Shadow v1 + GNU Pth | A "Shadow thread" at lowest priority in a user-level scheduler. It only gets the CPU when every other user thread is blocked; then it advances the clock. | section 2.7 |
| Hermit/Detcore | The scheduler "waits for quiescence" (all guest threads parked with a pending request) before each turn. | section 1.9 |
| Parrot | Own run queue and wait queue; when all threads are on the wait queue it starts an idle thread that "keeps the logical time flowing". | section 1.3 |
| PX4 today | Bitset of work queues with non-empty queues plus SIH's own bit. Not covering other threads. | section 0 |

---------------------------------------------------------------------
## 2. User-level threads and fibers

Reference switch costs (all with their own hardware, not comparable across rows):

| Item | Number | Hardware | Source |
|---|---|---|---|
| Boost.Context fcontext switch | 9 ns / 19 CPU cycles | dual Intel Xeon E5 2620v4 2.2 GHz, 16C/32T, 64 GB, Linux x86_64, gcc 6.3.1 | Boost 1.83 context docs, "Performance" table 1.1 |
| Boost.Context ucontext_t switch | 547 ns / 1130 CPU cycles | same | same |
| Boost.Context Windows-Fiber | 49 ns / 98 cycles | same (page does not say which OS ran it) | same |
| Pthread switch via pipe or condvar, pinned to one CPU | 1.2 to 1.5 microseconds | Haswell i7-4771 | Eli Bendersky, 2018 article |
| same, not pinned | about 2.2 microseconds | same | same |
| Go goroutine switch | about 170 ns | same | same |

URLs: https://www.boost.org/doc/libs/1_83_0/libs/context/doc/html/context/performance.html ; https://eli.thegreenplace.net/2018/measuring-context-switching-and-memory-overheads-for-linux-threads/

My arithmetic, NOT a measurement, hardware differs from the PX4 profile machine: 600,000 switches x 1.2 us = 0.72 s, x 1.5 us = 0.90 s, x 2.2 us = 1.32 s of a 10 s run if each switch cost that much. The same count at 9 ns would be 5.4 ms; at 547 ns, 0.33 s. These are only to size the idea. They omit the cost of the futex_wake syscall on the waker side, which the profile shows is the larger part (19.66% of SIH samples).

### 2.1 Boost.Fiber / Boost.Context (BSL-1.0, header-heavy C++)
* Docs: https://www.boost.org/doc/libs/1_83_0/libs/fiber/doc/html/fiber/overview.html (read), .../scheduling.html (read), .../performance.html (read). Not commit-pinned (documentation pages for release 1.83).
* Mechanism: cooperative; "The running fiber decides explicitly when it should yield". One scheduler per thread, `round_robin` default (ready queue, head out, tail in).
* The scheduler interface has exactly the hook a virtual clock needs: `suspend_until(time_point)` tells the scheduler "no fiber will be ready before" that time; it "need not return until abs_time" or `notify()`. A custom `algorithm` could, instead of sleeping, hand control to a virtual-clock driver. For `round_robin` it blocks in `std::condition_variable::wait_until`.
* Blocking: "a fiber that invokes a normal blocking I/O operation will block its entire thread", and so does any normal thread synchronization (docs overview). The docs advise "consistently use asynchronous I/O". The page I read did not mention fiber-aware mutex/condition_variable by name, only that Boost.Fiber's own sync objects work across threads by default.
* Measured (docs performance page, same dual Xeon E5 2620v4): per-fiber cost in a 1,000,000-fiber spawn benchmark, 1C/1T round-robin 1.69 to 1.79 microseconds, 16C/32T work stealing 0.05 to 0.09 microseconds; per pthread 54 to 73 microseconds. This benchmark includes creation, so it is not a pure switch time.
* Determinism: `round_robin` is FIFO so order is repeatable on one thread; `work_stealing` victim is random.
* For PX4: gives the switch primitive and the idle hook. Does nothing about real syscalls.

### 2.2 marl (Google, Apache-2.0)
* Commit b8406ab0a82524170ead6dd6c371b579bbc28b3e (pushed 2026-04-27).
* `include/marl/scheduler.h` 41-45: "Scheduler are initially constructed in single-threaded mode", `bind()` ties a scheduler to the calling thread (109-112), `Config::WorkerThread::count = 0` (62). Default fiber stack 1 MiB (57).
* Time type is `std::chrono::system_clock::time_point` (line 50). `ConditionVariable::wait_for/wait_until` exist (conditionvariable.h 53-70). A virtual clock would need a patch.
* Blocking system calls: `include/marl/blockingcall.h` 29-53 and 97-102. `marl::blocking_call(f)` spawns a new `std::thread`, binds it to the scheduler, runs `f` there, and the calling fiber waits on a `WaitGroup` while other fibers run. So the answer to "a fiber calls recvfrom" is "run that call on a helper OS thread". Cost: one thread creation per call (Boost docs above put pthread creation at 54 to 73 microseconds on their hardware), so use it only for rare calls.
* No switch-cost figure found in the files I read (I did not run or read its benchmarks).

### 2.3 Apollo Cyber RT coroutines (Apache-2.0)
* Commit d53aa3da47a06a08e6d0cd175d5623a34fa0d6aa. `cyber/croutine/croutine.h` 38 states `READY, FINISHED, SLEEP, IO_WAIT, DATA_WAIT`; 125-133 `Yield` is `SwapContext(...)`; `croutine.cc` 64 `MakeContext(CRoutineEntry, ...)`; hand-written swap in `cyber/croutine/detail/swap_x86_64.S` and `swap_aarch64.S` (exist in the tree; I did not read them). Sleep wake time uses `std::chrono::steady_clock` (croutine.h 102-103), real time.
* This is a production example (a robotics middleware) of many logical tasks on few threads with an explicit "waiting for data" state, i.e. close to PX4's work items waiting on uORB. I found no latency numbers for it in what I read.

### 2.4 NuttX sim and RIOT native (user-level switch inside one host thread)
* NuttX sim (Apache-2.0), commit 43f7f34b9958283b4403789c9bc21df920fd3143. `arch/sim/src/sim/sim_switchcontext.c` 58-90: `up_switch_context` saves registers with `sim_saveusercontext` and restores with `sim_restorestate`, no host thread hand-off. `sim_idle.c` 57-94 is the idle hook. Time comes from the host: `arch/sim/src/sim/posix/sim_hosttime.c` 74 and 99 (`clock_gettime(CLOCK_MONOTONIC)`, `timer_create(CLOCK_MONOTONIC,...)`), so it is real time, not virtual.
* RIOT native (LGPL-2.1, idea only), commit 9c9a6abec901838b1637ef9308427b50ef59ce5c. `cpu/native/cpu.c` 13-14 "In-process preemptive context switching utilizes POSIX ucontexts"; `thread_yield_higher` 177-194 uses `swapcontext`. Host I/O is polled from a signal-driven ISR context: `cpu/native/async_read.c` 28-38 (`real_poll(_fds, _next_index, 0)` with zero timeout). Real time again.
* Both prove the pattern: an RTOS's tasks mapped onto ONE host thread with user-level switches, host I/O polled non-blockingly. Neither gives virtual time or determinism by itself; their clocks are the host's.

### 2.5 FreeRTOS POSIX port (MIT) - the opposite pattern
* Commit ca759444e9ca59cfa3a9faa6cb8b49e0772ca520, `portable/ThirdParty/GCC/Posix/port.c`: one pthread per task (comment at 40: tick uses SIGALRM), resume by `SIG_RESUME` = SIGUSR1 (80), suspend "by waiting for a pthread_cond_signal event" (607), tick generated by a real timer (278-288) and `pthread_kill`+`usleep` (467-468). Each task switch is a condvar signal plus wait, the same futex cost as PX4. Not a speed precedent.

### 2.6 Others in the brief: not read
GNU Pth source itself, State Threads, libco (repo description only: "widely used in wechat back-end service"; license field NOASSERTION), libfiber, folly fibers (Apache-2.0 per GitHub), C++20 coroutines. I did not open any of their code or benchmarks, so no numbers and no claims about their blocking behaviour.

### 2.7 GNU Pth under a simulator: the closest user-level precedent for the quiescence trick
* Miller and Jansen, "Shadow-Bitcoin: Scalable Simulation via Direct Execution of Multi-threaded Applications", USENIX CSET 2015, section 3. https://www.usenix.org/system/files/conference/cset15/cset15-miller.pdf (read, text extracted locally).
* They run a multi-threaded application (bitcoind) as a Shadow plug-in using Pth (GNU Portable Threads: cooperative user-space threads, a substitute pthreads API and POSIX I/O API). Pth's I/O wrappers make fds non-blocking and yield with the awaited event instead of blocking.
* Quiescence: "we ensure that the 'Shadow thread' is always available to run, but assign it the lowest-priority value so that it is only activated when every other thread is blocked". When it runs it translates the other threads' waits into epoll requests and returns to the simulator, which advances time. Pth's own blocking `select` is bypassed.
* Cost of maintaining it: Phantom paper (below, section 3.1 "Transforming Shadow") says they removed 47,959 LoC (50.9% of 94,259 LoC in Shadow v1.14.0) "containing a custom version of the GNU portable threads library". That is the price of owning a pthread replacement.
* For PX4: this is the exact mechanism needed for "one SIH thread owns the clock". The idle fiber is the SIH time driver.

---------------------------------------------------------------------
## 3. Deterministic multithreading (DMT): what is guaranteed, what it costs

Short reading for PX4: none of these remove wake-ups. They add ordering control on top of real threads. The only ones whose design collapses threads are the process-per-thread systems, and they are unusable for PX4 (shared uORB memory). Details:

### 3.1 Parrot (Cui et al., SOSP 2013)
* Paper https://www.cs.columbia.edu/~junfeng/papers/parrot-sosp13.pdf (read, locally extracted). Code: github.com/columbia/smt-mc (commit ae1ff3722a0220f33bc9e33f7a8e6fec445a753b; GitHub license field is empty; code lives in the `xtern` submodule which I did not fetch; I found no license text, so treat as unlicensed = idea only).
* Guarantee: synchronization order is deterministic round-robin ("deterministic synchronizations") and "stable"; data races stay nondeterministic ("PARROT makes synchronizations outside performance critical sections deterministic but allows nondeterministic data races"). Not strong determinism.
* Mechanism: LD_PRELOAD wrappers around pthreads. A thread can enter a synchronization only when it is head of the run queue (`get_turn`/`put_turn`, Table 1 and sections 4.1-4.2). `get_turn` "spin-waits on the current thread's flag for a while before block-waiting on the condition variable"; spin-wait set to 105 cycles (section 7, setup). Real threads keep running in parallel between synchronizations.
* Time: section 4.5, timeouts become logical: real timeout `tr` becomes `tr/R` turns, R defaults to 3 microseconds. "When all threads are on the wait queue, PARROT spawns an idle thread to keep the logical time flowing. The thread repeatedly gets the turn, sleeps for time R, and gives up the turn." Fast-forwarding was tried and found slower because it "often wakes up threads prematurely before the pending external events ... are done". The paper says logical time "may be less useful for real-time applications".
* Blocking calls: section 4.4. Before a blocking `recv`-like call it calls `nondet_begin`, handing the thread to the OS scheduler; `nondet_end` on return. 33 network operations are wrapped. Those periods are nondeterministic by construction.
* Overhead: abstract and section 1: 6.9% geometric mean over 55 real-world programs, 12.7% over all 108, at 16-24 cores of a "24-core server" (exact CPU not stated in the text I extracted). On the 25 programs that DTHREADS and CoreDet can both run: Parrot 11.8%, DTHREADS 150.0%, CoreDet 115.1%. 5 programs with ad hoc busy-wait loops needed a `sched_yield` added.
* Fit for PX4: determinism of sync order, with a real-time pacing hole for every real blocking call (MAVLink poll, daemon accept). It does not reduce futex_wake; a hand-off is still a flag/condvar wake. Would not speed up SIH.

### 3.2 DTHREADS (Liu, Curtsinger, Berger, SOSP 2011)
* Paper https://people.cs.umass.edu/~emery/pubs/dthreads-sosp11.pdf (read, locally extracted; server needed `-k` for TLS). Code: github.com/plasma-umass/dthreads, GPL-2.0, last push 2015-09-24 (GitHub API). Idea only.
* Guarantee: full determinism including data races, by running each thread as a separate process with copy-on-write private memory, committing changes in a deterministic order at synchronization points ("serial phase", token passing).
* Cost: Xeon 2.33 GHz, two 4-core, 16 GB, CentOS 5.5. "outperforms CoreDet on 12 out of 14 benchmarks (between 14% and 11.2x faster)"; "runs substantially slower than pthreads for 4 of the 14". Process context switches cost more than thread switches (TLB). Does not enforce deterministic arrival of I/O events.
* Would break PX4 SITL: uORB, parameters, perf counters and the lockstep state are shared memory read without synchronization points. Isolation plus commit-at-sync would hide updates until a sync. Rejected.

### 3.3 Kendo (Olszewski, Ansel, Amarasinghe, ASPLOS 2009)
* Paper https://people.csail.mit.edu/mareko/asplos073-olszewski.pdf (read). Source code: not found.
* Guarantee: weak determinism only (deterministic lock acquisition order, for race-free programs). Logical clock from a hardware retired-stores counter, which needed a kernel modified to expose performance counter interrupts.
* Cost: "geometric mean overhead of only 16% when running on a 4-core processor" (SPLASH-2, 2.66 GHz Core 2 Quad, kernel 2.6.23, four threads); worst case about 53% (radiosity).
* Not usable: needs a patched 2009 kernel and performance-counter interrupts (whether WSL2 exposes counters to a guest is unchecked, see section 9).

### 3.4 CoreDet (Bergan et al., ASPLOS 2010)
* Paper https://homes.cs.washington.edu/~luisceze/publications/asplos103-bergan2.pdf (read). Code github.com/uwsampa/coredet: no license field, last push 2015-06-11 (GitHub API), LLVM compiler pass.
* Guarantee: strong determinism (including races) via instruction-count quanta, serial and parallel phases, ownership or buffering. I/O timing is explicitly left nondeterministic ("the timing of asynchronous I/O remains a source of nondeterminism").
* Hardware in paper: 8-core 2.4 GHz Xeon E5462, 10 GB, Ubuntu 8.10. Overhead only shown in Figure 7; I did not extract numbers.
* Not usable: recompile whole PX4 with a 2010-era LLVM pass.

### 3.5 Determinator (Aviram et al., OSDI 2010)
* Paper https://www.usenix.org/legacy/event/osdi10/tech/full_papers/Aviram.pdf (read). Clean-slate microkernel; "shared-nothing" address spaces and only three system calls. Not usable as it is an OS. Useful statement for design: it denies "realtime clocks, cycle counters, and writable shared memory" to get determinism, and treats timers as explicit I/O devices. Coarse-grained benchmarks "comparable" to Linux; a "deterministic scheduler's quantization ... incurs a fixed performance cost of about 35%" for blackscholes.

### 3.6 Grace, RFDet, Consequence
* Not reached. Grace is described in the DTHREADS paper as "threads-as-processes" with speculation that "rules out all interthread communication, including ... condition variables, barriers" (DTHREADS section 2) - which already excludes PX4. RFDet and Consequence: I only have a search-result snippet for Consequence ("19 benchmarks, worst-case 3.9x vs pthreads, 14 of 19 at or below 2.5x") that I did not verify at the source page. UNVERIFIED, not used.

### 3.7 Hermit / Detcore (Meta, Rust, BSD-3-Clause; newest and the only active one)
* Repo facebookexperimental/hermit, commit d552dbe0944fcdb6c798a8fab57d58f620178c95 (pushed 2026-11-05). LICENSE is BSD 3-Clause with "Copyright 2020-2022 (c) Meta Platforms" (GitHub's detector says NOASSERTION because of the header paragraph). README: x86-64 Linux only, ptrace + seccomp via Reverie, "User-space performance counters for precise scheduler preemption. Hermit can run without them, but CPU-bound workloads receive fewer preemption points."
* `docs/ARCHITECTURE.md` 520-583: with `sequentialize_threads`, each ordered operation sends `RequestResources` to the scheduler; the scheduler "Wait[s] for quiescence; advance[s] scheduler logical time", processes timers, futex wakes, signals, blocked I/O, then picks the next runnable TID (deterministic run queue: priority, then round robin) and commits one turn at a time. "The current implementation deliberately checks for broad quiescence before advancing a turn. This is conservative and limits parallelism".
* 585-598: spinning threads are preempted by counting retired conditional branches (RCB) with a timer; needs the PMU.
* 763-771: internal blocking I/O is made non-blocking and retried at deterministic turns; "For an endpoint outside the container ... `BlockingExternalIO` and `BlockedExternalContinue` to avoid deadlocking all guest threads while the host event is pending. The exact completion time of such an external event is not made deterministic by scheduling alone."
* 773-800: virtual time is `epoch + multiplier * (10,000 ns * syscalls + 10 ns * retired conditional branches + 25 ns * CPUID/RDTSC events)`; "The constants are model parameters, not measurements".
* No overhead numbers found in the files I read. Every guest syscall is a ptrace stop (Phantom paper below says ptrace needs at least 4 context switches per syscall), so this would add wake-ups, not remove them.
* Value for PX4: not as a tool to run, but as the cleanest written statement of how to do "turn-based, quiescence-before-clock-advance" and what has to be said about external I/O (replay the result, accept loss of determinism).

---------------------------------------------------------------------
## 4. Simulators that run real multithreaded programs in virtual time

### 4.1 Shadow v2 / "Phantom" (BSD-3-Clause style license, Rust + C)
* Repo shadow/shadow, commit b8e8ef680198b027b45d607938d4e60b98e42def (pushed 2026-11-05). LICENSE (lines 1-22) is the three-clause BSD text (copyright Rob Jansen; GitHub reports NOASSERTION because of the US-government paragraph).
* Paper: Jansen, Newsome, Wails, "Co-opting Linux Processes for High-Performance Network Simulation", USENIX ATC 2022. https://www.usenix.org/system/files/atc22-jansen.pdf (WebFetch could not decode it; I saved the PDF and extracted text locally with pdftotext).
* How threads are serialized (paper section 3.2.7; `src/main/host/managed_thread.rs` 187-321 `resume` loop, 427-463 `continue_plugin`): each managed thread waits on a shared-memory channel. Shadow sends one message, the thread runs until it hits an interposed syscall, sends back a syscall message, and waits. "each Phantom worker only allows a single thread of execution across all processes it manages; each of the remaining managed processes/threads will always be idle". A blocking syscall that cannot complete returns `SyscallReturn::Block` (managed_thread.rs 285-289); Shadow registers a condition ("a packet to arrive or a timeout to occur") and "continues executing simulation events (and advancing simulation time)".
* Quiescence rule, in the docs: `docs/limitations.md` 68-83: "Shadow runs each thread of managed processes until it's blocked by a syscall such as nanosleep, read, select, futex, or epoll. Likewise, time only moves forward when Shadow is blocked on such a call - Shadow effectively models the CPU as being infinitely fast." Consequence stated there: busy loops "deadlock the whole simulation". Workaround `--model-unblocked-syscall-latency` (lines 92-120) advances time a little on every syscall, vDSO call and rdtsc and switches threads, which "forces Shadow to switch between threads more frequently, which is costly".
* Interception: LD_PRELOAD shim for libc symbols plus a seccomp filter for raw syscalls (docs/design_2x.md; paper section 3.2.4). Time reads are served from shared memory in the shim (paper section 3.2.5), so `clock_gettime` costs no context switch.
* The hand-off itself is still a futex wake. `src/lib/vasi-sync/src/scchannel.rs` 179-210 (`send`: if the receiver set `has_sleeper`, `futex_wake_one`), 219-294 (`receive`: sets `has_sleeper`, `futex_wait`). No spinning in this file. Paper Appendix B.5: they prototyped "never making a syscall to perform IPC ... busy-wait" and discarded it because "this effectively halved the maximum concurrency".
* Context-switch benchmark (paper Appendix C, Figure 16): semaphores, message queues, Unix sockets and atomic flag, same core vs cross core vs cross NUMA, "average context-switch round-trip time taken over 100k repeated trials" on two 14-core Intel Xeon E5-2697 2.6 GHz, CentOS 7, kernel 5.11.6-1. The numbers are only in the bar chart; the text I extracted has none. Text conclusion: "semaphores are the most efficient synchronization method when the processes are pinned to the same core"; "Cross-core and cross-node context switching is significantly more expensive than same-core". CPU pinning is part of Shadow's design.
* Other paper numbers (not about switches): 1k hosts, 10 s of simulated time, Phantom 2.2x faster than Shadow v1 and 3.4x faster than NS-3 (section 5.4).
* PX4 consequences:
  * Shadow's model fits PX4 exactly in one respect: threads that sleep or wait on interposed calls cost nothing while the clock moves. But its serialized hand-off costs one futex round trip per syscall, so for PX4 it would be at best equal to today's cost.
  * Shadow models network syscalls; a thread doing real UDP `recvfrom` to QGroundControl would have to be a simulated socket. PX4 uses the real network in SITL.
  * Not usable directly: it runs processes (PX4 SIH is one process with threads sharing memory; Shadow handles threads via clone interception, but this is for Tor-like network apps), needs Linux, seccomp, and its own simulated network stack.

### 4.2 rr (record and replay; MIT-style license)
* Paper: O'Callahan et al., "Engineering Record And Replay For Deployability", USENIX ATC 2017, https://arxiv.org/pdf/1705.05937 (read, locally extracted). Repo rr-debugger/rr: license text is the MIT-style permission paragraph (Mozilla/VMware/Google copyright); GitHub says NOASSERTION.
* Scheduling: "avoiding non-deterministic data races by running only one thread at a time" and "RR preemptively schedules these threads, so context switch timing is nondeterminism that must be recorded." Preemption by CPU performance counters (retired conditional branches) delivering a signal at the same instruction count on replay.
* It does not need a quiescence rule because it has no virtual time: wall-clock time is recorded, not simulated. A blocking syscall lets another thread run (scratch buffers avoid races with kernel writes, section 2.2). Threads that block are detected with `PERF_COUNT_SW_CONTEXT_SWITCHES` events (section 3).
* Measured on a Dell XPS15 laptop, quad-core Intel Skylake (8 SMT threads), 16 GB, SSD, Fedora 23; workloads cp, make, octane, htmltest, sambatest (Table 1, geometric mean of 5 warm runs): "Single core" (all threads pinned to one core with taskset, no rr): 0.98x, 3.36x, 1.36x, 1.07x, 0.95x. Record: 1.49x, 7.85x, 1.79x, 1.49x, 1.57x for cp, make, octane, htmltest, sambatest (from the extracted Table 1 columns; make is the many-process outlier). Record with in-process syscall interception disabled: 24.53x, 11.32x, 2.65x, 4.66x, 2.23x. Text: "Excluding make, RR's recording slowdown is less than a factor of two."
* Reading the "Single core" row: for low-parallelism programs, forcing everything onto one CPU cost about 5% savings to 7% extra (cp 0.98, htmltest 1.07, sambatest 0.95) and for a CPU-heavy parallel one 1.36x. Those programs are not PX4 and the machine is a laptop, so this is a hint, not a prediction.
* PX4 use: `rr record` on a SITL run to get an identical replay for debugging is plausible (single-core serialization is built in). It does not speed anything up and replays one recorded run. It needs hardware performance counters; whether WSL2 exposes them is unknown to me and I did not test it (the rr paper itself lists the CPU constraints).

---------------------------------------------------------------------
## 5. Firmware simulators that collapsed or serialized threads

### 5.1 Zephyr native_sim (Apache-2.0): closest precedent for determinism, not for fewer wake-ups
Pin: zephyrproject-rtos/zephyr 25851fa15cdc2dc9c395807bce97fa81c46c10ee. Native simulator core: `scripts/native_simulator/common/src/`.
* One host pthread per Zephyr thread, one allowed to run at a time. `nct.c` 12-59 design comment: "The embedded OS threads are run as a set of native Linux pthreads. The embedded OS only sees one of this threads executing at a time." Each has a private semaphore (101). Swap: `nct_let_run` sets `currently_allowed_thread` and `sem_post(&next->sema)` (199-207); `nct_swap_threads` does that and then `nct_wait_until_allowed`, which `sem_wait`s its own semaphore (173-194, 224-244). So every Zephyr context switch is one futex wake plus one futex sleep, the same kernel traffic as PX4's cond signal/wait. Zephyr documents it: "Whenever the Zephyr kernel desires to context switch two threads, the POSIX arch, using NCT, blocks and unblocks the corresponding pthreads." (`boards/native/doc/arch_soc.rst` 394-399).
* What it gains is determinism and quiescence for free:
  * The CPU is "infinitely fast": no simulated time passes while code runs (arch_soc.rst 80-90).
  * Time advances only when the Zephyr idle path halts the CPU: `arch/posix/core/cpuhalt.c` 36-43 `arch_cpu_idle()` calls `posix_halt_cpu()`; `nce.c` 185-199 `nce_halt_cpu` does `sem_post(sem_hw)` then waits on `sem_sw`; the HW thread then runs `nsi_hws_one_event()` (`nsi_hw_scheduler.c` 123-134), which finds the earliest event among all HW models, sets `nsi_simu_time = next_timer_time` and runs its callback (75-94, 101-112); `nce_wake_cpu` (209-232) wakes the SW side again. That halt/wake pair is two more futex operations per advance.
  * No wall clock link unless asked: native_sim docs (`boards/native/native_sim/doc/index.rst` 238-280): "simulated time runs fully decoupled from the real host time and as fast as the host compute power would allow"; optional `--rt`, `--rt-ratio`, `--rt-drift` waits "before raising the next system tick interrupt"; "normal runs are still deterministic".
  * Documented determinism: "you can execute the code multiple times and get the exact same result" (index.rst 104-107).
* Limits Zephyr states (arch_soc.rst 96-150): busy-wait loops "will stall the execution of all threads, kernel, and HW models". Workaround: `k_cpu_idle()` or `k_busy_wait()` in the loop. `arch_busy_wait` is rebuilt as a timer plus CPU halt (`boards/native/native_sim/cpu_wait.c` 37-49).
* How Zephyr handles host blocking I/O: it does not. Drivers poll in a Zephyr thread that sleeps in simulated time: `drivers/serial/uart_native_pty.c` 388-423 (`native_pty_uart_async_poll_function`, `k_sleep(K_USEC(CONFIG_UART_NATIVE_PTY_RX_POLL_PERIOD_US))`, reads via `nsi_host_read` at 289) and `drivers/ethernet/eth_native_tap.c` 285 (`nsi_host_read`) and 323 (`k_sleep(K_MSEC(CONFIG_ETH_NATIVE_TAP_RX_TIMEOUT))`). Host input is therefore sampled at simulated-time polling periods, a deterministic function of sim time but not of when the bytes actually arrived. I did not verify that the fds are non-blocking.
* Applied to PX4: adopting Zephyr's token scheme with pthreads keeps the cost and adds token bookkeeping; adopting its "idle halts the CPU, HW models decide the next time" structure on a single user-level thread is the real prize. In PX4 terms: lockstep SIH is already the HW model; what is missing is that "idle" is known by construction rather than by the work-queue bitset.

### 5.2 NuttX sim, RIOT native, FreeRTOS POSIX
See sections 2.4 and 2.5. Summary: NuttX sim and RIOT native switch tasks in one host thread with saved contexts and use the host's real clock; FreeRTOS POSIX uses one pthread per task with condvar hand-off and real timers. None gives virtual time.

---------------------------------------------------------------------
## 6. Run-to-completion executors and what collapsing threads saved

### 6.1 ROS 2 executors
* The StaticSingleThreadedExecutor no longer exists: PR "Removed deprecated StaticSingleThreadedExecutor" (ros2/rclcpp #2835) merged 2025-05-12 (GitHub API). I could not get its latency papers (web search budget gone; one guessed arXiv id was wrong). No figures from papers.
* EventsExecutor (iRobot; ros2/rclcpp PR #1416, closed, https://github.com/ros2/rclcpp/pull/1416): "uses an events queue and a timers manager as opposed to waitsets". PR text says "greatly reduces CPU usage" with measurements "on a RaspberryPi 2 for a 20 nodes system", but the numbers are in an image I cannot read.
* Numbers I did read: ROS 2 discourse post https://discourse.openrobotics.org/t/ros2-middleware-change-proposal/15863 : proof of concept on a Raspberry Pi 1 (single core, about 700 MHz), Sierra-Nevada topology, intra-process off, baseline and event-queue both on the static single-threaded executor: event queue for subscriptions reduced load by 25%; latency from "just over 1ms" to 400 microseconds (2 to 4 times lower); direct FastDDS without ROS 2 showed ROS 2 overhead just under 300 microseconds per event. Node count not given. The saving there comes from not rebuilding a wait set per spin, not from fewer thread hand-offs, so it only loosely supports PX4.

### 6.2 Seastar (Apache-2.0)
* Pin 6a752c50208583cb122c1b7c0a405804a94805d2. `include/seastar/core/manual_clock.hh` 31-46: `manual_clock` with `static time_point now()` and `static void advance(duration)`; a virtual clock that fires timers on `advance`. Exists for tests; shows a run-to-completion reactor and virtual time coexisting.
* Tutorial (https://github.com/scylladb/seastar/blob/master/doc/tutorial.md, master, not pinned; summary by the fetch tool): one thread per core, cooperative tasks, 500 microsecond task quota, "reactor stall" over 20 ms, "the event-handling functions must never block, or the core will remain idle", shared-memory sharing between cores has "steep penalties", cross-core communication is explicit message passing. The summary did not show how blocking syscalls are offloaded. The page was truncated.
* Applied to PX4: the Seastar rule "never block in a task, send a message to another core" is the policy PX4 would need for any single-thread executor: MAVLink, logger, shell stay as real threads that talk to the executor through queues.

### 6.3 Drake (BSD-3-Clause)
* Pin d9c8a3cac5d3f2bcf8a478625b79e09faf44c484, `systems/analysis/simulator.h` 111-125 and 195-240: `AdvanceTo(t)` loops in one thread; at each event time it runs unrestricted updates, then discrete updates, then publishes, then the monitor, in a fixed order. Deterministic event order is part of the contract. No figures read.
* Applied to PX4: a model for "what runs at a given sim step, in which order": SIH step, then WQ items in a fixed order, then publishes.

### 6.4 Cyber RT, game job systems
Cyber RT: section 2.3. Game engine job systems (Naughty Dog fibers etc.): not reached, no numbers.

---------------------------------------------------------------------
## 7. Where each idea would NOT work for PX4, and why

1. Anything that serializes threads but keeps pthreads (Zephyr's nct, Parrot, Hermit, FreeRTOS POSIX, Shadow's channel): each hand-off remains a futex wake. They give a deterministic order, not fewer kernel wake-ups. Shadow's own authors found the only way to avoid the syscall is busy-waiting, which they dropped.
2. Process-per-thread determinism (DTHREADS, Grace, Determinator): breaks shared uORB/parameter memory and condition variables. No.
3. Kendo, CoreDet: patched kernel or recompiled-with-old-LLVM. No.
4. Threads that block in real system calls (MAVLink `poll`/`recvfrom` at mavlink_receiver.cpp 4016-4034; daemon `poll(-1)`/`accept` at server.cpp 160-175; logger `pthread_cond_wait` and file writes; `usleep` wall-clock waits in mavlink_main.cpp 1491-1517):
   * In a single-thread executor or a fiber scheduler, a real blocking call stops every other task. Options seen: run the call on a helper OS thread and wait on it (marl `blocking_call`, marl blockingcall.h 29-53); make the fd non-blocking and poll at sim-time intervals (Zephyr uart_native_pty.c 388-423); non-blocking fd plus yield with an event table (Pth in Shadow-Bitcoin section 3); keep that thread a normal pthread outside the scheduler and talk through a lock-free queue (Seastar rule). Hermit and Parrot both say the arrival time of a real external event cannot be made deterministic (Hermit ARCHITECTURE.md 763-771; Parrot section 4.4).
   * For replay: the bytes read from the MAVLink socket have to be recorded and injected at fixed sim times, or the flight is not identical. Nothing above does that for free except Hermit's "record/replay for its result" note.
5. Fibers share `thread_local`. `lockstep_scheduler.cpp` 134 uses `static thread_local TimedWait`; two fibers on one OS thread would share one object. Any fiber port must replace that, and any other `thread_local` in PX4 (not searched).
6. `ucontext` swaps cost 547 ns on the Boost reference machine, 60 times fcontext (9 ns): glibc `swapcontext` is not a cheap choice; use assembly fcontext (Boost.Context, marl osfiber_asm, Cyber RT swap_x86_64.S).
7. Busy loops: SIH itself polls `while(!_actuator_out_sub.updated()) usleep(100)` (sih.cpp 166-173). `usleep` is an interposable call, so a virtual-time scheduler must make `usleep(100)` yield, not spin. Any module that spins on an atomic with no call stalls the single-thread schedulers (Zephyr arch_soc.rst 96-125; Shadow limitations.md 76-83). I have not audited PX4 for this.
8. SCHED_FIFO priorities: WorkQueueManager.cpp 305-346 gives each work queue a real priority; a single-thread executor must reproduce "rate_ctrl runs before nav" ordering by an explicit rule (Hermit: lower priority number first, round robin within it).
9. Wall-clock vs sim-clock sleeps coexist (mavlink_main.cpp 1491-1495). A replacement must offer both.
10. Hermit/rr/Kendo style tools need performance counters; WSL2 availability not checked.

---------------------------------------------------------------------
## 8. Ranking for PX4 (three best-supported options)

### Option 1. Run every work queue on one run-to-completion executor in lockstep mode (Zephyr native_sim structure, Drake event order, Seastar rule, ROS 2 events queue)
* Sources and numbers:
  * Zephyr nct.c 12-59, nsi_hw_scheduler.c 75-134, cpuhalt.c 36-43: one thing runs, idle = time advance, deterministic by design; but still uses pthread semaphores per thread. The version PX4 would want drops those: run all `WorkItem::Run()` calls from one thread in a deterministic queue, advance time only when the queue is empty.
  * ROS 2 events queue: 25% less load, 1 ms to 400 microseconds latency, Raspberry Pi 1 at about 700 MHz (discourse post). Different mechanism; weak support.
  * Cost being removed: pthread switch 1.2-1.5 microseconds pinned, about 2.2 unpinned (Haswell i7-4771); my arithmetic for 600,000 switches: 0.72 to 1.32 s per 10 s run, plus the waker-side futex_wake (the 19.66% profile entry) which is not captured by that figure.
* What it breaks: work items that block (anything calling `px4_sem_wait`, `px4_usleep` or `px4_poll` inside `Run()`) would deadlock the executor (Zephyr arch_soc.rst 96-125); SCHED_FIFO preemption between queues turns into ordering only; the tty/SPI/I2C queue split is meaningless but harmless; MAVLink, logger, shell, SimulatorMavlink stay as threads and need a deterministic hand-off in (queue plus sim-time injection), otherwise replay is not identical. HRT callouts (HPWQ, drv_hrt.cpp 320-326) must become executor timers. Plain tasks (about 15) need either to be turned into work items or to stay threads.

### Option 2. User-level threads for work queues and plain tasks, with a lowest-priority "time driver" fiber (Shadow v1 + Pth)
* Sources and numbers:
  * Shadow-Bitcoin section 3: the lowest-priority simulator thread "only activated when every other thread is blocked" is the quiescence rule; it ran a real multi-threaded program (bitcoind) under virtual time.
  * Boost.Context: 9 ns per fcontext switch vs 547 ns ucontext, dual Xeon E5 2620v4 2.2 GHz; pthread 1.2-1.5 microseconds pinned (different machine). My arithmetic: 600,000 switches x 9 ns = 5.4 ms.
  * Boost.Fiber `suspend_until` / `notify` is the integration point for a virtual clock; marl `blocking_call` for real syscalls; NuttX sim `up_switch_context` and RIOT native `swapcontext` show tasks of an RTOS switching in one host thread.
  * Cost of ownership: Shadow dropped 47,959 LoC of custom Pth in the move to v2.
* What it breaks: every pthread/px4_sem/px4_usleep/poll call used by those threads must be intercepted or replaced (PX4 already wraps px4_sem and px4_usleep; direct pthread_mutex/cond uses exist, e.g. drv_hrt.cpp `_hrt_lock`, lockstep mutexes, log_writer_file.cpp); `thread_local` semantics; third-party code calling libc blocking functions; stack sizing; gdb/ASan/TSan lose per-thread views; real-blocking threads (MAVLink, daemon, logger) must stay OS threads or use blocking_call.
  Real I/O arrival times stay nondeterministic.

### Option 3. Keep pthreads, pin the whole SITL process to one CPU and make the existing barrier stricter (cheap experiment; supports determinism work, not elimination)
* Sources and numbers:
  * rr Table 1 "Single core" (all threads on one core, Skylake laptop): 0.98x, 3.36x, 1.36x, 1.07x, 0.95x for cp, make, octane, htmltest, sambatest.
  * Phantom paper Appendix C: same-core semaphore hand-off is the fastest; cross-core and cross-node are "significantly more expensive" (Xeon E5-2697 2.6 GHz; bars only, no text numbers).
  * Parrot: round-robin synchronization order costs 6.9% geometric mean (55 real programs), 12.7% (108 programs), when added to real threads on a 24-core server.
  * Hermit: "wait for quiescence before each turn" shows what to check beyond work queues (ARCHITECTURE.md 556-583).
* What it breaks / does not fix: futex_wake still happens, only it may stop sending cross-CPU wake-ups; real-time SCHED_FIFO threads spinning would starve others on one core; it gives no determinism by itself (the OS still picks the order); my claim that wake-ups to idle remote vCPUs are especially expensive under WSL2 is a hypothesis and not sourced.
  The brief's real determinism gap (plain tasks not tracked by the bitset) needs the barrier extended: register every task that can be woken by sim time, and have `px4_usleep` waiters count as in-progress until they block again.

Order of confidence: Option 3 is the easiest to test and has the thinnest claim; Option 1 has the most direct design precedent (Zephyr, Drake) for determinism and "idle means advance time"; Option 2 has the best numbers per switch and the only precedent that runs unmodified multi-threaded code under virtual time with a user-space scheduler, but is the most code.

---------------------------------------------------------------------
## 9. What I could not reach

* Shadow Phantom paper Figure 16 numeric values (bar chart only); the paper's Appendix B ptrace numbers.
* ROS 2 StaticSingleThreadedExecutor latency papers and the EventsExecutor benchmark image; Seastar cross-core cost numbers; Cyber RT switch/latency numbers; game engine fiber job systems (Naughty Dog); Drake performance numbers.
* Grace, RFDet, Consequence primary sources; Kendo and Parrot source (Parrot xtern submodule not fetched); CoreDet overhead figures (chart only); Hermit overhead figures.
* GNU Pth, State Threads, libco, libfiber, folly fibers: code and benchmarks not opened; marl benchmarks not opened.
* Whether WSL2 exposes hardware performance counters (matters for rr, Hermit RCB preemption, Kendo).
* A full-repo check of who calls `px4_lockstep_register_component` (code search API gave 401; no clones allowed).
* PX4-side measurement: no run, build or profile was done; all PX4 statements are from reading the pinned sources.
