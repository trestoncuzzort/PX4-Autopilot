/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <lockstep_scheduler/lockstep_scheduler.h>

#include <px4_platform_common/log.h>

LockstepScheduler::~LockstepScheduler()
{
	// cleanup the linked list
	std::unique_lock<std::mutex> lock_timed_waits(_timed_waits_mutex);

	while (_timed_waits) {
		TimedWait *tmp = _timed_waits;
		_timed_waits = _timed_waits->next;
		tmp->removed = true;
	}
}

void LockstepScheduler::set_absolute_time(uint64_t time_us)
{
	if (_time_us == 0 && time_us > 0) {
		PX4_INFO("setting initial absolute time to %" PRIu64 " us", time_us);
	}

	_time_us = time_us;

	std::lock_guard<std::mutex> lock_set_time(_set_time_mutex);

	// Phase 1: under _timed_waits_mutex, clean up done entries and thread the waiters that
	// have timed out onto a temporary list via signal_next. We do NOT take any passed_lock
	// here: that would create a lock-order cycle with cond_timedwait, which holds passed_lock
	// while acquiring _timed_waits_mutex.
	TimedWait *to_signal_head = nullptr;

	{
		std::lock_guard<std::mutex> lock_timed_waits(_timed_waits_mutex);

		TimedWait *timed_wait = _timed_waits;
		TimedWait *timed_wait_prev = nullptr;

		while (timed_wait) {
			// Clean up the ones that are already done from last iteration.
			if (timed_wait->done) {
				// Erase from the linked list
				if (timed_wait_prev) {
					timed_wait_prev->next = timed_wait->next;

				} else {
					_timed_waits = timed_wait->next;
				}

				TimedWait *tmp = timed_wait;
				timed_wait = timed_wait->next;
				tmp->removed = true;
				continue;
			}

			if (timed_wait->time_us <= time_us &&
			    !timed_wait->timeout) {
				// Pairs with the done store and signal_pending load in cond_timedwait(). Both are
				// sequentially consistent, so either this thread sees done and leaves the waiter
				// alone, or the waiter sees signal_pending and waits for phase 2 to finish with its
				// stack-local passed_lock and passed_cond. Possibly both, which is harmless.
				timed_wait->signal_pending = true;

				if (timed_wait->done) {
					timed_wait->signal_pending = false;

				} else {
					timed_wait->timeout = true;
					timed_wait->signal_next = to_signal_head;
					to_signal_head = timed_wait;
				}
			}

			timed_wait_prev = timed_wait;
			timed_wait = timed_wait->next;
		}
	}

	// Phase 2: signal each waiter outside _timed_waits_mutex. Holding passed_lock makes sure the
	// broadcast cannot be missed. A waiter woken by it can only see signal_pending once it has
	// re-acquired passed_lock, by which point it is already false, so the usual wake costs that
	// waiter nothing further. Only the unlock below still touches the waiter's lock, and POSIX
	// allows a mutex to be destroyed by whoever acquires it next.
	for (TimedWait *tw = to_signal_head; tw != nullptr;) {
		// Clearing signal_pending is the last access to the entry: the waiter can reuse it as soon
		// as it gets passed_lock back.
		TimedWait *next = tw->signal_next;
		pthread_mutex_t *passed_lock = tw->passed_lock;
		pthread_cond_t *passed_cond = tw->passed_cond;
		tw->signal_next = nullptr;
		pthread_mutex_lock(passed_lock);
		pthread_cond_broadcast(passed_cond);
		tw->signal_pending = false;
		pthread_mutex_unlock(passed_lock);
		tw = next;
	}
}

int LockstepScheduler::cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *lock, uint64_t time_us)
{
	// A TimedWait object might still be in timed_waits_ after we return, so its lifetime needs to be
	// longer. And using thread_local is more efficient than malloc.
	static thread_local TimedWait timed_wait;
	{
		std::lock_guard<std::mutex> lock_timed_waits(_timed_waits_mutex);

		// The time has already passed.
		if (time_us <= _time_us) {
			return ETIMEDOUT;
		}

		timed_wait.time_us = time_us;
		timed_wait.passed_cond = cond;
		timed_wait.passed_lock = lock;
		timed_wait.timeout = false;
		timed_wait.done = false;

		// Add to linked list if not removed yet (otherwise just re-use the object)
		if (timed_wait.removed) {
			timed_wait.removed = false;
			timed_wait.next = _timed_waits;
			_timed_waits = &timed_wait;
		}
	}

	// A single wait, returned on any wakeup. The caller holds 'lock' from
	// before registering until pthread_cond_wait releases it, and
	// set_absolute_time() takes 'lock' before broadcasting, so a timeout
	// broadcast cannot be missed. Re-waiting here would instead swallow a
	// signal from the caller's own signaler (e.g. px4_sem_post) that lands
	// while this thread is re-acquiring 'lock'.
	int result = pthread_cond_wait(cond, lock);

	const bool timeout = timed_wait.timeout;

	if (result == 0 && timeout) {
		result = ETIMEDOUT;
	}

	timed_wait.done = true;

	if (timed_wait.signal_pending) {
		// Rare: another signaler woke this thread after set_absolute_time() had selected it for
		// signaling. It still holds pointers to our possibly stack-local lock and cond, so we
		// must not return yet, and it needs 'lock' to make progress. Sleep rather than spin: the
		// time master can have a lower priority than this thread, and they can share a CPU.
		pthread_mutex_unlock(lock);

		while (timed_wait.signal_pending) {
			system_usleep(1);
		}

		pthread_mutex_lock(lock);
	}

	return result;
}

int LockstepScheduler::usleep_until(uint64_t time_us)
{
	pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

	pthread_mutex_lock(&lock);

	int result = cond_timedwait(&cond, &lock, time_us);

	if (result == ETIMEDOUT) {
		// This is expected because we never notified to the condition.
		result = 0;
	}

	pthread_mutex_unlock(&lock);

	return result;
}
