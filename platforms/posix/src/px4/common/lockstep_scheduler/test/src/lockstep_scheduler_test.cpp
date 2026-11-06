#include <lockstep_scheduler/lockstep_scheduler.h>
#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <vector>
#include <atomic>
#include <random>
#include <iostream>
#include <functional>
#include <chrono>

class TestThread
{
public:
	TestThread(const std::function<void()> &f)
		: _f(f)
	{
		_thread = std::thread(std::bind(&TestThread::execute, this));
	}

	void join(LockstepScheduler &ls)
	{
		// The unit-tests do not reflect the real usage, where
		// set_absolute_time() is called regularly and can do some
		// cleanup tasks. We simulate that here by waiting until
		// the given task returns (which is expected to happen quickly)
		// and then call set_absolute_time(), which can do the cleanup,
		// and _thread can then exit as well.
		while (!_done) {
			std::this_thread::yield(); // usleep is too slow here
		}

		ls.set_absolute_time(ls.get_absolute_time());
		_thread.join();
	}
private:
	void execute()
	{
		_f();
		_done = true;
	}
	std::function<void()> _f;
	std::atomic<bool> _done{false};
	std::thread _thread;
};

constexpr uint64_t some_time_us = 12345678;

void test_absolute_time()
{
	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);
	EXPECT_EQ(ls.get_absolute_time(),  some_time_us);
}

void test_condition_timing_out()
{
	// Create locked condition.
	pthread_cond_t cond;
	pthread_cond_init(&cond, NULL);

	// And a lock which needs to be locked
	pthread_mutex_t lock;
	pthread_mutex_init(&lock, NULL);

	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);

	std::atomic<bool> should_have_timed_out{false};

	// The thread that calls cond_timedwait must also lock the mutex,
	// otherwise set_absolute_time() would re-lock the main thread's
	// own mutex in its broadcast phase, causing a deadlock.
	TestThread thread([&ls, &cond, &lock, &should_have_timed_out]() {
		pthread_mutex_lock(&lock);
		EXPECT_EQ(ls.cond_timedwait(&cond, &lock, some_time_us + 1000), ETIMEDOUT);
		EXPECT_TRUE(should_have_timed_out);
		// It should be re-locked afterwards, so we should be able to unlock it.
		EXPECT_EQ(pthread_mutex_unlock(&lock), 0);
	});

	ls.set_absolute_time(some_time_us + 500);
	should_have_timed_out = true;
	ls.set_absolute_time(some_time_us + 1500);

	thread.join(ls);

	pthread_mutex_destroy(&lock);
	pthread_cond_destroy(&cond);

}

void test_locked_semaphore_getting_unlocked()
{
	// Create locked condition.
	pthread_cond_t cond;
	pthread_cond_init(&cond, NULL);

	// And a lock which needs to be locked
	pthread_mutex_t lock;
	pthread_mutex_init(&lock, NULL);

	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);

	std::atomic<bool> thread_locked{false};

	// The thread locks the mutex itself (POSIX requires the calling thread
	// to own the mutex passed to pthread_cond_wait).
	TestThread thread([&ls, &cond, &lock, &thread_locked]() {
		pthread_mutex_lock(&lock);
		thread_locked = true;
		ls.set_absolute_time(some_time_us + 500);
		EXPECT_EQ(ls.cond_timedwait(&cond, &lock, some_time_us + 1000), 0);
		// It should be re-locked afterwards, so we should be able to unlock it.
		EXPECT_EQ(pthread_mutex_unlock(&lock), 0);
	});

	// Wait until thread has locked the mutex. Our pthread_mutex_lock below
	// will then block until cond_timedwait releases it via pthread_cond_wait.
	while (!thread_locked) {
		std::this_thread::yield();
	}

	pthread_mutex_lock(&lock);
	pthread_cond_broadcast(&cond);
	pthread_mutex_unlock(&lock);

	thread.join(ls);

	pthread_mutex_destroy(&lock);
	pthread_cond_destroy(&cond);
}

class TestCase
{
public:
	TestCase(unsigned timeout, unsigned unlocked_after, LockstepScheduler &ls) :
		_timeout(timeout + some_time_us),
		_unlocked_after(unlocked_after + some_time_us),
		_ls(ls)
	{
		pthread_mutex_init(&_lock, NULL);
		pthread_cond_init(&_cond, NULL);
	}

	~TestCase()
	{
		EXPECT_TRUE(_is_done);
		pthread_mutex_destroy(&_lock);
		pthread_cond_destroy(&_cond);
	}

	void run()
	{
		_thread = std::make_shared<TestThread>([this]() {
			pthread_mutex_lock(&_lock);
			_thread_ready = true;
			_result = _ls.cond_timedwait(&_cond, &_lock, _timeout);
			pthread_mutex_unlock(&_lock);
		});
	}

	void wait_until_ready()
	{
		// Block until the worker thread has locked its mutex and is about to wait
		// on the condition. Otherwise advancing virtual time could pass this case's
		// timeout before the thread even starts, so check() (which early-returns on
		// !_thread_ready) would never complete it and its thread would be left
		// joinable at destruction.
		while (!_thread_ready) {
			std::this_thread::yield();
		}
	}

	void check()
	{
		if (_is_done || !_thread_ready) {
			return;
		}

		uint64_t time_us = _ls.get_absolute_time();

		const bool unlock_reached = (time_us >= _unlocked_after);
		const bool unlock_is_before_timeout = (_unlocked_after <= _timeout);
		const bool timeout_reached = (time_us >= _timeout);

		if (unlock_reached && unlock_is_before_timeout && !(timeout_reached)) {
			pthread_mutex_lock(&_lock);
			pthread_cond_broadcast(&_cond);
			pthread_mutex_unlock(&_lock);
			_is_done = true;
			// We can be sure that this triggers.
			_thread->join(_ls);
			EXPECT_EQ(_result, 0);
		}

		else if (timeout_reached) {
			_is_done = true;
			_thread->join(_ls);
			EXPECT_EQ(_result, ETIMEDOUT);
		}
	}
private:
	static constexpr int INITIAL_RESULT = 42;

	unsigned _timeout;
	unsigned _unlocked_after;
	pthread_cond_t _cond;
	pthread_mutex_t _lock;
	LockstepScheduler &_ls;
	std::atomic<bool> _is_done{false};
	std::atomic<bool> _thread_ready{false};
	std::atomic<int> _result {INITIAL_RESULT};
	std::shared_ptr<TestThread> _thread{};
};

int random_number(int min, int max)
{
	// We want predictable test results, so we always
	// start with the seed 0.
	static int iteration = 0;

	std::seed_seq seed{iteration++};
	std::default_random_engine engine{seed};
	std::uniform_int_distribution<> distribution(min, max);

	const int random_number = distribution(engine);
	return random_number;
}

void test_multiple_semaphores_waiting()
{

	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);

	// Use different timeouts in random order.
	std::vector<std::shared_ptr<TestCase>> test_cases{};

	test_cases.push_back(
		std::make_shared<TestCase>(
			11111, 11111, ls));

	test_cases.push_back(
		std::make_shared<TestCase>(
			20000, 20000, ls));

	test_cases.push_back(
		std::make_shared<TestCase>(
			0, 20000, ls));

	test_cases.push_back(
		std::make_shared<TestCase>(
			20000, 10000, ls));

	test_cases.push_back(
		std::make_shared<TestCase>(
			0, 0, ls));

	const int num_additional_threads = random_number(1, 20);

	for (int i = 0; i < num_additional_threads; ++i) {
		const unsigned timeout = random_number(1, 20000);
		const unsigned unlocked_after = random_number(1, 20000);
		test_cases.push_back(
			std::make_shared<TestCase>(
				timeout, unlocked_after, ls));
	}

	for (auto &test_case : test_cases) {
		test_case->run();
	}

	// Make sure every thread has started and is waiting before we advance virtual
	// time, so the loop below can't skip past a case's timeout before it registers.
	for (auto &test_case : test_cases) {
		test_case->wait_until_ready();
	}

	const int min_step_size = 1;
	const int max_step_size = 100;

	// We need to go until the max plus max step size to make sure we trigger
	// all timeouts or semaphores.
	for (unsigned time_us = 1;
	     time_us <= (20000 + max_step_size);
	     time_us += random_number(min_step_size, max_step_size)) {

		ls.set_absolute_time(some_time_us + time_us);

		for (auto &test_case : test_cases) {
			test_case->check();
		}
	}

	test_cases.clear();
}

#define WAIT_FOR(condition_) \
	while (!(condition_)) { \
		std::this_thread::yield(); \
	}

void test_usleep()
{
	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);

	enum class Step {
		Init,
		ThreadStarted,
		BeforeUsleep,
		UsleepNotTriggeredYet,
		UsleepTriggered
	};

	std::atomic<Step> step{Step::Init};

	TestThread thread([&step, &ls]() {
		step = Step::ThreadStarted;

		WAIT_FOR(step == Step::BeforeUsleep);

		step = Step::UsleepNotTriggeredYet;
		ls.set_absolute_time(some_time_us + 500);

		step = Step::UsleepTriggered;
		ls.set_absolute_time(some_time_us + 1500);
	});

	WAIT_FOR(step == Step::ThreadStarted);

	step = Step::BeforeUsleep;

	EXPECT_EQ(ls.usleep_until(some_time_us + 1000), 0);
	EXPECT_EQ(step, Step::UsleepTriggered);
	thread.join(ls);
}

// Waiters keep their lock and cond on the stack and are woken by time and by another signaler at the same moment.
// This is the case where the time master has selected a waiter while something else wakes it. Run under ASan or
// TSan, it catches the time master touching a lock or cond after the waiter has returned and destroyed them.
TEST(LockstepScheduler, StackWaitsWithCompetingSignaler)
{
	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);

	struct Slot {
		std::mutex mutex;
		pthread_mutex_t *lock{nullptr};
		pthread_cond_t *cond{nullptr};
	};

	static constexpr int num_waiters = 8;
	static constexpr unsigned min_ticks = 20000;
	Slot slots[num_waiters];
	std::atomic<bool> stop{false};
	std::atomic<bool> joined{false};
	std::atomic<uint64_t> completed{0};

	std::thread ticker([&ls, &joined]() {
		uint64_t time_us = some_time_us;

		while (!joined) {
			time_us += 2;
			ls.set_absolute_time(time_us);
		}
	});

	std::thread signaler([&slots, &stop]() {
		while (!stop) {
			for (Slot &slot : slots) {
				std::lock_guard<std::mutex> guard(slot.mutex);

				if (slot.lock) {
					pthread_mutex_lock(slot.lock);
					pthread_cond_broadcast(slot.cond);
					pthread_mutex_unlock(slot.lock);
				}
			}
		}
	});

	std::vector<std::thread> waiters;

	for (int i = 0; i < num_waiters; ++i) {
		waiters.emplace_back([&ls, &slots, &stop, &completed, i]() {
			while (!stop) {
				pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
				pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

				{
					std::lock_guard<std::mutex> guard(slots[i].mutex);
					slots[i].lock = &lock;
					slots[i].cond = &cond;
				}

				pthread_mutex_lock(&lock);
				const int result = ls.cond_timedwait(&cond, &lock, ls.get_absolute_time() + 1 + (i % 3));
				EXPECT_TRUE(result == 0 || result == ETIMEDOUT);
				pthread_mutex_unlock(&lock);

				{
					std::lock_guard<std::mutex> guard(slots[i].mutex);
					slots[i].lock = nullptr;
					slots[i].cond = nullptr;
				}

				pthread_mutex_destroy(&lock);
				pthread_cond_destroy(&cond);
				++completed;
			}
		});
	}

	while (completed < min_ticks) {
		std::this_thread::yield();
	}

	stop = true;
	signaler.join();

	// Each waiter's exit waits for the ticker to clear its entry from the scheduler, so time has to keep moving here.
	for (std::thread &waiter : waiters) {
		waiter.join();
	}

	joined = true;
	ticker.join();

	EXPECT_GE(completed, min_ticks);
}

// Lets a test see how far set_absolute_time() has got, instead of guessing from a sleep.
class LockstepSchedulerTestPeer
{
public:
	static int selected_waiters(LockstepScheduler &ls)
	{
		std::lock_guard<std::mutex> guard(ls._timed_waits_mutex);
		int count = 0;

		for (LockstepScheduler::TimedWait *entry = ls._timed_waits; entry != nullptr; entry = entry->next) {
			if (entry->signal_pending) {
				++count;
			}
		}

		return count;
	}
};

// One waiter's lock is held so that the time master stops partway through signaling. A second waiter, selected but not
// signaled yet, is woken by someone else. It must not return, because the time master still holds pointers to its lock
// and cond. Entries are signaled in the order they registered, so the first waiter here blocks the second.
TEST(LockstepScheduler, SelectedWaiterWokenByOtherSignalerWaitsForSignalPhase)
{
	LockstepScheduler ls;
	ls.set_absolute_time(some_time_us);
	const uint64_t deadline = some_time_us + 1000;

	pthread_mutex_t first_lock = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t first_cond = PTHREAD_COND_INITIALIZER;
	pthread_mutex_t second_lock = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t second_cond = PTHREAD_COND_INITIALIZER;
	std::atomic<bool> first_ready{false};
	std::atomic<bool> second_ready{false};
	std::atomic<bool> second_returned{false};

	std::thread first([&]() {
		pthread_mutex_lock(&first_lock);
		first_ready = true;
		EXPECT_EQ(ls.cond_timedwait(&first_cond, &first_lock, deadline), ETIMEDOUT);
		pthread_mutex_unlock(&first_lock);
	});

	while (!first_ready) { std::this_thread::yield(); }

	// Taking the lock proves the first waiter is inside its wait. Keep holding it to stall the time master there.
	pthread_mutex_lock(&first_lock);

	std::thread second([&]() {
		pthread_mutex_lock(&second_lock);
		second_ready = true;
		ls.cond_timedwait(&second_cond, &second_lock, deadline);
		second_returned = true;
		pthread_mutex_unlock(&second_lock);
	});

	while (!second_ready) { std::this_thread::yield(); }

	pthread_mutex_lock(&second_lock);
	pthread_mutex_unlock(&second_lock);

	std::thread time_master([&]() { ls.set_absolute_time(deadline); });

	// Wait until both are selected and the time master is stalled on the first waiter's lock.
	const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(30);

	while (LockstepSchedulerTestPeer::selected_waiters(ls) < 2 && std::chrono::steady_clock::now() < give_up) {
		std::this_thread::yield();
	}

	EXPECT_EQ(LockstepSchedulerTestPeer::selected_waiters(ls), 2);

	pthread_mutex_lock(&second_lock);
	pthread_cond_broadcast(&second_cond);
	pthread_mutex_unlock(&second_lock);

	// Long enough for a wrongly released waiter to get through. A correct one waits regardless of this time.
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	EXPECT_FALSE(second_returned);

	pthread_mutex_unlock(&first_lock);
	time_master.join();

	std::atomic<bool> joined{false};
	std::thread cleaner([&]() {
		uint64_t time_us = deadline;

		while (!joined) {
			ls.set_absolute_time(++time_us);
			std::this_thread::yield();
		}
	});

	first.join();
	second.join();
	joined = true;
	cleaner.join();

	EXPECT_TRUE(second_returned);
	pthread_mutex_destroy(&first_lock);
	pthread_cond_destroy(&first_cond);
	pthread_mutex_destroy(&second_lock);
	pthread_cond_destroy(&second_cond);
}

TEST(LockstepScheduler, All)
{
	for (unsigned iteration = 1; iteration <= 100; ++iteration) {
		//std::cout << "Test iteration: " << iteration << "\n";
		test_absolute_time();
		test_condition_timing_out();
		test_locked_semaphore_getting_unlocked();
		test_usleep();
		test_multiple_semaphores_waiting();
	}
}
