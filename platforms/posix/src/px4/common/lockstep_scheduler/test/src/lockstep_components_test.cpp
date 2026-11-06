#include <lockstep_scheduler/lockstep_components.h>
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace
{

class Waiter
{
public:
	explicit Waiter(LockstepComponents &components)
		: _thread([this, &components]() { components.wait_for_components(); _returned = true; })
	{}

	~Waiter() { _thread.join(); }

	bool returned_within(std::chrono::milliseconds timeout)
	{
		const auto deadline = std::chrono::steady_clock::now() + timeout;

		while (!_returned && std::chrono::steady_clock::now() < deadline) {
			std::this_thread::sleep_for(1ms);
		}

		return _returned;
	}

private:
	std::atomic<bool> _returned{false};
	std::thread _thread;
};

} // namespace

TEST(LockstepComponents, IdleComponentsDoNotReleaseNewWork)
{
	LockstepComponents components;

	// A work queue picking up and finishing work while nothing waits yet, as before the simulator
	// engages lockstep.
	for (int i = 0; i < 10; ++i) {
		components.unregister_component(components.register_component());
	}

	components.wait_for_components();
	const int busy = components.register_component();
	Waiter waiter(components);
	EXPECT_FALSE(waiter.returned_within(100ms));
	components.unregister_component(busy);
	EXPECT_TRUE(waiter.returned_within(5s));
}

TEST(LockstepComponents, ProgressIsConsumedOnce)
{
	LockstepComponents components;
	const int persistent = components.register_component();
	components.lockstep_progress(persistent);
	components.lockstep_progress(persistent);
	components.wait_for_components();

	Waiter waiter(components);
	EXPECT_FALSE(waiter.returned_within(100ms));
	components.lockstep_progress(persistent);
	EXPECT_TRUE(waiter.returned_within(5s));
}

TEST(LockstepComponents, UnregisterPreservesOtherComponentsProgress)
{
	LockstepComponents components;
	const int persistent = components.register_component();
	const int transient = components.register_component();
	components.lockstep_progress(persistent);
	Waiter waiter(components);
	EXPECT_FALSE(waiter.returned_within(100ms));
	components.unregister_component(transient);
	EXPECT_TRUE(waiter.returned_within(5s));
}

TEST(LockstepComponents, NewComponentJoinsActiveCycle)
{
	LockstepComponents components;
	const int first = components.register_component();
	Waiter waiter(components);
	EXPECT_FALSE(waiter.returned_within(100ms));
	const int second = components.register_component();
	components.unregister_component(first);
	EXPECT_FALSE(waiter.returned_within(100ms));
	components.unregister_component(second);
	EXPECT_TRUE(waiter.returned_within(5s));
}
