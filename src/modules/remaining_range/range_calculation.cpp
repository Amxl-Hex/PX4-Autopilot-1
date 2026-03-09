#include "range_calculation.hpp"

using namespace time_literals;

namespace remaining_range
{
	ModuleBase::Descriptor RangeCalculation::desc{task_spawn, print_usage};

	RangeCalculation::RangeCalculation() :
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{

	}

	RangeCalculation::~RangeCalculation()
	{

	}

	int RangeCalculation::task_spawn(int argc, char *argv[])
	{
		RangeCalculation *obj = new RangeCalculation();

		if (!obj) {
			PX4_ERR("alloc failed");
			return -1;
		}

		desc.object.store(obj);
		desc.task_id = task_id_is_work_queue;

		/* Schedule a cycle to start things. */
		obj->start();

		return 0;
	}

	void RangeCalculation::start()
	{
		ScheduleOnInterval(1000_ms); // 1 Hz
	}

	void RangeCalculation::Run()
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup(desc);
		}

		if (_bat_sub.updated()) {
			battery_status_s battery_status;
			_bat_sub.copy(&battery_status);

			float remaining_range = battery_status.remaining * MAX_RANGE_M;
			PX4_INFO("Remaining range: %.2f meters", (double)remaining_range);
		}
	}
}
