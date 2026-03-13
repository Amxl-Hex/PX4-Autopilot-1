#include "range_calculation.hpp"

using namespace time_literals;

namespace remaining_range
{
	ModuleBase::Descriptor RangeCalculation::desc{task_spawn, nullptr, print_usage};

	RangeCalculation::RangeCalculation() :
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{
		_range_calculation_pub.advertise();
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
			//PX4_INFO("Estimated range: %.2f m", remaining_range);


			range_calculation_s range_calc;
			range_calc.remaining_range_m = remaining_range;
			_range_calculation_pub.publish(range_calc);
		}
	}

	int RangeCalculation::print_usage(const char *reason)
	{
		if (reason) {
			PX4_ERR("%s\n", reason);
		}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
		### Description
		Module to calculate the remaining flight range based on current battery levels.
		)DESCR_STR"
		);
		PRINT_MODULE_USAGE_NAME("remaining_range", "system");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}


	extern "C" __EXPORT int remaining_range_main(int argc, char *argv[])
	{
		return ModuleBase::main(RangeCalculation::desc, argc, argv);
	}
}
