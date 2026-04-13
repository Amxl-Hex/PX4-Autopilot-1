/*




*/
#pragma once


#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/range_calculation.h>
//#include <uORB/topics/vehicle_global_position.h>
//#include <uORB/topics/rtl_status.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

namespace remaining_range
{
class RangeCalculation : public ModuleBase, public px4::ScheduledWorkItem
{
public:
    static Descriptor desc;

    RangeCalculation();
    ~RangeCalculation() override;

    static int task_spawn(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void start();

private:
    void Run() override;
    uORB::Subscription _bat_sub{ORB_ID(battery_status)};
    //uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
    //uORB::Subscription _rtl_status_sub{ORB_ID(rtl_status)};
    uORB::Publication<range_calculation_s> _range_calculation_pub{ORB_ID(range_calculation)};
    static constexpr float MAX_RANGE_M = 10000.0f; // Max range in meters for 100% battery
};

} // namespace remaining_range
