#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_land_detected.h>

using namespace time_literals;

class RemainingRange : public px4::ModuleBase<RemainingRange>, public px4::ScheduledWorkItem
{
public:
    RemainingRange() : ModuleBase<RemainingRange>(), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
    ~RemainingRange() override { ScheduleClear(); }

    static int task_spawn(int argc, char *argv[]) {
        RemainingRange *instance = new RemainingRange();
        if (instance) {
            _object.store(instance);
            instance->ScheduleOnInterval(1000000_us); // 1 Second interval
            return 0;
        }
        return -1;
    }

    static int print_usage(const char *reason = nullptr) {
        PRINT_MODULE_DESCRIPTION("Estimates remaining flight range");
        return 0;
    }

private:
    void Run() override {
        battery_status_s bat;
        vehicle_land_detected_s land;

        if (_bat_sub.copy(&bat) && _land_sub.copy(&land)) {
            // Only print if the drone is NOT on the ground
            if (!land.landed) {
                // bat.remaining is 0.0 to 1.0. Multiply by 100 for percentage.
                float range_m = (bat.remaining * 100.0f) * 100.0f;
                PX4_INFO("Remaining range = %.1f m", (double)range_m);
            }
        }
    }

    uORB::Subscription _bat_sub{ORB_ID(battery_status)};
    uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
};

extern "C" __EXPORT int remaining_range_main(int argc, char *argv[]) {
    return RemainingRange::main(argc, argv);
}
