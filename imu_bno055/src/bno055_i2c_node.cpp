/* bno055_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a BNO055I2C Activity class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

#include <memory>

#include "watchdog/watchdog.h"

#include <imu_bno055/bno055_i2c_activity.h>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto activity = std::make_shared<imu_bno055::BNO055I2CActivity>();

    auto watchdog = std::make_unique<watchdog::Watchdog>();

    if(!activity->start()) {
        RCLCPP_ERROR(activity->get_logger(), "Failed to start activity");
        return -4;
    }

    watchdog->start(5000);

    int param_rate;
    activity->get_parameter_or_set("rate", param_rate, 100);

    rclcpp::Rate rate(param_rate);
    while(rclcpp::ok()) {
        rate.sleep();
        if(activity->spinOnce()) {
            watchdog->refresh();
        }
    }

    activity->stop();
    watchdog->stop();

    rclcpp::shutdown();

    return 0;
}
