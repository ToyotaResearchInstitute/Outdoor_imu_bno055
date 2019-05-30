/* bno055_i2c_activity.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Defines a BNO055I2C Activity class, constructed with node handles
 * and which handles all ROS duties.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/logger.hpp"

#include "imu_bno055/bno055_i2c_activity.h"

namespace imu_bno055 {

// ******** constructors ******** //

BNO055I2CActivity::BNO055I2CActivity() : rclcpp::Node("bno055_i2c_node") {
    RCLCPP_INFO(get_logger(), "initializing");

#if defined(ROS_CRYSTAL)
    get_parameter_or_set("device", param_device, std::string("/dev/i2c-0"));
    get_parameter_or_set("address", param_address, (int)BNO055_ADDRESS_A);
    get_parameter_or_set("frame_id", param_frame_id, std::string("imu"));

    get_parameter_or_set("data_topic_name", data_topic_name, std::string("bno055_data"));
    get_parameter_or_set("raw_topic_name", raw_topic_name, std::string("bno055_raw"));
    get_parameter_or_set("mag_topic_name", mag_topic_name, std::string("bno055_mag"));
    get_parameter_or_set("temperature_topic_name", temperature_topic_name, std::string("bno055_temperature"));
    get_parameter_or_set("status_topic_name", status_topic_name, std::string("bno055_status"));
#elif defined(ROS_DASHING)
    declare_parameter("device", std::string("/dev/i2c-0"));
    declare_parameter("address", (int)BNO055_ADDRESS_A);
    declare_parameter("frame_id", std::string("imu"));

    declare_parameter("data_topic_name", std::string("bno055_data"));
    declare_parameter("raw_topic_name", std::string("bno055_raw"));
    declare_parameter("mag_topic_name", std::string("bno055_mag"));
    declare_parameter("temperature_topic_name", std::string("bno055_temperature"));
    declare_parameter("status_topic_name", std::string("bno055_status"));

    get_parameter("device", param_device);
    get_parameter("address", param_address);
    get_parameter("frame_id", param_frame_id);

    get_parameter("data_topic_name", data_topic_name);
    get_parameter("raw_topic_name", raw_topic_name);
    get_parameter("mag_topic_name", mag_topic_name);
    get_parameter("temperature_topic_name", temperature_topic_name);
    get_parameter("status_topic_name", status_topic_name);
#endif

    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::msg::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::msg::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::msg::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::msg::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::msg::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::msg::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);
}

// ******** private methods ******** //

bool BNO055I2CActivity::reset() {
    int i = 0;

    using std::chrono::nanoseconds;
    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    rclcpp::sleep_for(nanoseconds(25'000'000));

    // reset
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
    rclcpp::sleep_for(nanoseconds(25'000'000));

    // wait for chip to come back online
    while(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        rclcpp::sleep_for(nanoseconds(10'000'000));

        if(i++ > 500) {
            RCLCPP_ERROR(get_logger(), "chip did not come back online in 5 seconds after reset");
            return false;
        }
    }
    rclcpp::sleep_for(nanoseconds(100'000'000));


    // normal power mode
    _i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    rclcpp::sleep_for(nanoseconds(10'000'000));

    _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
    _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
    rclcpp::sleep_for(nanoseconds(25'000'000));

    _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
    rclcpp::sleep_for(nanoseconds(25'000'000));

    return true;
}

// ******** public methods ******** //

bool BNO055I2CActivity::start() {
    RCLCPP_INFO(get_logger(), "starting");

    if (!pub_data) {
        pub_data = this->create_publisher<sensor_msgs::msg::Imu>(data_topic_name, 10);
    }
    if (!pub_raw) {
        pub_raw = this->create_publisher<sensor_msgs::msg::Imu>(raw_topic_name, 10);
    }
    if (!pub_mag) {
        pub_mag = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_name, 10);
    }
    if (!pub_temp) {
        pub_temp = this->create_publisher<sensor_msgs::msg::Temperature>(temperature_topic_name, 10);
    }
    if (!pub_status) {
        pub_status = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(status_topic_name, 10);
    }

    if(!service_calibrate) {
        service_calibrate = this->create_service<std_srvs::srv::Trigger>("bno055_calibrate",
         [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void {
           this->onServiceCalibrate(request,response);
         });
    }

    if(!service_reset) {
        service_reset = this->create_service<std_srvs::srv::Trigger>("bno055_reset",
         [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void {
           this->onServiceReset(request,response);
         });
    }

    file = open(param_device.c_str(), O_RDWR|O_CLOEXEC);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        RCLCPP_ERROR(get_logger(), "i2c device open failed on: %s", param_device.c_str());
        return false;
    }

    if(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        RCLCPP_ERROR(get_logger(), "incorrect chip ID with s2c device: %s", param_device.c_str());
        return false;
    }
    RCLCPP_INFO(get_logger(), "Opened i2c device %s", param_device.c_str());

    std::stringstream ss;
    ss << "rev ids:"
      << " accel:" << _i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR)
      << " mag:" << _i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR)
      << " gyro:" << _i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR)
      << " sw:" << _i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR)
      << " bl:" << _i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR);
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

    if(!reset()) {
        RCLCPP_ERROR(get_logger(), "chip reset and setup failed");
        return false;
    }

    return true;
}

bool BNO055I2CActivity::spinOnce() {
    rclcpp::spin_some(shared_from_this());

    rclcpp::Time time = clock.now();

    IMURecord record{};

    unsigned char c = 0;

    seq++;

    // can only read a length of 0x20 at a time, so do it in 2 reads
    // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
        return false;
    }
    if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
        return false;
    }

    sensor_msgs::msg::Imu msg_raw;
    msg_raw.header.stamp = time;
    msg_raw.header.frame_id = param_frame_id;
    msg_raw.linear_acceleration.x = (double)record.raw_linear_acceleration_x / 100.0;
    msg_raw.linear_acceleration.y = (double)record.raw_linear_acceleration_y / 100.0;
    msg_raw.linear_acceleration.z = (double)record.raw_linear_acceleration_z / 100.0;
    msg_raw.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_raw.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_raw.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::msg::MagneticField msg_mag;
    msg_mag.header.stamp = time;
    msg_mag.header.frame_id = param_frame_id;
    msg_mag.magnetic_field.x = (double)record.raw_magnetic_field_x / 16.0;
    msg_mag.magnetic_field.y = (double)record.raw_magnetic_field_y / 16.0;
    msg_mag.magnetic_field.z = (double)record.raw_magnetic_field_z / 16.0;

    sensor_msgs::msg::Imu msg_data;
    msg_data.header.stamp = time;
    msg_data.header.frame_id = param_frame_id;

    double fused_orientation_norm = std::pow(
      std::pow(record.fused_orientation_w, 2) +
      std::pow(record.fused_orientation_x, 2) +
      std::pow(record.fused_orientation_y, 2) +
      std::pow(record.fused_orientation_z, 2), 0.5);

    msg_data.orientation.w = (double)record.fused_orientation_w / fused_orientation_norm;
    msg_data.orientation.x = (double)record.fused_orientation_x / fused_orientation_norm;
    msg_data.orientation.y = (double)record.fused_orientation_y / fused_orientation_norm;
    msg_data.orientation.z = (double)record.fused_orientation_z / fused_orientation_norm;
    msg_data.linear_acceleration.x = (double)record.fused_linear_acceleration_x / 100.0;
    msg_data.linear_acceleration.y = (double)record.fused_linear_acceleration_y / 100.0;
    msg_data.linear_acceleration.z = (double)record.fused_linear_acceleration_z / 100.0;
    msg_data.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_data.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_data.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::msg::Temperature msg_temp;
    msg_temp.header.stamp = time;
    msg_temp.header.frame_id = param_frame_id;
    msg_temp.temperature = (double)record.temperature;

    pub_data->publish(msg_data);
    pub_raw->publish(msg_raw);
    pub_mag->publish(msg_mag);
    pub_temp->publish(msg_temp);

    if(seq % 50 == 0) {
        current_status.values[DIAG_CALIB_STAT].value = std::to_string(record.calibration_status);
        current_status.values[DIAG_SELFTEST_RESULT].value = std::to_string(record.self_test_result);
        current_status.values[DIAG_INTR_STAT].value = std::to_string(record.interrupt_status);
        current_status.values[DIAG_SYS_CLK_STAT].value = std::to_string(record.system_clock_status);
        current_status.values[DIAG_SYS_STAT].value = std::to_string(record.system_status);
        current_status.values[DIAG_SYS_ERR].value = std::to_string(record.system_error_code);
        pub_status->publish(current_status);
    }

    return true;
}

bool BNO055I2CActivity::stop() {
    RCLCPP_INFO(get_logger(), "stopping");
    pub_data = nullptr;
    pub_raw = nullptr;
    pub_mag = nullptr;
    pub_temp = nullptr;
    pub_status = nullptr;
    service_calibrate = nullptr;
    service_reset = nullptr;

    return true;
}

bool BNO055I2CActivity::onServiceReset(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    if(!reset()) {
        RCLCPP_ERROR(get_logger(), "chip reset and setup failed");
        response->success = false;
        return false;
    }
    response->success = true;
    return true;
}

bool BNO055I2CActivity::onServiceCalibrate(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // TODO implement this
    (void)request;
    response->success = false;
    return true;
}

}  // namespace imu_bno055
