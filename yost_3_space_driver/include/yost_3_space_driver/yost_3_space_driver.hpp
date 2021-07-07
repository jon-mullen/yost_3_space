#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "yost_3_space_api/threespace_api_export.h"

class Yost3SpaceDriver : public rclcpp::Node
{
public:
    Yost3SpaceDriver();
    ~Yost3SpaceDriver();

    // initialize sensor
    bool initializeSensor();

    // do the hardware settings that make the sensor's axes etc. more compatible with ROS
    bool setCompatibilitySettings();

    // stop streaming and nicely close API
    void shutdownDriver();
    void shutdownAPI();

    bool startStream();

    // receive streaming packet and publish in ROS
    void getAndPublish();

private:

    static void packVector3(const float data[], geometry_msgs::msg::Vector3 &vec3, float multiplier = 1.0f);
    static void packQuat4(const float data[], geometry_msgs::msg::Quaternion &quat);

    // timer for periodically receiving streaming packet and publishing
    rclcpp::TimerBase::SharedPtr output_timer_;

    // publishers
    // see REP-145 for topics published by driver (https://www.ros.org/reps/rep-0145.html#topics)
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr data_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr data_raw_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_raw_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_ = nullptr;

    // TODO: services

    // parameters
    // for serial communication
    std::string device_address_ = "/dev/ttyACM0";
    // to comply with REP-145 (https://www.ros.org/reps/rep-0145.html#common-parameters)
    std::string frame_id_ = "imu_link";
    // TODO: need defaults for these (from Yost spec sheet?)
    double linear_acceleration_stddev_ = 0.0;
    double angular_velocity_stddev_ = 0.0;
    double magnetic_field_stddev_ = 0.0;
    double orientation_stddev_ = 0.0;

    // Yost API related
    tss_device_id device_id_ = 0;

};