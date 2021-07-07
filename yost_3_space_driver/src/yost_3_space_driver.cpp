//
// Created by jmull on 7/7/21.
//

#include "yost_3_space_driver/yost_3_space_driver.hpp"

Yost3SpaceDriver::Yost3SpaceDriver() : Node("yost_3_space_driver_node")
{
    // declare params
    this->declare_parameter<std::string>("device_address", "/dev/ttyACM0");

    this->declare_parameter<std::string>("frame_id", "imu_link");
    // TODO: need defaults for these (from Yost spec sheet?)
    this->declare_parameter<double>("linear_acceleration_stddev", 0.0);
    this->declare_parameter<double>("angular_velocity_stddev", 0.0);
    this->declare_parameter<double>("magnetic_field_stddev", 0.0);
    this->declare_parameter<double>("orientation_stddev", 0.0);

    this->get_parameter("device_address", this->device_address_);

    this->get_parameter("frame_id", this->frame_id_);

    this->get_parameter("linear_acceleration_stddev", this->linear_acceleration_stddev_);
    this->get_parameter("angular_velocity_stddev", this->angular_velocity_stddev_);
    this->get_parameter("magnetic_field_stddev", this->magnetic_field_stddev_);
    this->get_parameter("orientation_stddev", this->orientation_stddev_);

    // initialize the sensor
    bool initialized = initializeSensor();

    if (!initialized)
    {
        shutdownAPI();
        exit(EXIT_FAILURE);
    }

    // do the hardware settings that make the sensor's axes etc. more compatible with ROS
    bool compatibility_set = setCompatibilitySettings();

    if (!compatibility_set)
    {
        shutdownDriver();
        shutdownAPI();
        exit(EXIT_FAILURE);
    }

    // start stream
    bool stream_started = startStream();

    if (!stream_started)
    {
        shutdownDriver();
        shutdownAPI();
        exit(EXIT_FAILURE);
    }

    // create publishers
    data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    data_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
    mag_raw_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag_raw", 10);
    temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);

    // TODO: create services

    // create timer
    output_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Yost3SpaceDriver::getAndPublish, this));
}

Yost3SpaceDriver::~Yost3SpaceDriver()
{
    this->shutdownDriver();
    this->shutdownAPI();
}

bool Yost3SpaceDriver::initializeSensor()
{
    RCLCPP_INFO(this->get_logger(), "Initiailizing sensor...");

    TSS_ERROR error = tss_createSensor(this->device_address_.c_str(), &this->device_id_);

    if (error)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open sensor at %s: %s",
                     this->device_address_.c_str(), tss_error_string[error]);
        return false;
    }

    return true;
}

// do the hardware settings that make the sensor's axes etc. more compatible with ROS
bool Yost3SpaceDriver::setCompatibilitySettings()
{
    RCLCPP_INFO(this->get_logger(), "Setting compatibility settings...");

    bool all_set_correctly = true;

    // set axes so they are right-handed
    U32 set_axes_timestamp = 0;
    TSS_ERROR set_axes_error = tss_sensor_setAxisDirections(this->device_id_, 0x01, &set_axes_timestamp);

    if (set_axes_error)
    {
        RCLCPP_ERROR(this->get_logger(), "Error setting the axis directions: %s",
                     tss_error_string[set_axes_error]);
        all_set_correctly = false;
    }

    return all_set_correctly;

}

// stop streaming and remove sensor
void Yost3SpaceDriver::shutdownDriver()
{
    RCLCPP_INFO(this->get_logger(), "Stopping stream on IMU...");
    tss_sensor_stopStreamingWired(this->device_id_);
    tss_removeSensor(this->device_id_);
}

// nicely shutdown API
void Yost3SpaceDriver::shutdownAPI()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Yost 3-Space API...");
    tss_deinitAPI();
}

bool Yost3SpaceDriver::startStream()
{
    RCLCPP_INFO(this->get_logger(), "Starting stream on IMU...");

    // choose which signals to stream
    U32 data_flags = TSS_STREAM_RAW_ACCELEROMETER_DATA |
                    TSS_STREAM_RAW_GYROSCOPE_DATA |
                    TSS_STREAM_RAW_MAGNETOMETER_DATA |
                    TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION |
                    TSS_STREAM_CORRECTED_ACCELEROMETER_DATA |
                    TSS_STREAM_CORRECTED_GYROSCOPE_DATA |
                    TSS_STREAM_CORRECTED_MAGNETOMETER_DATA |
                    TSS_STREAM_CELSIUS_TEMPERATURE;

    // start the stream
    TSS_ERROR error = tss_sensor_startStreamingWired(this->device_id_, data_flags,
                                           1000 /*microseconds*/, TSS_STREAM_DURATION_INFINITE, 0);

    if (error)
    {
        RCLCPP_ERROR(this->get_logger(), "Error starting stream: %s",
                     tss_error_string[error]);
        return false;
    }

    return true;
}

// receive streaming packet and publish in ROS
void Yost3SpaceDriver::getAndPublish()
{
    TSS_Stream_Packet packet;
    TSS_ERROR error = tss_sensor_getLastStreamingPacket(this->device_id_, &packet);

    if (error)
    {
        RCLCPP_ERROR(this->get_logger(), "Error getting last stream packet: %s",
                     tss_error_string[error]);
    }
    else
    {
        // corrected imu data
        {
            sensor_msgs::msg::Imu data_msg = sensor_msgs::msg::Imu();

            // header
            data_msg.header = std_msgs::msg::Header();
            data_msg.header.frame_id = this->frame_id_;
            data_msg.header.stamp = this->now();

            // linear acceleration (with multiplication by g)
            data_msg.linear_acceleration = geometry_msgs::msg::Vector3();
            packVector3(packet.correctedAccelerometerData, data_msg.linear_acceleration, 9.81f);
            double linear_accel_cov = this->linear_acceleration_stddev_ * this->linear_acceleration_stddev_;
            data_msg.linear_acceleration_covariance =
                    { linear_accel_cov, 0.0f, 0.0f,
                     0.0f, linear_accel_cov, 0.0f,
                     0.0f, 0.0f, linear_accel_cov };

            // angular velocity
            data_msg.angular_velocity = geometry_msgs::msg::Vector3();
            packVector3(packet.correctedGyroscopeData, data_msg.angular_velocity);
            double angular_velocity_cov = this->angular_velocity_stddev_ * this->angular_velocity_stddev_;
            data_msg.angular_velocity_covariance =
                    { angular_velocity_cov, 0.0f, 0.0f,
                     0.0f, angular_velocity_cov, 0.0f,
                     0.0f, 0.0f, angular_velocity_cov };

            // orientation
            data_msg.orientation = geometry_msgs::msg::Quaternion();
            packQuat4(packet.untaredOrientQuat, data_msg.orientation);
            double orientation_cov = this->orientation_stddev_ * this->orientation_stddev_;
            data_msg.orientation_covariance =
                    { orientation_cov, 0.0f, 0.0f,
                     0.0f, orientation_cov, 0.0f,
                     0.0f, 0.0f, orientation_cov };

            // publish
            data_pub_->publish(data_msg);
        }


        // raw imu data
        {
            sensor_msgs::msg::Imu data_raw_msg = sensor_msgs::msg::Imu();

            // header
            data_raw_msg.header = std_msgs::msg::Header();
            data_raw_msg.header.frame_id = this->frame_id_;
            data_raw_msg.header.stamp = this->now();

            // linear acceleration (with multiplication by g)
            data_raw_msg.linear_acceleration = geometry_msgs::msg::Vector3();
            packVector3(packet.rawAccelerometerData, data_raw_msg.linear_acceleration, 9.81f);
            double linear_accel_cov = this->linear_acceleration_stddev_ * this->linear_acceleration_stddev_;
            data_raw_msg.linear_acceleration_covariance =
                    { linear_accel_cov, 0.0f, 0.0f,
                      0.0f, linear_accel_cov, 0.0f,
                      0.0f, 0.0f, linear_accel_cov };

            // angular velocity
            data_raw_msg.angular_velocity = geometry_msgs::msg::Vector3();
            packVector3(packet.rawGyroscopeData, data_raw_msg.angular_velocity);
            double angular_velocity_cov = this->angular_velocity_stddev_ * this->angular_velocity_stddev_;
            data_raw_msg.angular_velocity_covariance =
                    { angular_velocity_cov, 0.0f, 0.0f,
                      0.0f, angular_velocity_cov, 0.0f,
                      0.0f, 0.0f, angular_velocity_cov };

            // publish
            data_raw_pub_->publish(data_raw_msg);
        }

        // magnetometer
        {
            sensor_msgs::msg::MagneticField mag_msg = sensor_msgs::msg::MagneticField();
            mag_msg.header = std_msgs::msg::Header();
            mag_msg.header.frame_id = this->frame_id_;
            mag_msg.header.stamp = this->now();

            mag_msg.magnetic_field = geometry_msgs::msg::Vector3();
            packVector3(packet.correctedMagnetometerData, mag_msg.magnetic_field);
            double mag_cov = this->magnetic_field_stddev_ * this->magnetic_field_stddev_;
            mag_msg.magnetic_field_covariance =
                    { mag_cov, 0.0f, 0.0f,
                     0.0f, mag_cov, 0.0f,
                     0.0f, 0.0f, mag_cov};

            mag_pub_->publish(mag_msg);
        }

        // raw magnetometer
        {
            sensor_msgs::msg::MagneticField mag_raw_msg = sensor_msgs::msg::MagneticField();
            mag_raw_msg.header = std_msgs::msg::Header();
            mag_raw_msg.header.frame_id = this->frame_id_;
            mag_raw_msg.header.stamp = this->now();

            mag_raw_msg.magnetic_field = geometry_msgs::msg::Vector3();
            packVector3(packet.rawMagnetometerData, mag_raw_msg.magnetic_field);
            double mag_cov = this->magnetic_field_stddev_ * this->magnetic_field_stddev_;
            mag_raw_msg.magnetic_field_covariance =
                    { mag_cov, 0.0f, 0.0f,
                      0.0f, mag_cov, 0.0f,
                      0.0f, 0.0f, mag_cov};

            mag_raw_pub_->publish(mag_raw_msg);
        }

        // temperature
        {
            sensor_msgs::msg::Temperature temperature_msg = sensor_msgs::msg::Temperature();
            temperature_msg.header = std_msgs::msg::Header();
            temperature_msg.header.frame_id = this->frame_id_;
            temperature_msg.header.stamp = this->now();

            temperature_msg.temperature = packet.temperatureC;

            temperature_pub_->publish(temperature_msg);
        }

    }
}

void Yost3SpaceDriver::packVector3(const float data[], geometry_msgs::msg::Vector3 &vec3, float multiplier)
{
    vec3.x = multiplier * data[0];
    vec3.y = multiplier * data[1];
    vec3.z = multiplier * data[2];
}

void Yost3SpaceDriver::packQuat4(const float data[], geometry_msgs::msg::Quaternion &quat)
{
    // the user manual (sec 4.4) says that it is x, y, z, w order
    quat.x = data[0];
    quat.y = data[1];
    quat.z = data[2];
    quat.w = data[3];
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Yost3SpaceDriver>());
    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}