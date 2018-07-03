#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "vmu931/commands.hpp"
#include "vmu931/sensor.hpp"
#include "vmu931/types.hpp"
#include <boost/asio.hpp>
#include <algorithm>
#include <thread>

template<typename T>
constexpr bool all_equal(const T& first, const T& second)
{
    return first == second;
}

template<typename T, typename... Args>
constexpr bool all_equal(const T& first, const T& second, const Args&... args)
{
    return first == second ? all_equal(second, args...) : false;
}

class TimeCache
{
public:
    TimeCache() :
        TimeCache(0) {}

    TimeCache(uint32_t device_time) :
        m_device_time(device_time), m_ros_time(ros::Time::now()) {}

    const ros::Time& look_up(uint32_t device_time)
    {
        if (device_time != m_device_time) {
            m_device_time = device_time;
            m_ros_time = ros::Time::now();
        }

        return m_ros_time;
    }

private:
    uint32_t m_device_time;
    ros::Time m_ros_time;
};

class DeviceThread
{
public:
    DeviceThread(const std::string& device_name, const std::string& frame_id,
            const ros::Publisher& pub_imu, const ros::Publisher& pub_mf) :
        m_sensor(boost::asio::serial_port(m_io_service, device_name)),
        m_publisher_imu(pub_imu), m_publisher_mf(pub_mf)
    {
        // TODO set covariance matrices (consult VMU931 user guide and maybe own measurements)
        m_imu_msg.header.frame_id = frame_id;
        m_mf_msg.header.frame_id = frame_id;
    }

    void run()
    {
        m_sensor.register_sink([this](vmu931::Accelerometers accel) {
           // sensor_msgs/Imu - linear_acceleration
           m_sensor_accel = accel;
           publish_imu();
        });
        m_sensor.register_sink([this](vmu931::Gyroscopes gyro) {
            // sensor_msgs/Imu.angular_velocity
            m_sensor_gyro = gyro;
            publish_imu();
        });
        m_sensor.register_sink([this](vmu931::Magnetometers magneto) {
            // sensor_msgs/MagneticField.magnetic_field (VMU931 provides micro Tesla)
            m_sensor_magneto = magneto;
            publish_mf();
        });
        m_sensor.register_sink([this](vmu931::Quaternions quat) {
            // sensor_msgs/Imu.orientation
            m_sensor_quat = quat;
            publish_imu();
        });

        m_sensor.set_streams({
            vmu931::commands::Accelerometers,
            vmu931::commands::Gyroscopes,
            vmu931::commands::Magnetometers,
            vmu931::commands::Quaternions
        });

        m_thread = std::thread([this]() { m_io_service.run(); });
    }

    void stop()
    {
        m_io_service.stop();
        m_thread.join();
    }

private:
    void publish_imu()
    {
        if (!all_equal(m_sensor_accel.timestamp, m_sensor_gyro.timestamp, m_sensor_quat.timestamp)) {
            // only publish consistent IMU messages, i.e. from same time stamp
            return;
        }

        static const double pi = std::acos(-1.0);
        static const double g = 9.81; // m/s^2
        static const double dps = pi / 180.0;

        m_imu_msg.linear_acceleration.x = m_sensor_accel.x * g;
        m_imu_msg.linear_acceleration.y = m_sensor_accel.y * g;
        m_imu_msg.linear_acceleration.z = m_sensor_accel.z * g;

        m_imu_msg.angular_velocity.x = m_sensor_gyro.x * dps;
        m_imu_msg.angular_velocity.y = m_sensor_gyro.y * dps;
        m_imu_msg.angular_velocity.z = m_sensor_gyro.z * dps;

        m_imu_msg.orientation.x = m_sensor_quat.x;
        m_imu_msg.orientation.y = m_sensor_quat.y;
        m_imu_msg.orientation.z = m_sensor_quat.z;

        m_imu_msg.header.stamp = m_time_cache.look_up(m_sensor_quat.timestamp);
        m_publisher_imu.publish(m_imu_msg);
    }

    void publish_mf()
    {
        // VMU931 reports in micro Tesla
        m_mf_msg.magnetic_field.x = m_sensor_magneto.x * 1000.0 * 1000.0;
        m_mf_msg.magnetic_field.y = m_sensor_magneto.y * 1000.0 * 1000.0;
        m_mf_msg.magnetic_field.z = m_sensor_magneto.z * 1000.0 * 1000.0;

        m_mf_msg.header.stamp = m_time_cache.look_up(m_sensor_magneto.timestamp);
        m_publisher_mf.publish(m_mf_msg);
    }

    std::thread m_thread;
    std::string m_device_name;
    boost::asio::io_service m_io_service;
    vmu931::Sensor m_sensor;
    vmu931::Accelerometers m_sensor_accel;
    vmu931::Gyroscopes m_sensor_gyro;
    vmu931::Magnetometers m_sensor_magneto;
    vmu931::Quaternions m_sensor_quat;
    sensor_msgs::Imu m_imu_msg;
    sensor_msgs::MagneticField m_mf_msg;
    const ros::Publisher& m_publisher_imu;
    const ros::Publisher& m_publisher_mf;
    TimeCache m_time_cache;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vmu931");
    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");
    std::string device_name;
    if (nh_private.getParam("device_name", device_name)) {
        ROS_INFO("Opening VMU931 at %s", device_name.c_str());
    } else {
        ROS_ERROR("Failed to get parameter 'device_name'");
        return 1;
    }

    std::string frame_id = nh_private.param<std::string>("frame_id", "");
    ROS_INFO("Using frame_id '%s' in published messages", frame_id.c_str());

    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 200);
    ros::Publisher pub_mf = nh.advertise<sensor_msgs::MagneticField>("mf", 80);
    DeviceThread vmu(device_name, frame_id, pub_imu, pub_mf);

    vmu.run();
    ros::spin();
    vmu.stop();

    return 0;
}
