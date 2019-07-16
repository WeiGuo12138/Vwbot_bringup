#ifndef VWBOTROSDRIVER_H_
#define VWBOTROSDRIVER_H_

#include <string>
#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include <vwbot_bringup/VwbotSerialHardware.h>

namespace vwpp
{
    class VwbotRosDriver
    {
    public:
        VwbotRosDriver();

        virtual ~VwbotRosDriver();

        void cmd_vel_stamped_cb(const geometry_msgs::Twist::ConstPtr &msg);

        void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);


    private:

        VwbotSerialHardware *vwbot_serial_hardware;
        std::queue<VwbotSerialHardware::Velocity2D> que_vwbot_vel2d_data_;

        std::string node_name;
        std::string model;
        std::string port;
        int baud;
        int msg_length;

        std::string frame_id;
        geometry_msgs::Twist cmd_vel_stamped_;
        sensor_msgs::Imu imu_test_;

        ros::NodeHandle nh;
        ros::Subscriber cmd_vel_sub;
        ros::Subscriber imu_sub;
    };

} // namespace vwpp

#endif
