#include <vwbot_bringup/VwbotRosDriver.h>

using namespace std;
using namespace vwpp;

VwbotRosDriver::VwbotRosDriver():
    nh(ros::NodeHandle("~")),
    model("VWBOT_G1"),
    port("/dev/ttyUSB0"),
    baud(115200),
    frame_id("base_link"),
    msg_length(40)
{
    this->node_name = ros::this_node::getName();

    if (nh.hasParam("model"))
    {
        nh.getParam("model", this->model);
        ROS_INFO("%s, use model %s", this->node_name.c_str(), this->model.c_str());
    }
    else
    {
        ROS_WARN("%s, use the default model %s", this->node_name.c_str(), this->model.c_str());
    }


    if (nh.hasParam("port"))
    {
        nh.getParam("port", this->port);
        ROS_INFO("%s, use port %s", this->node_name.c_str(), this->port.c_str());
    }
    else
    {
        ROS_WARN("%s, use the default port %s", this->node_name.c_str(), this->port.c_str());
    }

    if (nh.hasParam("baud"))
    {
        nh.getParam("baud", this->baud);
        ROS_INFO("%s, use baud %d", this->node_name.c_str(), this->baud);
    }
    else
    {
        ROS_WARN("%s, use the default baud %d", this->node_name.c_str(), this->baud);
    }

    if (nh.hasParam("msg_length"))
    {
        nh.getParam("msg_length", this->msg_length);
        ROS_INFO("%s, msg_length %d", this->node_name.c_str(), this->msg_length);
    }
    else
    {
        ROS_WARN("%s, use the default msg_length %d", this->node_name.c_str(), this->msg_length);
    }

    this->vwbot_serial_hardware = new VwbotSerialHardware(this->model,
                                                    this->port,
                                                    this->baud,
                                                    this->msg_length);

    if (nh.hasParam("frame_id"))
    {
        nh.getParam("frame_id", this->frame_id);
        ROS_INFO("%s, use frame_id %s", this->node_name.c_str(), this->frame_id.c_str());
    }
    else
    {
        ROS_WARN("%s, use the default frame_id %s", this->node_name.c_str(), this->frame_id.c_str());
    }

    // TODO: Consider add to the initial list.
    std::string cmd_vel_sub_topic_name = "/cmd_vel_unstamped";
    std::string imu_sub_topic_name = "/imu";
    std::string MG995_topic_name="/send_to_hand";

    if (nh.hasParam("cmd_vel_topic"))
    {
        nh.getParam("cmd_vel_topic", cmd_vel_sub_topic_name);
        ROS_INFO("%s, use cmd_vel topic name %s", this->node_name.c_str(), cmd_vel_sub_topic_name.c_str());
    }
    else
    {
        ROS_WARN("%s, use the default cmd_vel topic name %s", this->node_name.c_str(), cmd_vel_sub_topic_name.c_str());
    }

    this->cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(cmd_vel_sub_topic_name, 1, &VwbotRosDriver::cmd_vel_stamped_cb, this);

    if (nh.hasParam("imu_topic"))
    {
    	nh.getParam("imu_topic",imu_sub_topic_name);
	ROS_INFO("%s, use imu topic name %s", this->node_name.c_str(), imu_sub_topic_name.c_str());
    }
    else
    {
	ROS_WARN("%s, use the default imu topic name %s", this->node_name.c_str(), imu_sub_topic_name.c_str());
    }

    this->imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_sub_topic_name, 1, &VwbotRosDriver::imu_cb, this);

    if (nh.hasParam("MG995_topic"))
    {
        nh.getParam("MG995_topic",MG995_topic_name);
        ROS_INFO("%s, use MG995 topic name %s", this->node_name.c_str(), MG995_topic_name.c_str());
    }
    else
    {
        ROS_WARN("%s, use the MG995 topic name %s", this->node_name.c_str(), MG995_topic_name.c_str());
    }

    this->MG995_sub = nh.subscribe<std_msgs::Bool>(MG995_topic_name,1,&VwbotRosDriver::MG995_cb,this);
}



VwbotRosDriver::~VwbotRosDriver()
{
    delete this->vwbot_serial_hardware;
}

void VwbotRosDriver::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)

{
    VwbotSerialHardware::Orientation vwbot_imu_orientation{};
    vwbot_imu_orientation.x = msg->orientation.x;
    vwbot_imu_orientation.y = msg->orientation.y;
    vwbot_imu_orientation.z = msg->orientation.z;
    vwbot_imu_orientation.w = msg->orientation.w;
    if (this->vwbot_serial_hardware->sendMessage_imu(vwbot_imu_orientation) != 1)
    {
    	ROS_ERROR("%s, Send Imu_Message failed ", this->node_name.c_str());
    }
}
void VwbotRosDriver::cmd_vel_stamped_cb(const geometry_msgs::Twist::ConstPtr& msg)
{

    VwbotSerialHardware::Velocity2D vwbot_cmd_vel{};
    vwbot_cmd_vel.x = msg->linear.x;
    vwbot_cmd_vel.y = msg->linear.y;
    vwbot_cmd_vel.yaw = msg->angular.z;

    if (this->vwbot_serial_hardware->sendMessage_cmd_vel(vwbot_cmd_vel) != 1)
    {
        ROS_ERROR("%s, Send Cmd_vel_Message failed!", this->node_name.c_str());
    }

}
void VwbotRosDriver::MG995_cb(const std_msgs::Bool::ConstPtr &msg)
{
    bool key;
    key = msg->data;
    if(this->vwbot_serial_hardware->sendMessage_MG995(key)!=1)
    {
        ROS_ERROR("%s, Send MG995_key_message failed!",this->node_name.c_str());
    }
}
