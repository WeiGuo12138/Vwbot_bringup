#include <utility>

#include <utility>

#include <vwbot_bringup/VwbotSerialHardware.h>
#include <vwbot_bringup/utils.h>
#include <eigen3/Eigen/Geometry> 
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>

using namespace vwpp;

VwbotSerialHardware::VwbotSerialHardware(std::string model_, std::string port_, int baud_, 
                                           int msg_length_,
                                           boost_serial_base::flow_control::type fc_type_,
                                           boost_serial_base::parity::type pa_type_, 
                                           boost_serial_base::stop_bits::type st_type_):
    model(std::move(model_)),
    port(std::move(port_)),
    baud(baud_),
    msg_length(msg_length_),
    fc_type(fc_type_),
    pa_type(pa_type_),
    st_type(st_type_)
{

    uint8_t tmp_msg_start[6] = {0xB5, 0x5B, 0x04, 0x01, 0x05, 0xBB};
    uint8_t tmp_msg_stop[6]  = {0xB5, 0x5B, 0x04, 0x02, 0x06, 0xBB};
    
    this->msg_start = tmp_msg_start;
    this->msg_stop  = tmp_msg_stop;

    this->boost_serial_communicator = new BoostSerialCommunicator(this->port, this->baud);
    mutex_init(&(this->mutex));

    if(model == "VWBOT_G1")
    {
        // TODO Add the start and stop header.
        // boost_serial_communicator->sendMessage(msg_stop);
        usleep(1000 * 1000);
        // boost_serial_communicator->sendMessage(msg_start);
        usleep(1000 * 1000);
    }

    std::cout << "\033[32m" << "Start communicate with vwbot..."
              << "\033[0m" << std::endl;

}


VwbotSerialHardware::~VwbotSerialHardware()
{
    delete   this->boost_serial_communicator;
    delete[] this->msg_start;
    delete[] this->msg_stop;
}

 
int VwbotSerialHardware::sendMessage_cmd_vel(VwbotSerialHardware::Velocity2D vel_)
{
    std::vector<uint8_t> vec_msg;
    // Header
    vec_msg.push_back(0xB5);
    vec_msg.push_back(0x5B);
    // Host ID
    vec_msg.push_back(0x01);
    // data length
    vec_msg.push_back(0x06);

    int16_t vel_x = round_float(vel_.x*100);
    auto vel_x_hbits = uint8_t(vel_x >> 8);
    auto vel_x_lbits = uint8_t(vel_x & (0x00FF));
    vec_msg.push_back(vel_x_hbits);
    vec_msg.push_back(vel_x_lbits);

    int16_t vel_y = round_float(vel_.y*100);
    auto vel_y_hbits = uint8_t(vel_y >> 8);
    auto vel_y_lbits = uint8_t(vel_y & (0x00FF));
    vec_msg.push_back(vel_y_hbits);
    vec_msg.push_back(vel_y_lbits);

    int16_t vel_yaw = round_float(vel_.yaw*100);
    auto vel_yaw_hbits = uint8_t(vel_yaw >> 8);
    auto vel_yaw_lbits = uint8_t(vel_yaw & (0x00FF));
    vec_msg.push_back(vel_yaw_hbits);
    vec_msg.push_back(vel_yaw_lbits);


    // checksum, add all the data before the checksum byte, include the headers.
    int data_sum = 0;
    for (auto item : vec_msg)
    {
        data_sum += item;
    }
    uint8_t checksum = (data_sum) % 256;


    vec_msg.push_back(checksum);

    vec_msg.push_back(0xBB);

    //Lock

    lock(&mutex);

    if (this->boost_serial_communicator->sendMessage(vec_msg) == 1)
    {
        unlock(&mutex);
        return 1;
    }
    else 
    {
        std::cerr << "\033[31m" << "Send message failed"
                  << "\033[0m"  << std::endl;
        unlock(&mutex);
        return (-1);
    }

}
int VwbotSerialHardware::sendMessage_imu(VwbotSerialHardware::Orientation imu_)
{
    std::vector<uint8_t> vec_msg;
    // Header
    vec_msg.push_back(0xC5);  
    vec_msg.push_back(0x5C);
    // Host ID
    vec_msg.push_back(0x02);
    // data length
    vec_msg.push_back(0x02);


    // New imu 7.12
    int16_t imu_yaw = round_float(atan(2*(imu_.z*imu_.w+imu_.y*imu_.x)/(1-2*(imu_.z*imu_.z+imu_.y*imu_.y)))/3.1415926*180);
    printf("yaw data: %d\n", imu_yaw);
    auto imu_yaw_hbits = uint8_t(imu_yaw >> 8);
    auto imu_yaw_lbits = uint8_t(imu_yaw & (0x00FF));
    vec_msg.push_back(imu_yaw_hbits);
    vec_msg.push_back(imu_yaw_lbits);

    // checksum, add all the data before the checksum byte, include the headers.
    int data_sum = 0;
    for (auto item : vec_msg)
    {
        data_sum += item;
    }
    uint8_t checksum = (data_sum) % 256;


    vec_msg.push_back(checksum);

    vec_msg.push_back(0xCC);

     // Lock  
     //
    lock(&mutex);

    if (this->boost_serial_communicator->sendMessage(vec_msg) == 1)
    {
        unlock(&mutex);
        return 1;
    }
    else 
    {
        std::cerr << "\033[31m" << "Send message failed"
                  << "\033[0m"  << std::endl;
        unlock(&mutex);
        return (-1);
    }
    
}

int VwbotSerialHardware::sendMessage_MG995(bool key_)
{
    std::vector<uint8_t> vec_msg;
    // Header
    vec_msg.push_back(0xC5);
    vec_msg.push_back(0x5C);
    // Host ID
    vec_msg.push_back(0x02);
    // data length
    vec_msg.push_back(0x01);

    if(key_== 1)
    {
        vec_msg.push_back(0x02);
    }
    if(key_== 0)
    {
        vec_msg.push_back(0x01);
    }

    // New imu 7.12
//    int16_t imu_yaw = round_float(atan(2*(imu_.z*imu_.w+imu_.y*imu_.x)/(1-2*(imu_.z*imu_.z+imu_.y*imu_.y)))/3.1415926*180);
//    printf("yaw data: %d\n", imu_yaw);
//    auto imu_yaw_hbits = uint8_t(imu_yaw >> 8);
//    auto imu_yaw_lbits = uint8_t(imu_yaw & (0x00FF));
//    vec_msg.push_back(imu_yaw_hbits);
//    vec_msg.push_back(imu_yaw_lbits);

    // checksum, add all the data before the checksum byte, include the headers.
    int data_sum = 0;
    for (auto item : vec_msg)
    {
        data_sum += item;
    }
    uint8_t checksum = (data_sum) % 256;

    vec_msg.push_back(checksum);

    vec_msg.push_back(0xCC);

    // Lock
    //
    lock(&mutex);

    if (this->boost_serial_communicator->sendMessage(vec_msg) == 1)
    {
        unlock(&mutex);
        return 1;
    }
    else
    {
        std::cerr << "\033[31m" << "Send message failed"
                  << "\033[0m"  << std::endl;
        unlock(&mutex);
        return (-1);
    }
}

