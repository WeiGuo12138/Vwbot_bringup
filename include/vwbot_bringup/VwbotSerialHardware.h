#ifndef VWBOTSERIALHARDWARE_H_
#define VWBOTSERIALHARDWARE_H_

#include <cstdint>
#include <cstring>
#include <queue>
#include <vwbot_bringup/BoostSerialCommunicator.h>
#include "utils.h"

namespace vwpp
{
    class VwbotSerialHardware
    {
    public:
        VwbotSerialHardware(std::string model_, std::string port_, int baud_, 
                             int msg_length_,
                             boost_serial_base::flow_control::type fc_type_ = boost_serial_base::flow_control::none, 
                             boost_serial_base::parity::type pa_type_ = boost_serial_base::parity::none, 
                             boost_serial_base::stop_bits::type st_type = boost_serial_base::stop_bits::one);
        virtual ~VwbotSerialHardware();

        struct Velocity2D
        {
        public:
            float x;
            float y;
            float yaw;
        };

	// New 7.12

	struct Orientation
	{
	public:
	    float x;
     	    float y;
            float z;
            float w;	    
	};

	int sendMessage_cmd_vel(Velocity2D vel_);
	int sendMessage_imu(Orientation imu_);
	int sendMessage_MG995(bool key_);
    // New 7.12

    private:

        std::string model;
        std::string port;
        const int baud;
        const int msg_length;

        boost_serial_base::flow_control::type fc_type;
        boost_serial_base::parity::type pa_type;
        boost_serial_base::stop_bits::type st_type;

        BoostSerialCommunicator* boost_serial_communicator;

        lock_t mutex{};

        const uint8_t* msg_stop;
        const uint8_t* msg_start;

    }; // class VwbotSerialHardware

} // namespace vwpp

#endif
