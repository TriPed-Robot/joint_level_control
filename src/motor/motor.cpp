#include <stdexcept>
#include <sstream>
#include <cerrno>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "joint_level_control/motor/motor.h"


Motor::Motor(const std::string& can_name, uint8_t can_id)
    : can_socket_(-1), current_control_(can_id)
{
    if((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        throw std::system_error();
    }
    
    ifreq interface_request;
    strcpy(interface_request.ifr_name, can_name.c_str());
    ioctl(can_socket_, SIOCGIFINDEX, &interface_request);
    
    sockaddr_can can_address;
    memset(&can_address, 0, sizeof(can_address));
    can_address.can_family = AF_CAN;
    can_address.can_ifindex = interface_request.ifr_ifindex;
    if((bind(can_socket_, (sockaddr*)&can_address, sizeof(can_address))) < 0)
    {
        throw std::system_error();
    }

    struct timeval timeout;
    timeout.tv_usec = 150000;
    if(setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        throw std::system_error();
    }
    
    startAcknowldegeTask();

   auto scan_msg = ScanMessage();
   auto frame = scan_msg.getMessage();
   write(can_socket_, frame, sizeof(struct can_frame));
   auto ping_msg = PingMessage(can_id); 
   write(can_socket_, ping_msg.getMessage(), sizeof(struct can_frame));
}


Motor::~Motor()
{
    close(can_socket_);
}


void Motor::setCurrent(double current)
{ 
    current_control_.setCurrent(current*10); // 10 is equal to 1A! 0.1 A resolution!
    can_frame* p_message = current_control_.getMessage();
    writeCAN(p_message);
}

int Motor::writeCAN(can_frame* frame) 
{
    if(write(can_socket_, frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        return 1;
    }
    return 0;
}


void Motor::startAcknowldegeTask()
{
    auto acknowledge_task = [](int can_socket)
    {
        while(true)
        {
            usleep(200000);
            struct can_frame message;
            if(sizeof(struct can_frame) != read(can_socket, &message, sizeof(struct can_frame)))
            {
                continue;
            }
            else
            { 
                continue;
                // TODO: still needs to be impl.
            }
        }
    };
    
    acknowledge_thread_ = std::thread(acknowledge_task, this->can_socket_);
}
