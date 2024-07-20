#pragma once

#include <iostream>
#include <string>
#include <thread>

#include <MinimalSocket/udp/UdpSocket.h>

#include "c_sdk_data_structures.h"
#include "udp_connection.h"


class EctaneHandler
{
public:
    explicit EctaneHandler();
    void run();

private:
    void sendPacket(std::string message, std::string address, MinimalSocket::Port port);
    void spinSender();
    void spinReceiver();
    std::string commandToString(CommandPacket command);

    bool*                           kill_now_;

    std::string                     recipient_address_;
    MinimalSocket::Port             send_port_;
    UdpConnection                   sender_;

    std::string                     sender_address_;
    MinimalSocket::Port             receive_port_;
    UdpConnection                   receiver_;

    State                           state_;
    CommandPacket                   command_packet_;
};
