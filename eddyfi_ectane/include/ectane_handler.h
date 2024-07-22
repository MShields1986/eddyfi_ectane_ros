#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <thread>

#include <MinimalSocket/udp/UdpSocket.h>

#include "c_sdk_data_structures.h"
#include "udp_connection.h"


class EctaneHandler
{
public:
    explicit EctaneHandler();
    void run();

    void doNothing();
    void socketInitialise();
    void connecttoDevice();
    void setUpDevice();
    void balanceDevice();
    void acquireData();
    void stopData();
    void disconnectDevice();

private:
    void spinSender();
    void spinReceiver();

    void populateCommandPacket();
    std::string commandToString(CommandPacket command);
    void changeState(State state);

    bool*                           kill_now_;

    std::string                     ectane_address_;

    std::string                     recipient_address_;
    MinimalSocket::Port             send_port_;
    UdpConnection                   sender_;

    std::string                     sender_address_;
    MinimalSocket::Port             receive_port_;
    UdpConnection                   receiver_;

    State                           state_;
    Configurations                  configuration_;
    int32_t                         acquisition_rate_;
    int32_t                         gain_;
    int32_t                         frequency_;
    float                           voltage_;

    CommandPacket                   command_packet_;
};
