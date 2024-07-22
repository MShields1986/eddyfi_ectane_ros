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
    void                            spinSenderTest();
    void                            spinReceiverTest();
    void                            runTest();

    void                            doNothing();
    void                            socketInitialise();
    void                            connecttoDevice();
    void                            setUpDevice();
    void                            balanceDevice();
    void                            acquireData();
    void                            stopData();
    void                            disconnectDevice();

private:
    void                            updateCommandPacket();
    void                            updateCommandString();
    void                            changeState(State state);

    bool*                           kill_now_;

    // Ectane
    std::string                     ectane_address_;

    // C SDK State Command Listener
    std::string                     recipient_address_;
    MinimalSocket::Port             send_port_;
    UdpConnection                   sender_;

    // C SDK Data Sender
    std::string                     sender_address_;
    MinimalSocket::Port             receive_port_;
    UdpConnection                   receiver_;

    // Internal Representation of Command State
    State                           state_;
    Configurations                  configuration_;
    int32_t                         acquisition_rate_;
    int32_t                         gain_;
    int32_t                         frequency_;
    float                           voltage_;

    CommandPacket                   command_packet_;
    std::string                     command_string_;
};
