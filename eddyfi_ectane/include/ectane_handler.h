#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <thread>

#include "c_sdk_data_structures.h"
#include "udp_client_boost.h"
#include "tcp_server_boost.h"

//#include <MinimalSocket/udp/UdpSocket.h>
//#include "tcp_server.h"


class EctaneHandler
{
public:
    explicit EctaneHandler();

    void                            setup(std::string ectane_address,
                                          Configurations configuration,
                                          int32_t acquisition_rate,
                                          int32_t gain,
                                          int32_t frequency,
                                          float voltage
                                          );


    void                            readData();

    void                            doNothing();
    void                            commandSocketInitialise();
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
    std::vector<int>                ec_data_;
    std::vector<int>*               ec_data_ptr_;
public:
    const std::vector<int>*         ec_data_ptr() const { return ec_data_ptr_; };

private:
    // C SDK Data Sender
    std::string                     sender_address_;
    //MinimalSocket::Port             receive_port_;
    int                             receive_port_;
    //TcpServer                       receiver_;
    //TcpServerBoost                  receiver_;
    UdpClientBoost                  receiver_;

    // C SDK State Command Listener
    std::string                     recipient_address_;
    //MinimalSocket::Port             send_port_;
    int                             send_port_;
    //TcpServer                       sender_;
    TcpServerBoost                  sender_;

    // Internal Representation of Command State
    State                           state_;
public:
    const State                     &state() const { return state_; };

private:
    Configurations                  configuration_;
    int32_t                         acquisition_rate_;
    int32_t                         gain_;
    int32_t                         frequency_;
    float                           voltage_;

    CommandPacket                   command_packet_;
    std::string                     command_string_;
};
