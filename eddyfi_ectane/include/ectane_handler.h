#pragma once

#include <iostream>
#include <optional>
#include <string>
#include <cstring>
#include <thread>
#include <vector>

#include <BoostSocketWrappers/tcp_server_boost.h>
#include <BoostSocketWrappers/udp_client_boost.h>

#include "c_sdk_data_structures.h"

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
                                          const std::vector<int32_t>& frequencies,
                                          const std::vector<float>& voltages
                                          );


    // Blocks until the C SDK sends a data cycle and returns the words received, or returns std::nullopt once
    // cancelReadData() is called. Throws on a socket error.
    std::optional<std::vector<int>> readData();
    // Makes a readData() blocked in another thread return std::nullopt, or the next one if none is waiting
    void                            cancelReadData();

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

    // Ectane
    std::string                     ectane_address_;

    // C SDK Data Sender
    std::string                     sender_address_;
    //MinimalSocket::Port             receive_port_;
    int                             receive_port_;
    //TcpServer                       receiver_;
    //TcpServerBoost                  receiver_;
    BoostSocketWrappers::UdpClientBoost receiver_;

    // C SDK State Command Listener
    std::string                     recipient_address_;
    //MinimalSocket::Port             send_port_;
    int                             send_port_;
    //TcpServer                       sender_;
    BoostSocketWrappers::TcpServerBoost sender_;

    // Internal Representation of Command State
    State                           state_;
public:
    const State                     &state() const { return state_; };

private:
    Configurations                  configuration_;
    int32_t                         acquisition_rate_;
    int32_t                         gain_;
    std::vector<int32_t>            frequencies_;
    std::vector<float>              voltages_;

    CommandPacket                   command_packet_;
    std::string                     command_string_;
};
