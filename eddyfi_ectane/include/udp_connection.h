#pragma once

#include <iostream>
#include <string>
#include <MinimalSocket/udp/UdpSocket.h>


class UdpConnection
{
public:
    UdpConnection(std::string ip_address, MinimalSocket::Port port);
    void send(std::string& message);
    std::string receive();

private:
    void openAndConnect();
    std::string ip_;
    MinimalSocket::Port port_;
    MinimalSocket::Address address_;
    std::size_t receive_message_max_size_;

    // TODO: Evaluate blocking vs. non-blocking
    MinimalSocket::udp::Udp<true> socket_;   // Blocking
    // MinimalSocket::udp::Udp<false> socket_;   // Non-blocking
    MinimalSocket::udp::UdpConnected<true> connected_socket_;   // Blocking
    // MinimalSocket::udp::UdpConnected<false> connected_socket_(socket_.connect(address_));   // Non-blocking

};