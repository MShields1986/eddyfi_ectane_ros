#pragma once

#include <iostream>
#include <string>
#include <MinimalSocket/tcp/TcpServer.h>

class TcpConnection {
public:
    TcpConnection(std::string ip_address, MinimalSocket::Port port);
    void send(std::string& message);
    std::string receive();

private:
    void openAndConnect();
    std::string ip_;
    MinimalSocket::Port port_;
    MinimalSocket::Address address_;
    std::size_t receive_message_max_size_;

    MinimalSocket::tcp::TcpServer<true> socket_;
    MinimalSocket::tcp::TcpConnectionBlocking connected_socket_;

};