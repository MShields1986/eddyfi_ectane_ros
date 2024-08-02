#pragma once

#include <iostream>
#include <string>
#include <MinimalSocket/tcp/TcpServer.h>

class TcpServer {
public:
    TcpServer(std::string ip_address, MinimalSocket::Port port);
    void send(std::string& message);
    std::string receive();
    //char receive();

private:
    void openAndConnect();
    std::string ip_;
    MinimalSocket::Port port_;
    MinimalSocket::Address address_;
    std::size_t receive_message_max_size_;

    // Blocking
    MinimalSocket::tcp::TcpServer<true> socket_;
    MinimalSocket::tcp::TcpConnectionBlocking connected_socket_;

    // None Blocking
    //MinimalSocket::tcp::TcpServer<false> socket_;
    //std::optional<MinimalSocket::tcp::TcpConnectionBlocking> connected_socket_;
    ////MinimalSocket::tcp::TcpConnectionNonBlocking connected_socket_noblock_;

public:
    //char buffer_[(32 * 8 * 5 ) * 32 / 8];
};