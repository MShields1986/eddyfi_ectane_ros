#pragma once

#include <iostream>
#include <string>
#include <MinimalSocket/tcp/TcpClient.h>

class TcpClient {
public:
    TcpClient(std::string ip_address, MinimalSocket::Port port);
    void connect();
    void send(std::string& message);
    std::string receive();

private:
    void openAndConnect();
    std::string ip_;
    MinimalSocket::Port port_;
    MinimalSocket::Address address_;
    std::size_t receive_message_max_size_;

    MinimalSocket::tcp::TcpClient<true> socket_;

};