#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <boost/asio.hpp>

//using namespace boost::asio;
//using ip::udp;

//using boost::asio::ip::udp;
//using boost::asio::ip::address;


class UdpClientBoost {
public:
    UdpClientBoost(std::string ip_address, int port);
    std::vector<int>                   receive();
    //void                               send(const std::string& message);

private:
    std::string                        ip_address_;
    int                                port_;
    std::size_t                        receive_message_max_size_;

    boost::asio::io_service            io_service_;
    //boost::asio::ip::tcp::acceptor     acceptor_;
    boost::asio::ip::udp::endpoint     remote_endpoint_;

public:
    boost::asio::ip::udp::socket       socket_;
};
