#pragma once

#include <iostream>
#include <string>
#include <boost/asio.hpp>

//using namespace boost::asio;
//using ip::tcp;


class TcpServerBoost {
public:
    TcpServerBoost(std::string ip_address, int port);
    std::string                        receive();
    void                               send(const std::string& message);

private:
    std::string                        ip_address_;
    int                                port_;
    std::size_t                        receive_message_max_size_;

    boost::asio::io_service            io_service_;
    boost::asio::ip::tcp::acceptor     acceptor_;

public:
    boost::asio::ip::tcp::socket       socket_;
};
