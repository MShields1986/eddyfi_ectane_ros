#include "tcp_client.h"

#include<unistd.h> 

TcpClient::TcpClient(std::string ip_address, MinimalSocket::Port port)
     : ip_(ip_address), 
       port_(port),
       address_(ip_, port_),
       receive_message_max_size_(8 * 6 * 32 / 8),
       socket_(address_)
{
    connect();
}


void TcpClient::connect() {
    bool success = false;
    int attempts = 5;

    while (!success && attempts > 0) {
        try {
            success = socket_.open();
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
        attempts--;
        sleep(1);
    }    
}


void TcpClient::send(std::string& message) {
    socket_.send(message);
}


std::string TcpClient::receive() {
    std::string received_message = socket_.receive(receive_message_max_size_);
    return received_message;
}