#include "tcp_connection.h"


TcpConnection::TcpConnection(std::string ip_address, MinimalSocket::Port port)
     : ip_(ip_address), 
       port_(port),
       address_(ip_, port_),
       receive_message_max_size_(2000),
       socket_(port_, MinimalSocket::AddressFamily::IP_V4),
       connected_socket_([this](){socket_.open(); return MinimalSocket::tcp::TcpConnectionBlocking(socket_.acceptNewClient());}()) {
}


void TcpConnection::send(std::string& message) {
    connected_socket_.send(message);
}


std::string TcpConnection::receive() {
    std::string received_message = connected_socket_.receive(receive_message_max_size_);
    return received_message;
}