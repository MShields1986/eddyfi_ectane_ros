#include "udp_connection.h"


UdpConnection::UdpConnection(std::string ip_address, MinimalSocket::Port port)
     : ip_(ip_address), 
       port_(port),
       address_(ip_, port_),
       receive_message_max_size_(8 * 6 * 32 / 8),
       socket_(port_, MinimalSocket::AddressFamily::IP_V4),
       connected_socket_([this](){socket_.open(); return MinimalSocket::udp::UdpConnected<true>(socket_.connect(address_));}()) {
}


void UdpConnection::send(std::string& message) {
    connected_socket_.send(message);
}


std::string UdpConnection::receive() {
    std::string received_message = connected_socket_.receive(receive_message_max_size_);
    return received_message;
}
