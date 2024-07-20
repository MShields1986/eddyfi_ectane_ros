#include "udp_connection.h"


UdpConnection::UdpConnection(std::string ip_address, MinimalSocket::Port port)
     : ip_(ip_address), 
       port_(port),
       address_(ip_, port_),
       receive_message_max_size_(2000),
       socket_(port_, MinimalSocket::AddressFamily::IP_V4),
       connected_socket_([this](){socket_.open(); return MinimalSocket::udp::UdpConnected<true>(socket_.connect(address_));}())
{
    //openAndConnect();
    //bool success = socket_.open();
    //if (!success) std::cerr << "Failed to open socket" << std::endl;
    //MinimalSocket::udp::UdpConnected<true> connected_socket_(socket_.connect(address_));       // Blocking
    //MinimalSocket::udp::UdpConnected<false> connected_socket_(socket_.connect(address_));   // Non-blocking

    // Trying to have connected_socket_as a member variable either compiles and fails to open the socker at run time
    // or throws the following compiler error:
    //         error: no matching function for call to ‘MinimalSocket::udp::UdpConnected<true>::UdpConnected()’

    // TODO: Unsure how to best initialise connected_socket_ as a class member due to needing to call socket_.open()
    //          - Tried passing in a creator function
    //          - Tried using a member function
    //          - Tried using a static member function
    //          - Tried instantiating then opening
    //          - Tried using a ternary operator - kinda works but not ideal
    //                   connected_socket_(socket_.open() ? socket_.connect(address_) : socket_.connect(address_))
    //          - Tried using a lambda function - better
    //                   connected_socket_([this](){socket_.open(); return MinimalSocket::udp::UdpConnected<true>(socket_.connect(address_));}())
    //          - Try using a smart pointer and put the socket on the heap?
    //          - Try using a factory class?
}


void UdpConnection::send(std::string& message)
{
    connected_socket_.send(message);
}


std::string UdpConnection::receive()
{
    std::string received_message = connected_socket_.receive(receive_message_max_size_);
    return received_message;
}