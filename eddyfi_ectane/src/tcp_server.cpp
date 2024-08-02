#include "tcp_server.h"

#include <climits>


TcpServer::TcpServer(std::string ip_address, MinimalSocket::Port port)
     : ip_(ip_address), 
       port_(port),
       address_(ip_, port_),
       receive_message_max_size_(   // In Bytes
        (
        32                          // Timeslots per observation
        * 8                         // Channels per timeslot
        * 5                         // Fields per channel (5 Freqs + 1 Voltage)
        //+ 3                         // Encoders
        //+ 32 / 4                    // Saturations
        //+ 1                         // Line 583 of hostdemo.c says ten | Seems wrong to me looks like it ought to be one
        )
        * 32                        // Bits per field
        / CHAR_BIT                  // Bits per byte
        ),
       socket_(port_, MinimalSocket::AddressFamily::IP_V4),
       connected_socket_([this](){socket_.open(); return MinimalSocket::tcp::TcpConnectionBlocking(socket_.acceptNewClient());}())
       //connected_socket_([this](){socket_.open(); return std::optional<MinimalSocket::tcp::TcpConnectionBlocking>(socket_.acceptNewClient());}())
{
    // None Blocking
    //connected_socket_->turnToNonBlocking();
}


void TcpServer::send(std::string& message) {
    // Blocking
    connected_socket_.send(message);

    // None Blocking
    //connected_socket_->send(message);
}


std::string TcpServer::receive() {
    // Blocking
    std::string received_message = connected_socket_.receive(receive_message_max_size_);

    // None Blocking
    //std::string received_message = connected_socket_->receive(receive_message_max_size_);
    return received_message;
}

/*
char TcpServer::receive() {
    MinimalSocket::BufferView view{&buffer_[0], receive_message_max_size_};
    //MinimalSocket::BufferView view{buffer_[0], receive_message_max_size_};
    size_t received_bytes = connected_socket_.receive(view);
            // received_bytes will contain the number of bytes
            // actually received and written into the buffer,
            // from the specified starting point
    return *buffer_;
}
*/