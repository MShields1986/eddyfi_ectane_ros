#include "udp_client_boost.h"



UdpClientBoost::UdpClientBoost(std::string ip_address, int port)
     : ip_address_(ip_address), 
       port_(port),
       receive_message_max_size_(   // In Bytes
        (
        32                          // Timeslots per observation
        * 8                         // Channels per timeslot
        * 5                         // Fields per channel (5 Freqs + 1 Voltage)
        //+ 3       NOT FORWARDED FROM SDK ATM  // Encoders
        //+ 32 / 4  NOT FORWARDED FROM SDK ATM  // Saturations
        //+ 1       NOT FORWARDED FROM SDK ATM  // Line 583 of hostdemo.c says ten | Seems wrong to me looks like it ought to be one
        )
        * 32                        // Bits per field
        / CHAR_BIT                  // Bits per byte
        ),
       io_service_(),
       socket_(io_service_, {boost::asio::ip::udp::v4(), (short unsigned int)port_})
{
}


std::vector<int> UdpClientBoost::receive() {
    boost::asio::streambuf buf;
    boost::asio::streambuf::mutable_buffers_type bufs = buf.prepare(receive_message_max_size_);

    boost::system::error_code error;

    size_t n = this->socket_.receive_from(bufs, remote_endpoint_, 0, error);

    if(error) {
        std::cout << "read error: " << error << std::endl;
    }

    buf.commit(n);
    //std::cout << "#################################" << std::endl;
    //std::cout << "            Start Read" << std::endl;
    //std::cout << "#################################" << std::endl;
    //std::cout << "TCP Server read " << n << " bytes" << std::endl;
    //std::cout << "Buffer size " << buf.size() << " bytes" << std::endl;

    std::vector<int> data(32 * 8 * 5);
    boost::asio::buffer_copy(boost::asio::buffer(data), buf.data());
    //std::cout << "TCP Server yielded a vector<int> of " << data.size() * 32 / 8 << " bytes" << std::endl;

    return data;
}

/*
void UdpClientBoost::send(const std::string& message) {
  const std::string msg = message;
  boost::asio::write(socket_, boost::asio::buffer(msg));
}
*/