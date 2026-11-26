#include "tcp_server_boost.h"



TcpServerBoost::TcpServerBoost(std::string ip_address, int port)
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
       io_context_(),
       acceptor_(io_context_),
       socket_(io_context_)
{
    acceptor_.open(boost::asio::ip::tcp::v4());

    int one = 1;
    setsockopt(acceptor_.native_handle(), SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &one, sizeof(one));

    acceptor_.bind( boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port_) );
    acceptor_.listen();
    acceptor_.accept(socket_);

    //std::cout << "TCP Server hosted on " << ip_address_ << ":" << port_ << std::endl;
}


std::vector<int> TcpServerBoost::receive() {
    boost::asio::streambuf buf;
    boost::asio::streambuf::mutable_buffers_type bufs = buf.prepare(receive_message_max_size_);

    boost::system::error_code error;

    size_t n = boost::asio::read(this->socket_, bufs, error);

    if(error) {
        throw boost::system::system_error(error, "TCP receive failed");
    }

    buf.commit(n);

    std::vector<int> data(receive_message_max_size_ / sizeof(int));
    boost::asio::buffer_copy(boost::asio::buffer(data), buf.data());
    //std::cout << "TCP Server yielded a vector<int> of " << data.size() * 32 / 8 << " bytes" << std::endl;

    return data;
}


void TcpServerBoost::send(const std::string& message) {
  const std::string msg = message;
  boost::asio::write(socket_, boost::asio::buffer(msg));
}
