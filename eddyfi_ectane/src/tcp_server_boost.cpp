#include "tcp_server_boost.h"



TcpServerBoost::TcpServerBoost(std::string ip_address, int port)
     : ip_address_(ip_address), 
       port_(port),
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
       io_service_(),
       acceptor_(io_service_),
       socket_(io_service_)
{
    acceptor_.open(boost::asio::ip::tcp::v4());

    int one = 1;
    setsockopt(acceptor_.native_handle(), SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &one, sizeof(one));

    acceptor_.bind( boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port_) );
    acceptor_.listen();
    acceptor_.accept(socket_);
}


std::string TcpServerBoost::receive() {
    boost::asio::streambuf buf;
    boost::asio::streambuf::mutable_buffers_type bufs = buf.prepare(receive_message_max_size_);

    boost::system::error_code error;

    size_t n = boost::asio::read(this->socket_, bufs, error);

    if(error) {
        std::cout << "read error: " << error << std::endl;
    }

    buf.commit(n);

    std::string data = boost::asio::buffer_cast<const char*>(buf.data());
    return data;
}


void TcpServerBoost::send(const std::string& message) {
  const std::string msg = message;
  boost::asio::write(socket_, boost::asio::buffer(msg));
}
