#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

// IMC headers.
#include <IMC/Base/Parser.hpp>

// ROS headers.
#include <ros/ros.h>

class TcpConnection : public boost::enable_shared_from_this<TcpConnection> {
 public:
  typedef boost::shared_ptr<TcpConnection> Ptr;

  static Ptr Create(boost::asio::io_service& io_service) {
    return Ptr(new TcpConnection(io_service));
  }

  boost::asio::ip::tcp::socket& GetSocket() {
    return socket_;
  }

  void Start() {
    std::string s = socket_.remote_endpoint().address().to_string();
    int port = socket_.remote_endpoint().port();
    ROS_INFO_STREAM("TCP Connection from " << s << ":" << port);
  }

private:
  TcpConnection(boost::asio::io_service& io_service)
    : socket_(io_service) {
  }

  boost::asio::ip::tcp::socket socket_;
};

class TcpServer {
public:
  TcpServer(boost::asio::io_service* io_service,
            boost::function<void (IMC::Message*)> recv_handler,
            const uint16_t& port = 6001)
    : io_service_(io_service),
      acceptor_(*io_service_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)),
      recv_handler_(recv_handler), running_(false) {
    ROS_INFO("TcpServer");
  }

  void Write(const IMC::Message* message) {
    ROS_INFO("Write");
    if (!running_) return;
    uint16_t rv = IMC::Packet::serialize(message, (uint8_t*)out_buffer_, sizeof(out_buffer_));
    try {
      for (size_t i = 0; i < clients_.size(); i++) {
        boost::asio::write(clients_[i]->GetSocket(), boost::asio::buffer(out_buffer_, rv));
      }
    } catch (std::exception& e) {
      ROS_WARN("Error: %s", e.what());
    }
  }

  void Read() {
    ROS_INFO("Read");
    if (!running_) return;
    for (size_t i = 0; i < clients_.size(); i++) {
      size_t rv = clients_[i]->GetSocket().read_some(
          boost::asio::buffer(in_buffer_, sizeof(in_buffer_)));
      for (size_t i = 0; i < rv; ++i) {
        IMC::Message* m = parser_.parse((uint8_t)in_buffer_[i]);
        if (m) {
          recv_handler_(m);
          delete m;
        }
      }
    }
  }

  void operator()() {
    ROS_INFO("IMC TCP server up and running...");
    io_service_->run();
    if (!running_) running_ = true;
    StartAccept();
    while (true) {
      if (clients_.size() > 0) {
        Read();
      } else {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      }
      boost::this_thread::interruption_point();
    }
  }

private:
  void StartAccept() {
    ROS_INFO("StartAccept");
    if (!running_) return;
    TcpConnection::Ptr new_connection =
      TcpConnection::Create(acceptor_.get_io_service());

    acceptor_.async_accept(new_connection->GetSocket(),
        boost::bind(&TcpServer::HandleAccept, this, new_connection,
          boost::asio::placeholders::error));
  }

  void HandleAccept(TcpConnection::Ptr new_connection,
      const boost::system::error_code& error) {
    ROS_INFO("HandleAccept");
    if (!error) {
      ROS_INFO("IMC TCP server: new connection started");
      clients_.push_back(new_connection);
      new_connection->Start();
      // prepare for new accepts
      StartAccept();
    }
  }

  boost::asio::io_service* io_service_;
  boost::asio::ip::tcp::acceptor acceptor_;
  std::vector<TcpConnection::Ptr> clients_;

  //! Incoming data buffer.
  char in_buffer_[1024];
  //! Outgoing data buffer.
  char out_buffer_[1024];
  //! Callback for received IMC messages.
  boost::function<void (IMC::Message*)> recv_handler_;
  //! IMC message parser.
  IMC::Parser parser_;
  bool running_;
};
