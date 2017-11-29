//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

// IMC headers.
#include <IMC/Base/Parser.hpp>

// ROS headers.
#include <ros/ros.h>

class UdpServer {
public:
  UdpServer(boost::asio::io_service* io_service,
            const std::string& ip = "224.0.75.69", const uint16_t& port = 30100)
     : io_service_(io_service),
       socket_(*io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)),
       remote_endpoint_(boost::asio::ip::address::from_string(ip), port),
       running_(false) {
    ROS_INFO("UdpServer");
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.set_option(boost::asio::socket_base::broadcast(true));
  }

  void operator()() {
    io_service_->run();
    ROS_INFO("IMC UDP server up and running...");
    if (!running_) running_ = true;
  }

  void Broadcast(const IMC::Message* message) {
    ROS_INFO("Broadcast");
    if (!running_) return;
    uint16_t rv = IMC::Packet::serialize(message, (uint8_t*)out_buffer_, sizeof(out_buffer_));

    socket_.async_send_to(boost::asio::buffer(out_buffer_, rv), remote_endpoint_,
      boost::bind(&UdpServer::HandleSend, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  }
 private:
  void HandleSend(const boost::system::error_code& error,
                  std::size_t bytes_transferred) {
    ROS_INFO("HandleSend");
    ROS_INFO_STREAM("IMC UDP Broadcast sent: " << bytes_transferred
                    << " bytes bytes_transferred!");
  }

  boost::asio::io_service* io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
  char out_buffer_[1024];
  //! IMC message parser.
  IMC::Parser parser_;
  bool running_;
};
