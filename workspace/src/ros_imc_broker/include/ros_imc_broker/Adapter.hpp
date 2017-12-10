//*************************************************************************
// Copyright (C) 2017 FEUP-LSTS - www.lsts.pt                             *
//*************************************************************************
// This program is free software; you can redistribute it and/or modify   *
// it under the terms of the GNU General Public License as published by   *
// the Free Software Foundation; either version 2 of the License, or (at  *
// your option) any later version.                                        *
//                                                                        *
// This program is distributed in the hope that it will be useful, but    *
// WITHOUT ANY WARRANTY; without even the implied warranty of             *
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU       *
// General Public License for more details.                               *
//                                                                        *
// You should have received a copy of the GNU General Public License      *
// along with this program; if not, write to the Free Software            *
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA          *
// 02110-1301 USA.                                                        *
//*************************************************************************
// Author: Paulo Dias                                                     *
// Author: Ricardo Martins (original)                                     *
//*************************************************************************

#ifndef ROS_IMC_BROKER_ADAPTER_HPP_INCLUDED_
#define ROS_IMC_BROKER_ADAPTER_HPP_INCLUDED_

// Boost headers.
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

// ROS headers.
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// IMC headers.
#include <IMC/Base/Factory.hpp>
#include <IMC/Spec/Constants.hpp>

// Local headers.
#include <ros_imc_broker/Mappings.hpp>
#include <ros_imc_broker/TcpLink.hpp>
#include <ros_imc_broker/UdpLink.hpp>

#include <ros_imc_broker/BrokerParamsConfig.h>
#include <ros_imc_broker/AdapterParamsConfig.h>

#define IMC_NULL_ID 0xFFFF
#define IMC_MULTICAST_ID 0x0000
#define IMC_ENTITY_ANY_ID 0xFF

namespace ros_imc_broker
{
  class Adapter
  {
  public:
    Adapter(ros::NodeHandle& node_handle):
      nh_(node_handle),
      udp_client_(NULL),
      udp_multicast_(NULL),
      system_imc_id_(0xFFFF)
    {
      srv_.setCallback(boost::bind(&Adapter::onReconfigure, this, _1, _2));
      advertiseAll();
      subscribeAll();
      setupPeriodicSenders();
    }

    ~Adapter(void)
    {
      stop();
    }

  private:
    long uid_;
    //! ROS node handle.
    ros::NodeHandle& nh_;
    // //! TCP client to DUNE's server.
    // TcpLink* tcp_client_;
    // //! TCP client thread.
    // boost::thread* tcp_client_thread_;
    //! Map of publishers.
    std::map<unsigned, ros::Publisher> pubs_;
    //! Map of subscribers by topic.
    std::map<std::string, ros::Subscriber> subs_;
    //! Dynamic reconfigure server.
    dynamic_reconfigure::Server<ros_imc_broker::AdapterParamsConfig> srv_;
    //! UDP client to DUNE's server.
    UdpLink* udp_client_;
    UdpLink* udp_multicast_;
    std::vector<ros::Timer> timers_;
    // Messages
    IMC::Announce announce_msg_;
    IMC::Heartbeat heartbeat_msg_;
    std::string system_name_;
    int system_imc_id_;
    // Additional external services.
    std::vector<std::string> adi_services_ext_;
    // External services.
    std::set<std::string> uris_ext_;
    IMC::EstimatedState* estimated_state_msg_;

    void
    onReconfigure(ros_imc_broker::AdapterParamsConfig& config, uint32_t level)
    {
      ROS_INFO("reconfigure request: System name %s with IMC ID 0x%04x",
               config.system_name.c_str(),
               config.system_imc_id);

      system_name_ = config.system_name;
      system_imc_id_ = config.system_imc_id;

      config.additional_services;

      start(config.udp_port, config.udp_port_tries, config.multicast_addr,
          config.multicast_port, config.multicast_port_range);
    }

    void
    stop(void)
    {
      if (udp_client_ == NULL)
        return;

      udp_client_->stop();
      delete udp_client_;
      udp_client_ = NULL;

      udp_multicast_->stop();
      delete udp_multicast_;
      udp_multicast_ = NULL;
    }

    //! Start TCP client.
    //! @param[in] addr server address.
    //! @param[in] port server port.
    void
    start(const int& udp_port, const int& udp_port_tries,
        const std::string& multicast_addr, const int& multicast_port,
        const int& multicast_port_range)
    {
      stop();

      uid_ = (long)(ros::Time::now().toSec() / 1E3);

      udp_client_ = new UdpLink(boost::bind(&Adapter::sendToRosBus, this, _1),
          udp_port, udp_port_tries);

      udp_multicast_ = new UdpLink(boost::bind(&Adapter::sendToRosBusMulticast, this, _1),
          multicast_addr, multicast_port, multicast_port_range);
    }

    std::map<unsigned, ros::Publisher>::iterator
    advertise(const std::string& msg_name, unsigned msg_id)
    {
      std::map<unsigned, PublisherCreator>::const_iterator itr = publisher_creators.find(msg_id);
      if (itr == publisher_creators.end())
        throw IMC::InvalidMessageId(msg_id);

      std::string topic("IMC/In/");
      topic.append(msg_name);

      return pubs_.insert(std::pair<unsigned, ros::Publisher>(msg_id, itr->second(nh_, topic, 1000, false))).first;
    }

    void
    sendToRosBusMulticast(const IMC::Message* msg)
    {
      // Filter self sent messages
      sendToRosBus(msg);
    }

    //! Publish an IMC message to the ROS message bus.
    //! @param[in] msg message instance.
    void
    sendToRosBus(const IMC::Message* msg)
    {
      std::map<unsigned, ros::Publisher>::iterator itr = pubs_.find(msg->getId());
      if (itr == pubs_.end())
        return;

      std::map<unsigned, Publisher>::const_iterator titr = publisher_by_id.find(msg->getId());
      if (titr == publisher_by_id.end())
        return;

      titr->second(itr->second, msg);
    }

    //! Send message to TCP server.
    //! @param[in] msg message instance.
    template<typename T>
    void
    sendToImcSystems(const T& msg)
    {
      T* nMsg = msg.clone();
      nMsg->setSource((uint16_t)system_imc_id_);
      if (nMsg->getTimeStamp() <= 0)
        nMsg->setTimeStamp(ros::Time::now().toSec());
      //@FIXME Set the proper destination
      udp_client_->send(nMsg, "127.0.0.1", 6001);

      if (nMsg->getId() == IMC::EstimatedState::getIdStatic())
      { // Let us save the estimated state
        if (estimated_state_msg_ == NULL)
          delete estimated_state_msg_;
        estimated_state_msg_ = IMC::EstimatedState::cast(nMsg);
      }
      else
      {
        delete nMsg;
      }
      nMsg = NULL;

      ROS_INFO("sent.....x %d:", udp_client_->isConnected());
    }

    //! Subscribe to all IMC messages received over the ROS message bus.
    void
    advertiseAll(void)
    {
      std::vector<std::string> abbrevs;
      IMC::Factory::getAbbrevs(abbrevs);

      std::vector<std::string>::const_iterator itr = abbrevs.begin();
      for (; itr != abbrevs.end(); ++itr)
        advertise(*itr, IMC::Factory::getIdFromAbbrev(*itr));
    }

    //! Subscribe to all IMC messages received over the ROS message bus.
    void
    subscribeAll(void)
    {
      std::vector<std::string> abbrevs;
      IMC::Factory::getAbbrevs(abbrevs);

      std::vector<std::string>::const_iterator itr = abbrevs.begin();
      for (; itr != abbrevs.end(); ++itr)
      {
        std::string topic = std::string("IMC/Out/") + *itr;

        if (false)
        {
          continue;
        }
#define MESSAGE(id, abbrev, md5)                                        \
        else if (*itr == #abbrev)                                       \
        {                                                               \
          ROS_INFO("subscribing: %s", topic.c_str());                  \
          ros::Subscriber sub = nh_.subscribe(topic, 1000, &Adapter::sendToImcSystems<IMC::abbrev>, this); \
          subs_.insert(std::pair<std::string, ros::Subscriber>(topic, sub)); \
        }
#include <IMC/Spec/Factory.xdef>
      }
    }

    void
    setupPeriodicSenders()
    {
      timers_.push_back(nh_.createTimer(ros::Duration(1), &Adapter::heartbeatSender, this));
      timers_.push_back(nh_.createTimer(ros::Duration(10), &Adapter::announceSender, this));
    }

    void
    heartbeatSender(const ros::TimerEvent&)
    {
      heartbeat_msg_.setTimeStamp(0);
      sendToImcSystems(heartbeat_msg_);
    }

    void
    announceSender(const ros::TimerEvent&)
    {
      announce_msg_.sys_name = system_name_;
      announce_msg_.sys_type = IMC::SYSTEMTYPE_UUV;
      announce_msg_.owner = IMC_NULL_ID;
      if (estimated_state_msg_ != NULL)
      {
        //@FIXME Calc lst, lon, height from estimated state
        announce_msg_.lat = estimated_state_msg_->lat;
        announce_msg_.lon = estimated_state_msg_->lon;
        announce_msg_.height = estimated_state_msg_->height;
      }


      std::ostringstream vers;
      vers << "ros://0.0.0.0/version/" << ROS_VERSION;
      uris_ext_.insert(vers.str());

      std::ostringstream uid;
      uid << "ros://0.0.0.0/uid/" << uid_;
      uris_ext_.insert(uid.str());

      std::ostringstream imcvers;
      imcvers << "imc+info://0.0.0.0/version/" << IMC_CONST_VERSION;
      uris_ext_.insert(imcvers.str());

      announce_msg_.setDestination(IMC_MULTICAST_ID);
      announce_msg_.setTimeStamp(0);
      sendToImcSystems(announce_msg_);
    }

  };
}

#endif
