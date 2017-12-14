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
// Author: Ricardo Martins (original Broker.hpp)                          *
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
#include <ros_imc_broker/Network/UdpLink.hpp>

#include <ros_imc_broker/BrokerParamsConfig.h>
#include <ros_imc_broker/AdapterParamsConfig.h>

#include <ros_imc_broker/Concurrency/RWLock.hpp>
#include <ros_imc_broker/Network/NetworkUtil.hpp>
#include <ros_imc_broker/Util/String.hpp>

#define IMC_NULL_ID 0xFFFF
#define IMC_MULTICAST_ID 0x0000
#define IMC_ENTITY_ANY_ID 0xFF

namespace ros_imc_broker
{
  struct Destination
  {
    // Destination address.
    std::string addr;
    // Destination port.
    unsigned port;
    // True if address is local.
    bool local;
  };

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
    Network::UdpLink* udp_client_;
    Network::UdpLink* udp_multicast_;
    std::string multicast_addr_;
    int multicast_port_;
    int multicast_port_range_;
    std::vector<ros::Timer> timers_;
    // Messages
    IMC::Announce announce_msg_;
    IMC::Heartbeat heartbeat_msg_;
    std::string system_name_;
    IMC::SystemType system_type_;
    int system_imc_id_;
    // Additional external services.
    std::vector<std::string> adi_services_ext_;
    // External services.
    std::set<std::string> uris_ext_;
    IMC::EstimatedState* estimated_state_msg_ = NULL;
    //std::vector<boost::asio::ip::address> network_interfaces_;
    std::vector<Destination> multicast_destinations_;
    std::vector<Destination> static_destinations_;
    bool enable_loopback_;
    Concurrency::RWLock::Mutex mutex_multicast_destinations_;
    Concurrency::RWLock::Mutex mutex_static_destinations_;

    void
    onReconfigure(ros_imc_broker::AdapterParamsConfig& config, uint32_t level)
    {
      ROS_INFO("reconfigure request: System name %s with IMC ID 0x%04x",
               config.system_name.c_str(),
               config.system_imc_id);

      system_name_ = config.system_name;
      system_type_ = translateSystem(config.system_type);
      system_imc_id_ = config.system_imc_id;

      config.additional_services;

      enable_loopback_ = config.enable_loopback;

      Concurrency::RWLock::WriteLock lock(mutex_static_destinations_);
      // Parsing static desinations
      static_destinations_.clear();
      std::vector<std::string> static_dest;
      Util::String::split(config.static_destinations_addrs, ",", static_dest);
      for (unsigned int i = 0; i < static_dest.size(); ++i)
      {
        std::vector<std::string> addr_port;
        Util::String::split(config.static_destinations_addrs, ":", addr_port);
        if (addr_port.size() != 2)
          continue;
        try
        {
          Destination dst;
          dst.port = std::atoi(addr_port[1].c_str());
          dst.addr = addr_port[0];
          dst.local = false;
          static_destinations_.push_back(dst);
        }
        catch (std::exception & ex)
        {
          std::cerr << "[" << boost::this_thread::get_id() << "] Exception: "
              << ex.what() << std::endl;
        }
      }
      lock.unlock();

      start(config.udp_port, config.udp_port_tries, config.multicast_addr,
          config.multicast_port, config.multicast_port_range);
    }

    IMC::SystemType
    translateSystem(std::string type)
    {
      Util::String::toLowerCase(type);

      if (type == "uuv")
        return IMC::SYSTEMTYPE_UUV;
      else if (type == "auv")
        return IMC::SYSTEMTYPE_UUV;
      else if (type == "uav")
        return IMC::SYSTEMTYPE_UAV;
      else if (type == "usv")
        return IMC::SYSTEMTYPE_USV;
      else if (type == "ugv")
        return IMC::SYSTEMTYPE_UGV;
      else if (type == "ccu")
        return IMC::SYSTEMTYPE_CCU;
      else if (type == "mobilesensor")
        return IMC::SYSTEMTYPE_MOBILESENSOR;
      else if (type == "staticsensor")
        return IMC::SYSTEMTYPE_STATICSENSOR;
      else if (type == "humansensor")
        return IMC::SYSTEMTYPE_HUMANSENSOR;
      else if (type == "wsn")
        return IMC::SYSTEMTYPE_WSN;
      else
        return IMC::SYSTEMTYPE_UUV;
    }
    
    void
    stop(void)
    {
      if (udp_client_ == NULL)
        return;

      stopPeriodicSenders();

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
      
      uid_ = (long)(ros::Time::now().toSec() * 1E3);

      udp_client_ = new Network::UdpLink(boost::bind(&Adapter::sendToRosBus, this, _1),
          udp_port, udp_port_tries);

      udp_multicast_ = new Network::UdpLink(boost::bind(&Adapter::sendToRosBusMulticast, this, _1),
          multicast_addr, multicast_port, multicast_port_range);
      
      multicast_addr_ = multicast_addr;
      multicast_port_ = multicast_port;
      multicast_port_range_ = multicast_port_range;

      probeInterfacesForMulticast();
      setupPeriodicSenders();
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
      if (msg->getSource() == system_imc_id_)
        return;

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

    //! @param[in] msg message instance.
    template<typename T>
    void
    sendToImcSystems(const T& msg)
    {
      if (udp_client_ == NULL)
      {
        ROS_WARN("udp is closed");
        return;
      }

      try
      {
        T* nMsg = msg.clone();
        nMsg->setSource((uint16_t)system_imc_id_);
        if (nMsg->getTimeStamp() <= 0)
          nMsg->setTimeStamp(ros::Time::now().toSec());

        //@FIXME Set the proper destination besides the static defined ones
        // udp_client_->send(nMsg, "127.0.0.1", 6001);
        Concurrency::RWLock::ReadLock lock(mutex_static_destinations_);
        for (unsigned int i = 0; i < static_destinations_.size(); ++i)
        {
          udp_client_->send(nMsg, static_destinations_[i].addr, static_destinations_[i].port);
        }
        lock.unlock();

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
      }
      catch (std::exception & ex)
      {
        std::cerr << "[" << boost::this_thread::get_id() << "] Exception: "
            << ex.what() << std::endl;
      }

      ROS_INFO("sent %s:", msg.getName());
    }

    template<typename T>
    void
    sendToImcSystemsMulticast(const T& msg)
    {
      if (udp_multicast_ == NULL)
      {
        ROS_WARN("udp multicast is closed");
        return;
      }

      try
      {
        T* nMsg = msg.clone();
        nMsg->setSource((uint16_t)system_imc_id_);
        if (nMsg->getTimeStamp() <= 0)
          nMsg->setTimeStamp(ros::Time::now().toSec());

        Concurrency::RWLock::ReadLock lock(mutex_multicast_destinations_);
        for (unsigned i = 0; i < multicast_destinations_.size(); ++i)
        {
          udp_multicast_->send(nMsg, multicast_destinations_[i].addr, multicast_destinations_[i].port);
        }
        lock.unlock();

        delete nMsg;
        nMsg = NULL;
      }
      catch (std::exception & ex)
      {
        std::cerr << "[" << boost::this_thread::get_id() << "] Exception: "
            << ex.what() << std::endl;
      }

      ROS_INFO("multicast sent %s:", msg.getName());
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
    setupPeriodicSenders(void)
    {
      timers_.push_back(nh_.createTimer(ros::Duration(30), &Adapter::probeInterfacesForMulticast, this));
      timers_.push_back(nh_.createTimer(ros::Duration(1), &Adapter::heartbeatSender, this));
      timers_.push_back(nh_.createTimer(ros::Duration(10), &Adapter::announceSender, this));
    }

    void
    stopPeriodicSenders(void)
    {
      for (unsigned i = 0; i < timers_.size(); ++i)
      {
        timers_[i].stop();
      }
      timers_.clear();
    }

    void
    probeInterfacesForMulticast(const ros::TimerEvent&)
    {
      probeInterfacesForMulticast();
    }

    void
    probeInterfacesForMulticast(void)
    {
      Concurrency::RWLock::WriteLock lock(mutex_multicast_destinations_);

      multicast_destinations_.clear();

      // Setup loopback.
      if (enable_loopback_)
      {
        for (unsigned i = multicast_port_; i < multicast_port_ + multicast_port_range_; ++i)
        {
          Destination dst;
          dst.port = i;
          dst.addr = "127.0.0.1";
          dst.local = true;
          multicast_destinations_.push_back(dst);
        }
      }

      // Setup multicast.
      for (unsigned i = multicast_port_; i < multicast_port_ + multicast_port_range_; ++i)
      {
        Destination dst;
        dst.port = i;
        dst.addr = multicast_addr_;
        dst.local = false;
        multicast_destinations_.push_back(dst);
      }

      // Setup broadcast.
      {
        for (unsigned i = multicast_port_; i < multicast_port_ + multicast_port_range_; ++i)
        {
          Destination dst;
          dst.port = i;
          dst.addr = "255.255.255.255";
          dst.local = false;
          multicast_destinations_.push_back(dst);
        }

        try
        {
          std::vector<boost::asio::ip::address> itfs = Network::NetworkUtil::getNetworkInterfaces();
          for (unsigned i = 0; i < itfs.size(); ++i)
          {
            if (!itfs[i].is_v4())
              continue;
            
            // Discard loopback addresses. @FIXME any filter
            if (itfs[i].is_loopback() || itfs[i].to_v4().broadcast() == itfs[i].to_v4().broadcast().any())
              continue;

            for (unsigned j = multicast_port_; j < multicast_port_ + multicast_port_range_; ++j)
            {
              Destination dst;
              dst.port = j;
              dst.addr = itfs[i].to_v4().broadcast().to_string();
              dst.local = false;
              multicast_destinations_.push_back(dst);
            }
          }
        }
        catch (std::exception & ex)
        {
          std::cerr << "[" << boost::this_thread::get_id() << "] Exception: "
              << ex.what() << std::endl;
        }
      }

      lock.unlock();

      ROS_INFO("found %d multicast destinations", (int)multicast_destinations_.size());
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
      announce_msg_.sys_type = system_type_;
      announce_msg_.owner = IMC_NULL_ID;
      if (estimated_state_msg_ != NULL)
      {
        //@FIXME Calc lst, lon, height from estimated state
        announce_msg_.lat = estimated_state_msg_->lat;
        announce_msg_.lon = estimated_state_msg_->lon;
        announce_msg_.height = estimated_state_msg_->height;
      }
      else
      {
        ROS_INFO("null EstimatedState!!!");
        announce_msg_.lat = 0;
        announce_msg_.lon = 0;
        announce_msg_.height = 0;
      }

      // Services collection
      announce_msg_.services.clear();

      std::ostringstream vers;
      vers << "ros://0.0.0.0/version/" << ROS_VERSION;
      addURI(announce_msg_, vers.str());

      std::ostringstream uid;
      uid << "ros://0.0.0.0/uid/" << uid_;
      addURI(announce_msg_, uid.str());

      std::ostringstream imcvers;
      imcvers << "imc+info://0.0.0.0/version/" << IMC_CONST_VERSION;
      addURI(announce_msg_, imcvers.str());
      
      std::set<std::string> uris_info;
      try
      {
        std::vector<boost::asio::ip::address> itfs = Network::NetworkUtil::getNetworkInterfaces();
        for (unsigned i = 0; i < itfs.size(); ++i)
        {
          if (!itfs[i].is_v4())
            continue;
          
          // Discard loopback addresses.
          if (itfs[i].is_loopback())
            continue;

          if (udp_client_ != NULL && udp_client_->isConnected())
          {
            std::ostringstream udps;
            udps << "imc+udp://" << itfs[i].to_string() << ":" << udp_client_->bindedPort()
                    << "/";
            uris_info.insert(udps.str());
          }
        }
      }
      catch (std::exception & ex)
      {
        std::cerr << "[" << boost::this_thread::get_id() << "] Exception: "
            << ex.what() << std::endl;
      }

      std::set<std::string>::iterator itr = uris_info.begin();
      for (itr = uris_info.begin(); itr != uris_info.end(); ++itr)
        addURI(announce_msg_, *itr);

      announce_msg_.setDestination(IMC_MULTICAST_ID);
      announce_msg_.setTimeStamp(0);
      sendToImcSystemsMulticast(announce_msg_);
    }

    void
    addURI(IMC::Announce& announce, const std::string& uri)
    {
      if (!announce.services.empty())
        announce.services.append(";");

      announce.services.append(uri);
    }

  };
}

#endif