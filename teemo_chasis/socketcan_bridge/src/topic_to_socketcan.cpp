/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_interface/string.h>
#include <string>
#include <string.h>
#include <chrono>
#include <ros/time.h>
// #include <zmq.hpp>// 引入ZeroMQ库
#include <iostream>
// #include <zmqpp/zmqpp.hpp>// 引入ZeroMQ库

namespace socketcan_bridge
{
TopicToSocketCAN::TopicToSocketCAN(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
                                   can::DriverInterfaceSharedPtr driver)
{
    can_topic_ = nh->subscribe<can_msgs::Frame>("sent_messages", 5,
                    boost::bind(&TopicToSocketCAN::msgCallback, this, _1));
    
    driver_ = driver;
}

void TopicToSocketCAN::setup()
{
    state_listener_ = driver_->createStateListener(
            can::StateInterface::StateDelegate(this, &TopicToSocketCAN::stateCallback));
}

void TopicToSocketCAN::msgCallback(const can_msgs::Frame::ConstPtr& msg)
{
    // translate it to the socketcan frame type.
    can_msgs::Frame m = *msg.get();     // ROS message
    can::Frame f;                       // socketcan type

    // debug
    // std::cout<<m.header.stamp.nsec << std::endl;
    //debug end
    std::chrono::nanoseconds ns(m.header.stamp.nsec);
    std::chrono::seconds s(m.header.stamp.sec);
    std::chrono::system_clock::time_point tp(s + ns);

    //check chrono ture
    // std::cout << "topic time s " << m.header.stamp.sec << " topic time ns " <<m.header.stamp.nsec << std::endl;
    // std::cout << "systemtime s " <<s.count() << " systemtime ns " <<ns.count() << " add "<< tp.time_since_epoch().count()<<std::endl;
    //check result ture

    // std::cout << "time point: " << tp.time_since_epoch().count() << std::endl;
    // std::cout << "time point: " << tp.time_since_epoch().count() << std::endl;


    convertMessageToSocketCAN(m, f);

    if (!f.isValid())  // check if the id and flags are appropriate.
    {
        // ROS_WARN("Refusing to send invalid frame: %s.", can::tostring(f, true).c_str());
        // can::tostring cannot be used for dlc > 8 frames. It causes an crash
        // due to usage of boost::array for the data array. The should always work.
        ROS_ERROR("Invalid frame from topic: id: %#04x, length: %d, is_extended: %d", m.id, m.dlc, m.is_extended);
        return;
    }

    bool res = driver_->send(f);
    if (!res)
    {
        ROS_ERROR("Failed to send message: %s.", can::tostring(f, true).c_str());
    }
    std::chrono::nanoseconds nsec_tmp(ros::Time::now().nsec);
    std::chrono::seconds sec_tmp(ros::Time::now().sec);
    std::chrono::system_clock::time_point tp_new(sec_tmp + nsec_tmp);
    
    //zmq to TCP or UDP port  for debug.
    // zmqpp::context  context;
    // zmqpp::socket socket(context, ZMQ_REQ);
    // zmqpp::socket_type type = zmqpp::socket_type::reply;
    // zmqpp::socket socket (context, type);
    // socket.connect("tcp://localhost:1211");
    // zmqpp::message topic_time_zmq,real_message_zmq;
    // topic_time_zmq << tp_new.time_since_epoch().count();
    // socket.send(topic_time_zmq);
    //initialize
    std::chrono::duration<double> elapsed = tp_new - tp;
    // std::cout << "ros";
    std::cout << "cmd_time" << "elapsed time: " << elapsed.count() << "s\n";
}

void TopicToSocketCAN::stateCallback(const can::State & s)
{
    std::string err;
    driver_->translateError(s.internal_error, err);
    if (!s.internal_error)
    {
        ROS_INFO("State: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
    }
    else
    {
        ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
    }
}
};  // namespace socketcan_bridge
