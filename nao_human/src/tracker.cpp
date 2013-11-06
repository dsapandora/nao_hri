/*
# Copyright 2013 Séverin Lemaignan, EPFL
# Copyright 2011 Daniel Maier, University of Freiburg
# http://www.ros.org/wiki/nao_hri
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
*/



// duration, in seconds, a human frame must be kept published once the human is
// not seen anymore
#define KEEP_HUMAN_ALIVE 100

#define NODE_RATE 10.0

#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "interactions.h"

// Aldebaran includes
#include <alcommon/albroker.h>
#include <alerror/alerror.h>

// Other includes
#include <boost/program_options.hpp>

using namespace std;

static const string camera_frame = "CameraTop_frame";
static const string base_frame = "odom";

/*
from nao_driver import *

import threading
from threading import Thread

*/
class NaoNode
{
   public:
      NaoNode();
      ~NaoNode();
      bool connectNaoQi();
      void parse_command_line(int argc, char ** argv);
   protected:
      std::string m_pip;
      std::string m_ip;
      int m_port;
      int m_pport;
      std::string m_brokerName;
      boost::shared_ptr<AL::ALBroker> m_broker;


};


NaoNode::NaoNode() : m_pip("nao.local"),m_ip("0.0.0.0"),m_port(16712),m_pport(9559),m_brokerName("nao_human_ROSBroker")
{


}

NaoNode::~NaoNode()
{
}

void NaoNode::parse_command_line(int argc, char ** argv)
{
   std::string pip;
   std::string ip;
   int pport;
   int port;
   boost::program_options::options_description desc("Configuration");
   desc.add_options()
      ("help", "show this help message")
      ("ip", boost::program_options::value<std::string>(&ip)->default_value(m_ip),
       "IP/hostname of the broker")
      ("port", boost::program_options::value<int>(&port)->default_value(m_port),
       "Port of the broker")
      ("pip", boost::program_options::value<std::string>(&pip)->default_value(m_pip),
       "IP/hostname of parent broker")
      ("pport", boost::program_options::value<int>(&pport)->default_value(m_pport),
       "port of parent broker")
      ;
   boost::program_options::variables_map vm;
   boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
   boost::program_options::notify(vm);
   m_port = vm["port"].as<int>();
   m_pport = vm["pport"].as<int>();
   m_pip = vm["pip"].as<std::string>();
   m_ip = vm["ip"].as<std::string>();
   cout << "pip is " << m_pip << endl;
   cout << "ip is " << m_ip << endl;
   cout << "port is " << m_port << endl;
   cout << "pport is " << m_pport << endl;

   if (vm.count("help")) {
      std::cout << desc << "\n";
      return ;
   }
}



bool NaoNode::connectNaoQi()
{
   // Need this to for SOAP serialization of floats to work
   setlocale(LC_NUMERIC, "C");
   // A broker needs a name, an IP and a port:
   // FIXME: would be a good idea to look for a free port first
   // listen port of the broker (here an anything)
   try
   {
      m_broker = AL::ALBroker::createBroker(m_brokerName, m_ip, m_port, m_pip, m_pport, false);
   }
   catch(const AL::ALError& e)
   {
      ROS_ERROR( "Failed to connect broker to: %s:%d",m_pip.c_str(),m_port);
      //AL::ALBrokerManager::getInstance()->killAllBroker();
      //AL::ALBrokerManager::kill();
      return false;
   }
   cout << "broker ready." << endl;
   return true;
}

class HumanTracker : public NaoNode
{
public:

    HumanTracker(int argc, char ** argv);
    ~HumanTracker();

    void run();


protected:

    double m_rate;
    InteractionMonitor m_monitor;

    // ROS
    ros::NodeHandle m_nh;
    ros::NodeHandle m_privateNh;

    std::string m_baseFrameId;
    std::string m_cameraFrameId;

    tf::TransformBroadcaster m_transformBroadcaster;
    tf::TransformListener m_listener;
};

HumanTracker::HumanTracker(int argc, char ** argv)
 : m_rate(NODE_RATE), m_privateNh("~"),
   m_baseFrameId(base_frame)
{
    parse_command_line(argc,argv);
    if (!connectNaoQi())
    {
      ROS_ERROR("Gosh! Failed to connect to NaoQI! Throwing an exception!");
      throw std::exception();
    }

    m_monitor.init(m_broker);

    // get base_frame_id (and fix prefix if necessary)
    m_privateNh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
    // Resolve TF frames using ~tf_prefix parameter
    m_baseFrameId = m_listener.resolve(m_baseFrameId);
    m_cameraFrameId = m_listener.resolve(camera_frame);

    ROS_INFO("nao_human initialized. Base frame is %s", m_baseFrameId.c_str());

}
HumanTracker::~HumanTracker()
{
}
void HumanTracker::run()
{

    ros::Rate r(m_rate);
    ros::Time stamp1;
    ros::Time stamp2;
    ros::Time stamp;

    map<string, Human> humans;

    tf::StampedTransform pose; // human pose in the camera frame

    pose.frame_id_ = m_cameraFrameId;

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    pose.setRotation(q);


    ROS_INFO("Starting main loop of nao_human_tracker");
    while(ros::ok() )
    {
        r.sleep();
        ros::spinOnce();

        stamp1 = ros::Time::now();
        m_monitor.getHumans(humans);
        stamp2 = ros::Time::now();
        stamp = stamp1 + ros::Duration((stamp2-stamp1).toSec()/2.0);

        for (std::map<string, Human>::iterator it = humans.begin() ; it != humans.end(); ++it)
        {
            Human h = it->second;
            if (difftime(time(NULL), h.lastseen) > KEEP_HUMAN_ALIVE) {
                    ROS_DEBUG("Human %s not seen since a while. Discarding it.", h.id.c_str());
                    continue;
            }

            ROS_INFO("Publishing human %s at (%.2f, %.2f, %.2f)", h.id.c_str(), h.x, h.y, h.z);
            pose.setOrigin(tf::Vector3(-h.y, -h.z, h.x));
            pose.stamp_ = stamp;
            pose.child_frame_id_ = "human_" + h.id;

            try{
                if (m_baseFrameId != m_cameraFrameId) {
                    m_listener.waitForTransform(m_cameraFrameId, m_baseFrameId, 
                                                ros::Time(0), 
                                                ros::Duration(1));

                    tf::StampedTransform camera2world;
                    m_listener.lookupTransform(m_baseFrameId, m_cameraFrameId, 
                                               ros::Time(0), 
                                               camera2world);
                    pose *= camera2world;
                }
                m_transformBroadcaster.sendTransform(pose);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("Not publishing human %s: %s", h.id.c_str(), ex.what());
            }
        }
    }
    ROS_INFO("nao_human_tracker stopped.");

}

int main(int argc, char ** argv)
{
   ros::init(argc, argv, "nao_human_tracker");
   ROS_INFO("Starting Nao's human tracker");

   HumanTracker* tracker;
   try{
      tracker = new HumanTracker(argc,argv);
   }
   catch (const std::exception & e)
   {
      ROS_ERROR("Creating HumanTracker object failed with error ");
      return -1;
   }
   tracker->run();
   delete tracker;

   ROS_INFO("End of the road for nao_human_tracker");
   return 0;
}
