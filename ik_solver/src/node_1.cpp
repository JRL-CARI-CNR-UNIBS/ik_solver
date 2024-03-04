/*
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <array>
#include <string>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <ik_solver_core/ik_solver_base_class.h>
#include <boost/shared_ptr.hpp>

#include <ik_solver/internal/services_1.h>

// For boost -> std shared_ptr conversion
namespace {
    template<class SharedPointer> struct Holder {
        SharedPointer p;

        Holder(const SharedPointer &p) : p(p) {}
        Holder(const Holder &other) : p(other.p) {}
        Holder(Holder &&other) : p(std::move(other.p)) {}

        void operator () (...) { p.reset(); }
    };
}

template<class T> std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T> &p) {
    typedef Holder<std::shared_ptr<T>> H;
    if(H *h = boost::get_deleter<H>(p)) {
        return h->p;
    } else {
        return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
    }
}

template<class T> boost::shared_ptr<T> to_boost_ptr(const std::shared_ptr<T> &p){
    typedef Holder<boost::shared_ptr<T>> H;
    if(H * h = std::get_deleter<H>(p)) {
        return h->p;
    } else {
        return boost::shared_ptr<T>(p.get(), Holder<std::shared_ptr<T>>(p));
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");

  ros::NodeHandle nh("~");
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader("ik_solver", "ik_solver::IkSolver");

  std::string plugin_name;
  std::string what;
  if(!cnr::param::get(std::string(node->get_namespace()) + "/type", plugin_name, what))
  {
    RCLCPP_ERROR(node->get_logger(), "%s/type is not defined",node->get_namespace());
    RCLCPP_DEBUG_STREAM(node->get_logger(), what);
    return -1;
  }

  std::string rd;
  if(!nh.getParam("robot_description", rd))
  {
    ROS_ERROR("Cannot retrieve robot description");
  }

  std::string what;
  cnr::param::set("/robot_description", rd, what);
  ROS_DEBUG("what: %s", what.c_str());

  ik_solver::IkSolversPool  ik_solvers;
  ROS_DEBUG("Creating %s (type %s)",nh.getNamespace().c_str(),plugin_name.c_str());
  for(std::size_t i=0;i<ik_solver::MAX_NUM_PARALLEL_IK_SOLVER;i++ )
  {
    boost::shared_ptr<ik_solver::IkSolver> ptr = ik_loader.createInstance(plugin_name);
    ik_solvers.at(i) = to_std_ptr<ik_solver::IkSolver>(ptr);
    ROS_DEBUG("Configuring %s (type %s)",nh.getNamespace().c_str(),plugin_name.c_str());
    if (!ik_solvers.at(i)->config())
    {
      ROS_ERROR("unable to configure %s (type %s)",nh.getNamespace().c_str(),plugin_name.c_str());
      return 0;
    }
  }
  ROS_DEBUG("%s (type %s) is ready to compute IK",nh.getNamespace().c_str(),plugin_name.c_str());
  // ==============================================

  ik_solver::IkServices services(nh, ik_solvers);

  // ==============================================

  ros::spin();
  return 0;
}
