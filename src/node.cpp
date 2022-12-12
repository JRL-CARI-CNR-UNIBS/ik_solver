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


#include <ros/ros.h>
#include <ik_solver/ik_solver_base_class.h>

#include <pluginlib/class_loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh("~");
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader("ik_solver", "ik_solver::IkSolver");

  std::string plugin_name;
  if (!nh.getParam("type",plugin_name))
  {
    ROS_ERROR("%s/type is not defined",nh.getNamespace().c_str());
    return -1;
  }
  ROS_DEBUG("Creating %s (type %s)",nh.getNamespace().c_str(),plugin_name.c_str());
  boost::shared_ptr<ik_solver::IkSolver> ik_solver = ik_loader.createInstance(plugin_name);
  ROS_DEBUG("Configuring %s (type %s)",nh.getNamespace().c_str(),plugin_name.c_str());
  if (!ik_solver->config(nh))
  {
    ROS_ERROR("unable to configure %s (type %s)",nh.getNamespace().c_str(),plugin_name.c_str());
    return 0;
  }
  ROS_DEBUG("%s (type %s) is ready to compute IK",nh.getNamespace().c_str(),plugin_name.c_str());
  ros::spin();
  return 0;
}
