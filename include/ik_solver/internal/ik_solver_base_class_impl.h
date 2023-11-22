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

#ifndef IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H
#define IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H

#include <exception>
#include <ostream>
#include <sstream>
#include <cmath>
#include <functional>
#include <iostream>
#include <numeric>
#include <valarray>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string>
#include <chrono>
#include <mutex>
#include <tuple>
#include <vector>
#include <regex>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <cstdio>
#include "XmlRpcValue.h"
#include "ik_solver/internal/types.h"

#include <geometry_msgs/Pose.h>
#include <ik_solver_msgs/GetIkArray.h>
#include <ik_solver_msgs/GetBound.h>
#include <ik_solver/internal/utils.h>

#include <ik_solver/ik_solver_base_class.h>


using namespace std::chrono_literals;

namespace ik_solver
{

inline bool IkSolver::config(const ros::NodeHandle& nh, const std::string& params_ns)
{
  params_ns_ = ik_solver::resolve_ns(nh, params_ns);
  robot_nh_ = nh;

  std::map<std::string, std::string*> sparams{
    { params_ns_ + "base_frame", &base_frame_ },
    { params_ns_ + "flange_frame", &flange_frame_ },
    { params_ns_ + "tool_frame", &tool_frame_ },
  };

  if (!get_and_return(sparams))
  {
    return false;
  }

  std::map<std::string, std::pair<int*, int>> iparams{
    { params_ns_ + "desired_solutions", { &desired_solutions_, desired_solutions_ } },
    { params_ns_ + "min_stall_iterations", { &min_stall_iter_, min_stall_iter_ } },
    { params_ns_ + "max_stall_iterations", { &max_stall_iter_, max_stall_iter_ } },
    { params_ns_ + "parallel_ik_mode", { &parallelize_, ik_solver_msgs::GetIkArray::Request::PARALLELIZE_DISABLE } },
  };

  if (!get_and_default(iparams))
  {
    return false;
  }

  if (!getFlangeTool())
  {
    ROS_ERROR("%s: no TF from flange and tool", params_ns_.c_str());
    return false;
  }

  model_.initParam("robot_description");

  auto pn = params_ns_ + "joint_names";
  if (!ros::param::get(pn, joint_names_))
  {
    ROS_ERROR("[IkSolver::config] %s is not specified", pn.c_str());
    return false;
  }

  jb_.resize(joint_names_.size());
  revolute_.resize(joint_names_.size());
  std::map<std::string, urdf::JointSharedPtr> joint_models = model_.joints_;
  for (size_t iax = 0; iax < joint_names_.size(); iax++)
  {
    if (joint_models.count(joint_names_.at(iax)) == 0)
    {
      ROS_ERROR("%s: %s is not a valid joint name", params_ns_.c_str(), joint_names_.at(iax).c_str());
      return false;
    }
    const urdf::JointSharedPtr& jmodel = joint_models.at(joint_names_.at(iax));
    ik_solver::Range urdf_range;
    urdf_range.min() = jmodel->limits->lower;
    urdf_range.max() = jmodel->limits->upper;
    jb_.at(iax).first = joint_names_.at(iax);
    jb_.at(iax).second.push_back(urdf_range);

    revolute_.at(iax) = jmodel->type == jmodel->REVOLUTE;

    if(ros::param::has(params_ns_ + "limits/" + joint_names_.at(iax)))
    {
      XmlRpc::XmlRpcValue list_of_limits;
      if(!ros::param::get(params_ns_ + "limits/" + joint_names_.at(iax), list_of_limits))
      {
        ROS_ERROR("Wierd the param %s exists but is invalid", (params_ns_ + "limits/" + joint_names_.at(iax)).c_str());
        return false;
      }

      auto to_str = [](const XmlRpc::XmlRpcValue& p)-> std::string
      {
        std::ostream stream(nullptr); // useless ostream (badbit set)
        std::stringbuf str;
        stream.rdbuf(&str); // uses str
        p.write(stream);
        return str.str();
      };

      auto to_double = [&to_str](const XmlRpc::XmlRpcValue& p, double& val, std::string& what) -> bool
      {
        if(p.getType()==XmlRpc::XmlRpcValue::TypeDouble)
        {
          val = double(p);
        }
        else if(p.getType()==XmlRpc::XmlRpcValue::TypeInt)
        {
          val = double(int(p));
        }
        else
        {
          what = "The value is neither a double nor an int: " + to_str(p);
          return false;
        }
        return true;
      };

      auto get_range = [&to_double, &to_str] (const XmlRpc::XmlRpcValue& p, const ik_solver::Range& urdf_range, ik_solver::Range& range, std::string& what) -> bool
      {
        size_t ll = __LINE__;
        try
        {
          ll = __LINE__;
          if(!p.hasMember("upper") && !p.hasMember("lower"))
          {
            std::ostream stream(nullptr); // useless ostream (badbit set)
            std::stringbuf str;
            stream.rdbuf(&str); // uses str
            p.write(stream);
            what = "The entry is ill-formed, there is neither 'upper' nor lower' key (Value: '" + str.str() + "')";
            return false;
          }
          ll = __LINE__;
          
          if(p.hasMember("lower"))
          {
            ll = __LINE__;
            if(!to_double(p["lower"], range.min(), what))
            {
              return false;
            }
          }
          else
          {
            ll = __LINE__;
            range.min() = urdf_range.min();
          }
          if(p.hasMember("upper"))
          {
            ll = __LINE__;
            if(!to_double(p["upper"], range.max(), what))
            {
              return false;
            }
          }
          else
          {
            ll = __LINE__;
            range.max() = urdf_range.max();
          }

          if (range.min() > range.max())
          {
            ll = __LINE__;
            what = "The entry is ill-formed, the 'upper' value is less than the 'lower' value";
            return false;
          }
        }
        catch(std::exception& e)
        {
          what = ("Last Line executed: " + std::to_string(ll) + ": ") + e.what();
          return false;
        }

        return true;
      };

      std::string what;
      if(list_of_limits.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        Range r;
        if(!get_range(list_of_limits, urdf_range, r,what))
        {
          ROS_ERROR("%s", what.c_str());
          return false;
        }
        jb_.at(iax).first = joint_names_.at(iax);
        jb_.at(iax).second.push_back(r);
      }
      else if(list_of_limits.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for(auto it = list_of_limits.begin(); it != list_of_limits.end(); ++it)
        {
          Range r;
          if(!get_range(it->second, urdf_range, r,what))
          {
            ROS_ERROR("%s", what.c_str());
            return false;
          }
          jb_.at(iax).first = joint_names_.at(iax);
          jb_.at(iax).second.push_back(r);
        }
      }
      else
      {
        assert(0);
      }
    }
  }
  ROS_WARN_STREAM("jb:" << jb_ );
  return true;
}

inline bool IkSolver::getFlangeTool()
{
  return getTF(tool_frame_, flange_frame_, T_tool_flange_);
}

// Eigen::Affine3d IkSolver::getFK(const Configuration& s)
// {
//   Eigen::Affine3d I;
//   I.setIdentity();
//   return I;
// }

}  // end namespace ik_solver

#endif  // IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H
