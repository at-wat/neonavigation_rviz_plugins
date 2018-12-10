/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * Copyright (c) 2014-2018, the neonavigation authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#ifndef TRAJECTORY_TRACKER_RVIZ_PLUGINS_VALIDATE_FLOATS_H
#define TRAJECTORY_TRACKER_RVIZ_PLUGINS_VALIDATE_FLOATS_H

#include <vector>

#include <rviz/validate_floats.h>
#ifdef HAVE_VALIDATE_QUATERNION_H
#include <rviz/validate_quaternions.h>
#endif

#include <trajectory_tracker_msgs/PathWithVelocity.h>
#include <trajectory_tracker_msgs/PoseStampedWithVelocity.h>

namespace trajectory_tracker_rviz_plugins
{
inline bool validateFloats(const trajectory_tracker_msgs::PoseStampedWithVelocity& msg)
{
  bool valid = true;
  valid = valid && rviz::validateFloats(msg.pose.position);
  valid = valid && rviz::validateFloats(msg.pose.orientation);
  valid = valid && rviz::validateFloats(msg.linear_velocity);
  return valid;
}

template <typename T>
inline bool validateFloats(const std::vector<T>& vec)
{
  typedef std::vector<T> VecType;
  typename VecType::const_iterator it = vec.begin();
  typename VecType::const_iterator end = vec.end();
  for (; it != end; ++it)
  {
    if (!validateFloats(*it))
    {
      return false;
    }
  }

  return true;
}

template <typename T, size_t N>
inline bool validateFloats(const boost::array<T, N>& arr)
{
  typedef boost::array<T, N> ArrType;
  typename ArrType::const_iterator it = arr.begin();
  typename ArrType::const_iterator end = arr.end();
  for (; it != end; ++it)
  {
    if (!validateFloats(*it))
    {
      return false;
    }
  }

  return true;
}

inline bool validateFloats(const trajectory_tracker_msgs::PathWithVelocity& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.poses);
  return valid;
}

#ifdef HAVE_VALIDATE_QUATERNION_H
inline bool validateQuaternions(const trajectory_tracker_msgs::PoseStampedWithVelocity& msg)
{
  return rviz::validateQuaternions(msg.pose.orientation);
}

template <typename T>
inline bool validateQuaternions(const std::vector<T>& vec)
{
  typedef std::vector<T> VecType;
  typename VecType::const_iterator it = vec.begin();
  typename VecType::const_iterator end = vec.end();
  for (; it != end; ++it)
  {
    if (!validateQuaternions(*it))
    {
      return false;
    }
  }

  return true;
}

template <typename T, size_t N>
inline bool validateQuaternions(const boost::array<T, N>& arr)
{
  typedef boost::array<T, N> ArrType;
  typename ArrType::const_iterator it = arr.begin();
  typename ArrType::const_iterator end = arr.end();
  for (; it != end; ++it)
  {
    if (!validateQuaternions(*it))
    {
      return false;
    }
  }

  return true;
}
#endif

}  // namespace trajectory_tracker_rviz_plugins

#endif  // TRAJECTORY_TRACKER_RVIZ_PLUGINS_VALIDATE_FLOATS_H
