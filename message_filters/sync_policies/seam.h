/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef MESSAGE_FILTERS__SYNC_SEAM_H_
#define MESSAGE_FILTERS__SYNC_SEAM_H_

#include <cassert>
#include <deque>
#include <string>
#include <tuple>
#include <vector>

#include <inttypes.h>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include "message_filters/connection.h"
#include "message_filters/message_traits.h"
#include "message_filters/null_types.h"
#include "message_filters/signal9.h"
#include "message_filters/synchronizer.h"


#ifndef RCUTILS_ASSERT
// TODO(tfoote) remove this after it's implemented upstream
// https://github.com/ros2/rcutils/pull/112
#define RCUTILS_ASSERT assert
#endif
#ifndef RCUTILS_BREAK
#include <cassert>
// TODO(tfoote) remove this after it's implemented upstream
// https://github.com/ros2/rcutils/pull/112
#define RCUTILS_BREAK std::abort
#endif
// Uncomment below intead
//#include <rcutils/assert.h>

namespace message_filters
{
namespace sync_policies
{

template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct SEAM : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<SEAM> Sync;
  typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;
  typedef typename Super::M0Event M0Event;
  typedef typename Super::M1Event M1Event;
  typedef typename Super::M2Event M2Event;
  typedef typename Super::M3Event M3Event;
  typedef typename Super::M4Event M4Event;
  typedef typename Super::M5Event M5Event;
  typedef typename Super::M6Event M6Event;
  typedef typename Super::M7Event M7Event;
  typedef typename Super::M8Event M8Event;
  typedef std::deque<M0Event> M0Deque;
  typedef std::deque<M1Event> M1Deque;
  typedef std::deque<M2Event> M2Deque;
  typedef std::deque<M3Event> M3Deque;
  typedef std::deque<M4Event> M4Deque;
  typedef std::deque<M5Event> M5Deque;
  typedef std::deque<M6Event> M6Deque;
  typedef std::deque<M7Event> M7Deque;
  typedef std::deque<M8Event> M8Deque;
  typedef std::vector<M0Event> M0Vector;
  typedef std::vector<M1Event> M1Vector;
  typedef std::vector<M2Event> M2Vector;
  typedef std::vector<M3Event> M3Vector;
  typedef std::vector<M4Event> M4Vector;
  typedef std::vector<M5Event> M5Vector;
  typedef std::vector<M6Event> M6Vector;
  typedef std::vector<M7Event> M7Vector;
  typedef std::vector<M8Event> M8Vector;
  typedef Events Tuple;
  typedef std::tuple<M0Deque, M1Deque, M2Deque, M3Deque, M4Deque, M5Deque, M6Deque, M7Deque, M8Deque> DequeTuple;
  typedef std::tuple<M0Vector, M1Vector, M2Vector, M3Vector, M4Vector, M5Vector, M6Vector, M7Vector, M8Vector> VectorTuple;

  SEAM()
  : parent_(0)
  // , queue_size_(queue_size)
  , num_non_empty_deques_(0)
  , pivot_(NO_PIVOT)
  , max_interval_duration_(rclcpp::Duration(std::numeric_limits<int32_t>::max(),999999999))
  , age_penalty_(0.1)
  , has_dropped_messages_(9, false)
  , inter_message_lower_bounds_(9, rclcpp::Duration(0, 0))
  , warned_about_incorrect_bound_(9, false)
  {
    // RCUTILS_ASSERT(queue_size_ > 0);  // The synchronizer will tend to drop many messages with a queue size of 1. At least 2 is recommended.
  }

  // SEAM(uint32_t queue_size)
  // : parent_(0)
  // , queue_size_(queue_size)
  // , num_non_empty_deques_(0)
  // , pivot_(NO_PIVOT)
  // , max_interval_duration_(rclcpp::Duration(std::numeric_limits<int32_t>::max(),999999999))
  // , age_penalty_(0.1)
  // , has_dropped_messages_(9, false)
  // , inter_message_lower_bounds_(9, rclcpp::Duration(0, 0))
  // , warned_about_incorrect_bound_(9, false)
  // {
  //   RCUTILS_ASSERT(queue_size_ > 0);  // The synchronizer will tend to drop many messages with a queue size of 1. At least 2 is recommended.
  // }

  SEAM(const SEAM& e)
  : max_interval_duration_(rclcpp::Duration(std::numeric_limits<int32_t>::max(),999999999))
  {
    *this = e;
  }

  SEAM& operator=(const SEAM& rhs)
  {
    parent_ = rhs.parent_;
    queue_size_ = rhs.queue_size_;
    num_non_empty_deques_ = rhs.num_non_empty_deques_;
    pivot_time_ = rhs.pivot_time_;
    pivot_ = rhs.pivot_;
    max_interval_duration_ = rhs.max_interval_duration_;
    age_penalty_ = rhs.age_penalty_;
    candidate_start_ = rhs.candidate_start_;
    candidate_end_ = rhs.candidate_end_;
    deques_ = rhs.deques_;
    past_ = rhs.past_;
    has_dropped_messages_ = rhs.has_dropped_messages_;
    inter_message_lower_bounds_ = rhs.inter_message_lower_bounds_;
    warned_about_incorrect_bound_ = rhs.warned_about_incorrect_bound_;
    x_ms_ = rhs.x_ms_;

    return *this;
  }

  void initParent(Sync* parent)
  {
    parent_ = parent;
  }

  template<int i>
  void checkInterMessageBound()
  {
    namespace mt = message_filters::message_traits;
    if (warned_about_incorrect_bound_[i])
    {
      return;
    }
    std::deque<typename std::tuple_element<i, Events>::type>& deque = std::get<i>(deques_);
    std::vector<typename std::tuple_element<i, Events>::type>& v = std::get<i>(past_);
    RCUTILS_ASSERT(!deque.empty());
    const typename std::tuple_element<i, Messages>::type &msg = *(deque.back()).getMessage();
    rclcpp::Time msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(msg);
    rclcpp::Time previous_msg_time;
    if (deque.size() == (size_t) 1)
    {
      if (v.empty())
      {
	// We have already published (or have never received) the previous message, we cannot check the bound
	return;
      }
      const typename std::tuple_element<i, Messages>::type &previous_msg = *(v.back()).getMessage();
      previous_msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(previous_msg);
    }
    else
    {
      // There are at least 2 elements in the deque. Check that the gap respects the bound if it was provided.
      const typename std::tuple_element<i, Messages>::type &previous_msg = *(deque[deque.size()-2]).getMessage();
      previous_msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(previous_msg);
    }
    if (msg_time < previous_msg_time)
    {
      RCUTILS_LOG_WARN_ONCE("Messages of type %d arrived out of order (will print only once)", i);
      warned_about_incorrect_bound_[i] = true;
    }
    else if ((msg_time - previous_msg_time) < inter_message_lower_bounds_[i])
    {
      RCUTILS_LOG_WARN_ONCE("Messages of type %d arrived closer ("
        "%" PRId64 ") than the lower bound you provided ("
        "%" PRId64 ") (will print only once)",
        i,
        (msg_time - previous_msg_time).nanoseconds(),
        inter_message_lower_bounds_[i].nanoseconds());
      warned_about_incorrect_bound_[i] = true;
    }
  }

  template<int i>
  void add(const typename std::tuple_element<i, Events>::type& evt)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::deque<typename std::tuple_element<i, Events>::type>& deque = std::get<i>(deques_);
    deque.push_back(evt);
    if (deque.size() == (size_t)1) 
    {
      // We have just added the first message, so it was empty before
      ++num_non_empty_deques_;
      if (num_non_empty_deques_ == (uint32_t)RealTypeCount::value)
      {
        // All deques have messages
        // Case 1 : All the deque-front time is in the bound range
        // Case 2 : Not all the deque-front time is in the bound range
        findEarliestMsg<i>();
      }

      if (num_non_empty_deques_ == (uint32_t)RealTypeCount::value)
      {
        // the front msg of all deques can be a candidate now
        synCandidate();
        publishCandidate();
      }
    }
    else
    {
      checkInterMessageBound<i>();
    }
  }

  template<int i>
  void findEarliestMsg()
  {
    namespace mt = message_filters::message_traits;

    std::deque<typename std::tuple_element<i, Events>::type>& deque = std::get<i>(deques_);
    const typename std::tuple_element<i, Messages>::type &msg = *(deque.back()).getMessage();
    rclcpp::Time last_msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(msg);    
    // rclcpp::Time previous_msg_time;

    // 2 topics
    if (i != 0)
    {
      M0Deque& q_0 = std::get<0>(deques_);
      if (!q_0.empty())
      {
        const typename std::tuple_element<0, Messages>::type &previous_msg = *(q_0.front()).getMessage();
        rclcpp::Time previous_msg_time = mt::TimeStamp<typename std::tuple_element<0, Messages>::type>::value(previous_msg);
        if (last_msg_time > previous_msg_time)
        {
          while (previous_msg_time < (last_msg_time - BOUND))
          {
            q_0.pop_front();
            if (q_0.empty())
            {
              --num_non_empty_deques_;
              break;
            }
            const typename std::tuple_element<0, Messages>::type &pre_msg = *(q_0.front()).getMessage();
            previous_msg_time = mt::TimeStamp<typename std::tuple_element<0, Messages>::type>::value(pre_msg);
          }
        }
        else
        {
          RCUTILS_LOG_WARN_ONCE("the order of messages in differen topics is wrong in topic 0 and %d", i);
        }
      }
    }
    if (i != 1)
    {
      M1Deque& q_1 = std::get<1>(deques_);
      if (!q_1.empty())
      {
        const typename std::tuple_element<1, Messages>::type &previous_msg = *(q_1.front()).getMessage();
        rclcpp::Time previous_msg_time = mt::TimeStamp<typename std::tuple_element<1, Messages>::type>::value(previous_msg);
        if (last_msg_time > previous_msg_time)
        {
          while (previous_msg_time < (last_msg_time - BOUND))
          {
            q_1.pop_front();
            if (q_1.empty())
            {
              --num_non_empty_deques_;
              break;
            }
            const typename std::tuple_element<1, Messages>::type &pre_msg = *(q_1.front()).getMessage();
            previous_msg_time = mt::TimeStamp<typename std::tuple_element<1, Messages>::type>::value(pre_msg);
          }
        }
        else
        {
          RCUTILS_LOG_WARN_ONCE("the order of messages in differen topics is wrong in topic 1 and %d", i);
        }
      }   
    }
    
    if (RealTypeCount::value > 2)
    {
      // 3 topics
      if (i != 2)
      {
        M2Deque& q_2 = std::get<2>(deques_);
        if (!q_2.empty())
        {
          const typename std::tuple_element<2, Messages>::type &previous_msg = *(q_2.front()).getMessage();
          rclcpp::Time previous_msg_time = mt::TimeStamp<typename std::tuple_element<2, Messages>::type>::value(previous_msg);
          if (last_msg_time > previous_msg_time)
          {
            while (previous_msg_time < (last_msg_time - BOUND))
            {
              q_2.pop_front();
              if (q_2.empty())
              {
                --num_non_empty_deques_;
                break;
              }
              const typename std::tuple_element<2, Messages>::type &pre_msg = *(q_2.front()).getMessage();
              previous_msg_time = mt::TimeStamp<typename std::tuple_element<2, Messages>::type>::value(pre_msg);
            }
          }
          else
          {
            RCUTILS_LOG_WARN_ONCE("the order of messages in differen topics is wrong in topic 2 and %d", i);
          }
        }
      }
      
      if (RealTypeCount::value > 3)
      {
        // 4 topics
        if (i != 3)
        {
          M3Deque& q_3 = std::get<3>(deques_);
          if (!q_3.empty())
          {
            const typename std::tuple_element<3, Messages>::type &previous_msg = *(q_3.front()).getMessage();
            rclcpp::Time previous_msg_time = mt::TimeStamp<typename std::tuple_element<3, Messages>::type>::value(previous_msg);
            if (last_msg_time > previous_msg_time)
            {
              while (previous_msg_time < (last_msg_time - BOUND))
              {
                q_3.pop_front();
                if (q_3.empty())
                {
                  --num_non_empty_deques_;
                  break;
                }
                const typename std::tuple_element<3, Messages>::type &pre_msg = *(q_3.front()).getMessage();
                previous_msg_time = mt::TimeStamp<typename std::tuple_element<3, Messages>::type>::value(pre_msg);
              }
            }
            else
            {
              RCUTILS_LOG_WARN_ONCE("the order of messages in differen topics is wrong in topic 3 and %d", i);
            }
          }
        }
        
        if (RealTypeCount::value > 4)
        {
          // 5 topics
          if (i != 4)
          {
            M4Deque& q_4 = std::get<4>(deques_);
            if (!q_4.empty())
            {
              const typename std::tuple_element<4, Messages>::type &previous_msg = *(q_4.front()).getMessage();
              rclcpp::Time previous_msg_time = mt::TimeStamp<typename std::tuple_element<4, Messages>::type>::value(previous_msg);
              if (last_msg_time > previous_msg_time)
              {
                while (previous_msg_time < (last_msg_time - BOUND))
                {
                  q_4.pop_front();
                  if (q_4.empty())
                  {
                    --num_non_empty_deques_;
                    break;
                  }
                  const typename std::tuple_element<4, Messages>::type &pre_msg = *(q_4.front()).getMessage();
                  previous_msg_time = mt::TimeStamp<typename std::tuple_element<4, Messages>::type>::value(pre_msg);
                }
              }
              else
              {
                RCUTILS_LOG_WARN_ONCE("the order of messages in differen topics is wrong in topic 4 and %d", i);
              }
            }
          }
          
          if (RealTypeCount::value > 5)
          {
            // 6 topics
            if (i != 5)
            {
              M5Deque& q_5 = std::get<5>(deques_);
              if (!q_5.empty())
              {
                const typename std::tuple_element<5, Messages>::type &previous_msg = *(q_5.front()).getMessage();
                rclcpp::Time previous_msg_time = mt::TimeStamp<typename std::tuple_element<5, Messages>::type>::value(previous_msg);
                if (last_msg_time > previous_msg_time)
                {
                  while (previous_msg_time < (last_msg_time - BOUND))
                  {
                    q_5.pop_front();
                    if (q_5.empty())
                    {
                      --num_non_empty_deques_;
                      break;
                    }
                    const typename std::tuple_element<5, Messages>::type &pre_msg = *(q_5.front()).getMessage();
                    previous_msg_time = mt::TimeStamp<typename std::tuple_element<5, Messages>::type>::value(pre_msg);
                  }
                }
                else
                {
                  RCUTILS_LOG_WARN_ONCE("the order of messages in differen topics is wrong in topic 5 and %d", i);
                }
              }
            }
            
            if (RealTypeCount::value > 6)
            {
              // 7 topics
              if (i != 6)
              {
                M6Deque& q_6 = std::get<6>(deques_);
                if (!q_6.empty())
                {
                  const typename std::tuple_element<6, Messages>::type &previous_msg = *(q_6.front()).getMessage();
                  rclcpp::Time previous_msg_time = mt::TimeStamp<typename std::tuple_element<6, Messages>::type>::value(previous_msg);
                  if (last_msg_time > previous_msg_time)
                  {
                    while (previous_msg_time < (last_msg_time - BOUND))
                    {
                      q_6.pop_front();
                      if (q_6.empty())
                      {
                        --num_non_empty_deques_;
                        break;
                      }
                      const typename std::tuple_element<6, Messages>::type &pre_msg = *(q_6.front()).getMessage();
                      previous_msg_time = mt::TimeStamp<typename std::tuple_element<6, Messages>::type>::value(pre_msg);
                    }
                  }
                else
                {
                  RCUTILS_LOG_WARN_ONCE("the order of messages in differen topics is wrong in topic 6 and %d", i);
                }
                }
              }
              
              if (RealTypeCount::value > 7)
              {
                // 8 topics
                if (i != 7)
                {
                  M7Deque& q_7 = std::get<7>(deques_);
                  if (!q_7.empty())
                  {
                    const typename std::tuple_element<7, Messages>::type &previous_msg = *(q_7.front()).getMessage();
                    rclcpp::Time previous_msg_time = mt::TimeStamp<typename std::tuple_element<7, Messages>::type>::value(previous_msg);
                    if (last_msg_time > previous_msg_time)
                    {
                      while (previous_msg_time < (last_msg_time - BOUND))
                      {
                        q_7.pop_front();
                        if (q_7.empty())
                        {
                          --num_non_empty_deques_;
                          break;
                        }
                        const typename std::tuple_element<7, Messages>::type &pre_msg = *(q_7.front()).getMessage();
                        previous_msg_time = mt::TimeStamp<typename std::tuple_element<7, Messages>::type>::value(pre_msg);
                      }
                    }
                    else
                    {
                      RCUTILS_LOG_WARN_ONCE("the order of messages in differen topics is wrong in topic 7 and %d", i);
                    }
                  }
                }
                
                if (RealTypeCount::value > 8)
                {
                  // 9 topics
                  if (i != 8)
                  {
                    M8Deque& q_8 = std::get<8>(deques_);
                    if (!q_8.empty())
                    {
                      const typename std::tuple_element<8, Messages>::type &previous_msg = *(q_8.front()).getMessage();
                      rclcpp::Time previous_msg_time = mt::TimeStamp<typename std::tuple_element<8, Messages>::type>::value(previous_msg);
                      if (last_msg_time > previous_msg_time)
                      {
                        while (previous_msg_time < (last_msg_time - BOUND))
                        {
                          q_8.pop_front();
                          if (q_8.empty())
                          {
                            --num_non_empty_deques_;
                            break;
                          }
                          const typename std::tuple_element<8, Messages>::type &pre_msg = *(q_8.front()).getMessage();
                          previous_msg_time = mt::TimeStamp<typename std::tuple_element<8, Messages>::type>::value(pre_msg);
                        }
                      }
                      else
                      {
                        RCUTILS_LOG_WARN_ONCE("the order of messages in differen topics is wrong in topic 8 and %d", i);
                      }
                    }
                  }
                  
                }
              }
            }
          }
        }
      }
    }

    // if (num_non_empty_deques_ == RealTypeCount::value)
    // {
    //   // the front msg of all deques can be a candidate now
    // }
    // else
    // {
    //   // there are still empty queues, need to return
    // }    
  }

  void setInterMessageLowerBound(int i, rclcpp::Duration lower_bound) {
    // For correctness we only need age_penalty > -1.0, but most likely a negative age_penalty is a mistake.
    RCUTILS_ASSERT(lower_bound >= rclcpp::Duration(0,0));
    inter_message_lower_bounds_[i] = lower_bound;
  }
  void setBOUND(int x_ms)
  {
    int64_t nanosec = x_ms * 1000000;
    rclcpp::Duration duration(nanosec);
    BOUND = duration;
  }

private:

  void synCandidate()
  {
    candidate_ = Tuple(); // Discards old one if any
    std::get<0>(candidate_) = std::get<0>(deques_).front();
    std::get<1>(candidate_) = std::get<1>(deques_).front();
    if (RealTypeCount::value > 2)
    {
      std::get<2>(candidate_) = std::get<2>(deques_).front();
      if (RealTypeCount::value > 3)
      {
        std::get<3>(candidate_) = std::get<3>(deques_).front();
	      if (RealTypeCount::value > 4)
	      {
          std::get<4>(candidate_) = std::get<4>(deques_).front();
          if (RealTypeCount::value > 5)
          {
            std::get<5>(candidate_) = std::get<5>(deques_).front();
            if (RealTypeCount::value > 6)
            {
              std::get<6>(candidate_) = std::get<6>(deques_).front();
              if (RealTypeCount::value > 7)
              {
                std::get<7>(candidate_) = std::get<7>(deques_).front();
                if (RealTypeCount::value > 8)
                {
                  std::get<8>(candidate_) = std::get<8>(deques_).front();
                }
	            }
	          }
	        }
	      }
      }
    }
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == RealTypeCount::value
  void publishCandidate()
  {
    //printf("Publishing candidate\n");
    // Publish
    parent_->signal(std::get<0>(candidate_), std::get<1>(candidate_), std::get<2>(candidate_), std::get<3>(candidate_),
                    std::get<4>(candidate_), std::get<5>(candidate_), std::get<6>(candidate_), std::get<7>(candidate_),
                    std::get<8>(candidate_));
    // Delete this candidate
    candidate_ = Tuple();
    // grd_succeed++;

    std::get<0>(deques_).pop_front();
    if (std::get<0>(deques_).empty())
    {
      --num_non_empty_deques_;
    }
    std::get<1>(deques_).pop_front();
    if (std::get<1>(deques_).empty())
    {
      --num_non_empty_deques_;
    }
    if (RealTypeCount::value > 2)
    {
      std::get<2>(deques_).pop_front();
      if (std::get<2>(deques_).empty())
      {
        --num_non_empty_deques_;
      }
      if (RealTypeCount::value > 3)
      {
        std::get<3>(deques_).pop_front();
        if (std::get<3>(deques_).empty())
        {
          --num_non_empty_deques_;
        }
        if (RealTypeCount::value > 4)
        {
          std::get<4>(deques_).pop_front();
          if (std::get<4>(deques_).empty())
          {
            --num_non_empty_deques_;
          }
          if (RealTypeCount::value > 5)
          {
            std::get<5>(deques_).pop_front();
            if (std::get<5>(deques_).empty())
            {
              --num_non_empty_deques_;
            }
            if (RealTypeCount::value > 6)
            {
              std::get<6>(deques_).pop_front();
              if (std::get<6>(deques_).empty())
              {
                --num_non_empty_deques_;
              }
              if (RealTypeCount::value > 7)
              {
                std::get<7>(deques_).pop_front();
                if (std::get<7>(deques_).empty())
                {
                  --num_non_empty_deques_;
                }
                if (RealTypeCount::value > 8)
                {
                  std::get<8>(deques_).pop_front();
                  if (std::get<8>(deques_).empty())
                  {
                    --num_non_empty_deques_;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  Sync* parent_;
  uint32_t queue_size_;
  rclcpp::Duration BOUND = rclcpp::Duration(1, 0); //1s
  int x_ms_;

  static const uint32_t NO_PIVOT = 9;  // Special value for the pivot indicating that no pivot has been selected

  DequeTuple deques_;
  uint32_t num_non_empty_deques_;
  VectorTuple past_;
  Tuple candidate_;  // NULL if there is no candidate, in which case there is no pivot.
  rclcpp::Time candidate_start_;
  rclcpp::Time candidate_end_;
  rclcpp::Time pivot_time_;
  uint32_t pivot_;  // Equal to NO_PIVOT if there is no candidate
  std::mutex data_mutex_;  // Protects all of the above

  rclcpp::Duration max_interval_duration_; // TODO: initialize with a parameter
  double age_penalty_;

  std::vector<bool> has_dropped_messages_;
  std::vector<rclcpp::Duration> inter_message_lower_bounds_;
  std::vector<bool> warned_about_incorrect_bound_;
};

}
}

#endif // MESSAGE_FILTERS__SYNC_SEAM_H_