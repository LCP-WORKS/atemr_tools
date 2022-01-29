#ifndef REACTIVEBASE_H
#define REACTIVEBASE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <atemr_msgs/SpeedRampStatus.h>
#include <mutex>
#include <deque>
#include <boost/bind.hpp>
#include <boost/atomic.hpp>
#include "../fuzzylite/fl/Headers.h"
#include <atemr_dbus/atemr_exception.h>
#include <chrono>

namespace rb
{
  class ReactiveBase
  {
  public:
    ReactiveBase(const ros::NodeHandle &, const ros::NodeHandle &);

    void run();
    void patternCB(std_msgs::Float32MultiArray::ConstPtr );
    void opInputCB(geometry_msgs::Twist::ConstPtr);
    void speedRampCB(atemr_msgs::SpeedRampStatus::ConstPtr);

  private:
    ros::NodeHandle p_nh_, nh_;
    ros::Subscriber pattern_sub_, operator_vel_sub_, sramp_sub_;
    ros::Publisher vel_pub_;
    geometry_msgs::Twist op_vel_msg_, reactive_vel_msg_;
    std::string idle_engine_path_, manual_engine_path_;
    boost::atomic<bool> is_idle_{true}, is_emergency_{false}, is_aware_{false}, is_idle_timer_{false};
    boost::mutex ptn_mtx_, vel_mtx_;
    double react_percent_, obstacle_stop_dist_, max_x_vel_, idle_transition_time_;
    std_msgs::Float32MultiArray pattern_;
    std::deque<std_msgs::Float32MultiArray> pattern_list_;
    std::unique_ptr<fl::Engine> idle_engine_, manual_engine_;
    std::vector<fl::InputVariable*> manual_variable_, idle_variable_;
    struct EngineOutput
    {
      fl::OutputVariable *meng_steerfwd, *meng_steerrev;
      fl::OutputVariable *lin_steer, *ang_steer;
    } engx_out_;
    std::chrono::time_point<std::chrono::system_clock> idle_since_;
  };
}

#endif // REACTIVEBASE_H
