#include <reactive_base/reactivebase.h>

namespace rb
{
  ReactiveBase::ReactiveBase(const ros::NodeHandle &p_nh, const ros::NodeHandle &g_nh): p_nh_(p_nh), nh_(g_nh)
  {
    pattern_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>
        ("/pattern", 1, boost::bind(&ReactiveBase::patternCB, this, _1));
    operator_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>
        ("tmux/cmd_vel", 5, boost::bind(&ReactiveBase::opInputCB, this, _1));
    sramp_sub_ = nh_.subscribe<atemr_msgs::SpeedRampStatus>
        ("/speedramp_status", 1, boost::bind(&ReactiveBase::speedRampCB, this, _1));
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("reactive/cmd_vel", 1);

    p_nh_.param<double>("react_percent", react_percent_, 0.7);
    p_nh_.param<double>("obstacle_stop_distance", obstacle_stop_dist_, 0.25);
    p_nh_.param<double>("max_linear_vel", max_x_vel_, 0.26);
    p_nh_.param<double>("idle_transition_time", idle_transition_time_, 8.0);

    manual_engine_path_ = ros::package::getPath
        ("reactive_base") + "/fuzzylite/logic/manualDriveSteering_90.fll";
    idle_engine_path_ = ros::package::getPath
        ("reactive_base") + "/fuzzylite/logic/reactiveSteering_90.fll";

    manual_engine_.reset(fl::FllImporter().fromFile(manual_engine_path_));
    idle_engine_.reset(fl::FllImporter().fromFile(idle_engine_path_));

    std::string status;
    if(not manual_engine_->isReady(&status))
      ATEMR_EXCP(DBUS_ERROR, "[engine error] MANUAL engine is not ready:\n" + status);
    if(not idle_engine_->isReady(&status))
      ATEMR_EXCP(DBUS_ERROR, "[engine error] IDLE engine is not ready:\n" + status);

    {//!prepare input/ output variables
      manual_variable_.push_back(manual_engine_->getInputVariable("FR"));
      manual_variable_.push_back(manual_engine_->getInputVariable("FL"));
      manual_variable_.push_back(manual_engine_->getInputVariable("LC"));
      manual_variable_.push_back(manual_engine_->getInputVariable("RL"));
      manual_variable_.push_back(manual_engine_->getInputVariable("RR"));
      manual_variable_.push_back(manual_engine_->getInputVariable("RC"));
      engx_out_.meng_steerfwd = manual_engine_->getOutputVariable("angSteerFWD");
      engx_out_.meng_steerrev = manual_engine_->getOutputVariable("angSteerBKWD");

      idle_variable_.push_back(idle_engine_->getInputVariable("FR"));
      idle_variable_.push_back(idle_engine_->getInputVariable("FL"));
      idle_variable_.push_back(idle_engine_->getInputVariable("LC"));
      idle_variable_.push_back(idle_engine_->getInputVariable("RL"));
      idle_variable_.push_back(idle_engine_->getInputVariable("RR"));
      idle_variable_.push_back(idle_engine_->getInputVariable("RC"));
      engx_out_.lin_steer = idle_engine_->getOutputVariable("linSteer");
      engx_out_.ang_steer = idle_engine_->getOutputVariable("angSteer");
    }


  }

  void ReactiveBase::run()
  {
    ros::Rate r(25);
    while (ros::ok() && !ros::isShuttingDown())
    {
      reactive_vel_msg_.linear.x = 0.0;
      reactive_vel_msg_.angular.z = 0.0;

      if(!pattern_list_.empty())
      {
        {
          boost::mutex::scoped_lock ptn_lock(ptn_mtx_, boost::try_to_lock);
          pattern_ = pattern_list_.front();
          pattern_list_.pop_front();
        }

        if(!is_emergency_ && is_aware_)
        {
          //! feed data into both engines
          for(std::size_t i = 0; i < manual_variable_.size(); i++)
          {
            manual_variable_[i]->setValue((double(pattern_.data[i]) <= 0.0) ? 1.0 : double(pattern_.data[i]));
            idle_variable_[i]->setValue((double(pattern_.data[i]) <= 0.0) ? 1.0 : double(pattern_.data[i]));
          }

          if(is_idle_)
          {
            idle_engine_->process();
            reactive_vel_msg_.angular.z = engx_out_.ang_steer->getValue();
            reactive_vel_msg_.linear.x = engx_out_.lin_steer->getValue();
            /*std::cout << "AngSteer: " << reactive_vel_msg_.angular.z
                      << " | LinSteer: " << reactive_vel_msg_.linear.x << std::endl;*/
          }
          else
          {
            manual_engine_->process();
            boost::mutex::scoped_lock vel_lock(vel_mtx_, boost::try_to_lock);
            if(op_vel_msg_.linear.x > 0.0)
            {//! forward motion
              double steer = engx_out_.meng_steerfwd->getValue();
              if(((op_vel_msg_.angular.z > 0.0) && (steer > 0.0)) ||
                 ((op_vel_msg_.angular.z < 0.0) && (steer < 0.0)) ||
                 (abs(steer) <= 0.0))
              {//! if steering in the same direction, use operator input
                reactive_vel_msg_.angular.z = op_vel_msg_.angular.z;
              }
              else
              {//! use inference result with percentage
                reactive_vel_msg_.angular.z = (steer * react_percent_) + (op_vel_msg_.angular.z * (1.0 - react_percent_));
              }

              //! detect head-on collision and react
              //! if within 30cm of entering the emergency field  and no angular vel is being applied
              //! reduce linear velocity to base minimum allowed and turn base based on the ratio of obstacle distances (FL and FR)
              double fl = double(pattern_.data[0]); double fr = double(pattern_.data[1]);
              if((fl > obstacle_stop_dist_ && fl < (obstacle_stop_dist_ + 0.30)) &&
                 (fr > obstacle_stop_dist_ && fr < (obstacle_stop_dist_ + 0.30)) &&
                 (abs(reactive_vel_msg_.angular.z) <= 0.10))
              {
                reactive_vel_msg_.angular.z = (fl > fr) ? 0.35 : -0.35;
                reactive_vel_msg_.linear.x = (op_vel_msg_.linear.x > max_x_vel_) ? max_x_vel_ : op_vel_msg_.linear.x ;
                reactive_vel_msg_.linear.x *= -1.0;
              }
              else
              {//! set max x-vel
                reactive_vel_msg_.linear.x = (op_vel_msg_.linear.x > max_x_vel_) ? max_x_vel_ : op_vel_msg_.linear.x ;
                if(abs(manual_variable_[0]->getValue()) <= 0.30 || abs(manual_variable_[1]->getValue()) <= 0.30)
                  reactive_vel_msg_.linear.x *= -1.0;
              }

              /*std::cout << "\nSteer(fuzzy): " << (steer * react_percent_)
                        << "\nSteer(operator): " << op_vel_msg_.angular.z
                        << "\nSteer(combined): " << reactive_vel_msg_.angular.z
                        << " | LinSteer: " << reactive_vel_msg_.linear.x << std::endl;*/
            }

            if(op_vel_msg_.linear.x < 0.0)
            {//! backward motion
              double steer = engx_out_.meng_steerrev->getValue();
              if(((op_vel_msg_.angular.z > 0.0) && (steer > 0.0)) ||
                 ((op_vel_msg_.angular.z < 0.0) && (steer < 0.0)) ||
                 (abs(steer) <= 0.0))
              {//! if steering in the same direction, use operator input
                reactive_vel_msg_.angular.z = op_vel_msg_.angular.z;
              }
              else
              {//! use inference result with percentage
                reactive_vel_msg_.angular.z = (steer * react_percent_) + (op_vel_msg_.angular.z * (1.0 - react_percent_));
              }

              //! detect head-on collision and react
              //! if within 30cm of entering the emergency field  and no angular vel is being applied
              //! reduce linear velocity to base minimum allowed and turn base based on the ratio of obstacle distances (FL and FR)
              double rl = double(pattern_.data[3]); double rr = double(pattern_.data[4]);
              if((rl > obstacle_stop_dist_ && rl < (obstacle_stop_dist_ + 0.30)) &&
                 (rr > obstacle_stop_dist_ && rr < (obstacle_stop_dist_ + 0.30)) &&
                 (abs(reactive_vel_msg_.angular.z) <= 0.10))
              {
                reactive_vel_msg_.angular.z = (rl > rr) ? -0.35 : 0.35;
                reactive_vel_msg_.linear.x = -1.0 * ((abs(op_vel_msg_.linear.x) > max_x_vel_) ? max_x_vel_ : op_vel_msg_.linear.x);
                reactive_vel_msg_.linear.x *= -1.0;
              }
              else
              {//! set max x-vel
                reactive_vel_msg_.linear.x = -1.0 * ((abs(op_vel_msg_.linear.x) > max_x_vel_) ? max_x_vel_ : op_vel_msg_.linear.x);
                if(abs(manual_variable_[3]->getValue()) <= 0.30 || abs(manual_variable_[4]->getValue()) <= 0.30)
                  reactive_vel_msg_.linear.x *= -1.0;
              }

              /*std::cout << "\nSteer(fuzzy): " << (steer * react_percent_)
                        << "\nSteer(operator): " << op_vel_msg_.angular.z
                        << "\nSteer(combined): " << reactive_vel_msg_.angular.z
                        << " | LinSteer: " << reactive_vel_msg_.linear.x << std::endl;*/
            }
          }
        }
      }
      else
      {//! if no obstacle pattern
        reactive_vel_msg_ = op_vel_msg_;
      }
      if(is_aware_)
        vel_pub_.publish(reactive_vel_msg_); //! publish fuzzy control

      {//!IDLE/RUNNING detection
        boost::mutex::scoped_lock vel_lock(vel_mtx_, boost::try_to_lock);
        if((abs(op_vel_msg_.linear.x) > 0.0) || (abs(op_vel_msg_.angular.z) > 0.0))
        {
          is_idle_ = false;
        }
        else
        {
          if((!is_idle_) && (!is_idle_timer_))
          {//! start count down if not idle
            is_idle_timer_ = true;
            idle_since_ = std::chrono::system_clock::now();
          }

          if((!is_idle_) && (std::chrono::duration_cast<std::chrono::seconds>
                (std::chrono::system_clock::now() -  idle_since_).count() >= idle_transition_time_))
          {//! activate idle mode after time period
            is_idle_ = true;
            is_idle_timer_ = false;
          }
        }
      }

      ros::spinOnce();
      r.sleep();
    }
  }

  void ReactiveBase::patternCB(std_msgs::Float32MultiArray::ConstPtr msg)
  {
    if(!is_emergency_ && is_aware_)
    {
      boost::mutex::scoped_lock ptn_lock(ptn_mtx_, boost::try_to_lock);
      pattern_list_.push_back(*msg);
    }
  }

  void ReactiveBase::opInputCB(geometry_msgs::Twist::ConstPtr msg)
  {
    boost::mutex::scoped_lock vel_lock(vel_mtx_, boost::try_to_lock);
    op_vel_msg_ = *msg;
  }

  void ReactiveBase::speedRampCB(atemr_msgs::SpeedRampStatus::ConstPtr msg)
  {
    is_emergency_ = msg->is_emergency.data;
    is_aware_ = msg->is_aware.data;
  }
}
