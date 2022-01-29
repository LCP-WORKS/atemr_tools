#include <scan_processor/scanprocessor.h>

namespace sp
{
  ScanProcessor::ScanProcessor(const ros::NodeHandle &pnh, const ros::NodeHandle &gnh): p_nh_(pnh), g_nh_(gnh)
  {
    tf_buffer_.reset(new tf2_ros::Buffer());
    listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    base_frame_ = std::string("base_footprint");
    odom_frame_ = std::string("odom");
    input_pcl2_.reset(new pcl::PCLPointCloud2());
    out_pcl2_.reset(new pcl::PCLPointCloud2());
    verts_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    //! prepare pcl filter
    p_nh_.param("obstacle_radius", obstacle_radius_, 0.12);
    p_nh_.param("pattern_type", pattern_type_, 0);
    outrem_.reset(new pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2>());
    outrem_->setRadiusSearch(obstacle_radius_);
    outrem_->setMinNeighborsInRadius(3);


    cloud_pub2_.reset(new ros::Publisher());
    *(cloud_pub2_) = g_nh_.advertise<sensor_msgs::PointCloud2>("/cloud2", 2);

    obst_pub_.reset(new ros::Publisher());
    *(obst_pub_) = g_nh_.advertise<std_msgs::Float32MultiArray>("/pattern", 1);
    obst_msg_.layout.dim.resize(1, std_msgs::MultiArrayDimension());
    obst_msg_.layout.dim[0].size = 6;
    obst_msg_.layout.dim[0].stride = 1;

    scan_sub_.reset(new ros::Subscriber());
    *(scan_sub_) = g_nh_.subscribe<sensor_msgs::LaserScan>("/scan", 2, boost::bind(&ScanProcessor::scanCB, this, _1));

    HEX_.reserve(6);
    for(std::size_t i=0; i < 6; i++)
    { HEX_.push_back(ROLLINMEANDBL(tag::rolling_window::window_size = WINDOW_SIZE)); }
    t_.reset(new boost::thread(&ScanProcessor::patternSegmentation, this));
  }

  void ScanProcessor::scanCB(const sensor_msgs::LaserScanConstPtr &msg)
  {
    processScan(msg);
  }

  void ScanProcessor::run()
  {
    ros::Rate r(15);
    while(ros::ok() && !ros::isShuttingDown())
    {
      ROS_INFO_THROTTLE(5, "SP running ...");
      ros::spin();
      r.sleep();
    }
    can_run_ = false;

    ROS_INFO("Stopping pattern thread ...");
    t_->interrupt();
    while(t_->interruption_requested())
      t_->join();
  }

  void ScanProcessor::processScan(const sensor_msgs::LaserScanConstPtr &scan_in)
  {
    try
    {
      if(tf_buffer_->canTransform(base_frame_, scan_in->header.frame_id, scan_in->header.stamp, ros::Duration(0)))
      {
        sensor_msgs::PointCloud2 cloud_msg, cloud_msg2;
        projector_.transformLaserScanToPointCloud(base_frame_, *scan_in, cloud_msg, *tf_buffer_);
        base_tf_ = tf_buffer_->lookupTransform(odom_frame_, base_frame_, ros::Time::now(), ros::Duration(3));
        pcl_conversions::toPCL(cloud_msg, *input_pcl2_);
        if(cld_mtx_.try_lock())
        {
          outrem_->setInputCloud(input_pcl2_);
          outrem_->filter(*out_pcl2_);
          cld_mtx_.unlock();
        }
        /*pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new PointCloud);
        pcl::fromPCLPointCloud2(pcl_pc2_, *temp_cloud);
        pcl_conversions::toPCL(ros::Time::now(), temp_cloud->header.stamp);
        cloud_pub_->publish(*temp_cloud);*/
        //cloud_pub2_->publish(cloud_msg);
        pcl_conversions::fromPCL(*out_pcl2_, cloud_msg2);
        cloud_msg.header.frame_id = base_frame_;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_pub2_->publish(cloud_msg2);
      }
    }
    catch (tf2::LookupException &ex){ ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
    catch(tf2::ConnectivityException &ex){ ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
    catch(tf2::ExtrapolationException &ex){ ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
    catch(tf2::InvalidArgumentException &ex){ ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
    catch (tf2::TransformException &ex) { ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
  }

  void ScanProcessor::patternSegmentation()
  {
    /*
     * Iterates over the filtered cloud
     * Groups obstacle distances according to the selected pattern (uses accumulator<average>)
    */
    while(can_run_)
    {
      if(cld_mtx_.try_lock())
      {
        //ROS_INFO_STREAM("Cloud. size: " << out_pcl2_->data.size());
        verts_->clear();
        pcl::fromPCLPointCloud2(*out_pcl2_, *verts_);
        cld_mtx_.unlock();
      }

      if(!verts_->empty())
      {
        try
        {
          double res = 0.0;
          /*std::cout << "Verts. size: " << verts_->size()
                    << "\n RobotPose: " << base_tf_.transform.translation.x << ", "
                                        << base_tf_.transform.translation.y << ", "
                                        << base_tf_.transform.translation.z << std::endl;*/
          if(pattern_type_ == PATTERN_TYPE::HEXAGONAL)
          {
            obst_msg_.layout.dim[0].label = "hex-pattern";
            for(const auto &point : *verts_)
            {//! segment by pattern
              /*std::cout << point._PointXYZ::y << ", " << point._PointXYZ::x
                        << " Dist: " << getDistance(point)
                        << " Angle: " << getAngle(point) << std::endl;*/
              res = getAngle(point);
              if(res >= 0.0 && res < 1.04719) //! 0° - 59°
                HEX_[0](getDistance(point));
              else if(res >= 1.04719 && res < 2.09439) //! 60° - 119° (dead ahead)
                HEX_[1](getDistance(point));
              else if(res >= 2.09439 && res < 3.14159) //! 120° - 179°
                HEX_[2](getDistance(point));
              else if(res >= 3.14159 && res < 4.18879) //! 180° - 239° (aft)
                HEX_[3](getDistance(point));
              else if(res >= 4.18879 && res < 5.23598) //! 240° - 299°
                HEX_[4](getDistance(point));
              else //! 300° - 360°
                HEX_[5](getDistance(point));
            }
            /*std::cout << "FR-" << rolling_mean(HEX_[0])
                      << "\nFront - " << rolling_mean(HEX_[1])
                      << "\nFL- " << rolling_mean(HEX_[2])
                      << "\nRL- " << rolling_mean(HEX_[3])
                      << "\nRear - " << rolling_mean(HEX_[4])
                      << "\nRR- " << rolling_mean(HEX_[5]) << std::endl;*/
          }
          else if(pattern_type_ == PATTERN_TYPE::HEXAGONAL_90)
          {
            obst_msg_.layout.dim[0].label = "hex-pattern-90";
            for(const auto &point : *verts_)
            {//! segment by pattern
              res = getAngle(point);
              if(res >= 0.52359 && res < 1.57079) //! 30° - 90°
                HEX_[0](getDistance(point));
              else if(res >= 1.57079 && res < 2.61799) //! 90° - 150° (dead ahead)
                HEX_[1](getDistance(point));
              else if(res >= 2.61799 && res < 3.66519) //! 150° - 210°
                HEX_[2](getDistance(point));
              else if(res >= 3.66519 && res < 4.71238) //! 210° - 270° (aft)
                HEX_[3](getDistance(point));
              else if(res >= 4.71238 && res < 5.75958) //! 270° - 330°
                HEX_[4](getDistance(point));
              else //! 330° - 30°
                HEX_[5](getDistance(point));
            }
          }
          for(std::size_t i=0; i < 6; i++)
          { obst_msg_.data.push_back(float(rolling_mean(HEX_[i]))); }
          obst_pub_->publish(obst_msg_);
          HEX_.clear();
          obst_msg_.data.clear();
          for(std::size_t i=0; i < 6; i++)
          { HEX_.push_back(ROLLINMEANDBL(tag::rolling_window::window_size = WINDOW_SIZE)); }
          std::this_thread::sleep_for(50ms);
        }
        catch (tf2::LookupException &ex){ ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
        catch(tf2::ConnectivityException &ex){ ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
        catch(tf2::ExtrapolationException &ex){ ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
        catch(tf2::InvalidArgumentException &ex){ ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
        catch (tf2::TransformException &ex) { ROS_ERROR_STREAM(static_cast<std::string>(ex.what())); }
      }
      else
      { std::this_thread::sleep_for(50ms); }
    }
  }

  double ScanProcessor::getAngle(const pcl::PointXYZ &p)
  {
    double res = double(atan2(p._PointXYZ::x, -1*p._PointXYZ::y));
    if(res < 0.0)
      res += (2 * M_PI);
    return res;
  }
  double ScanProcessor::getDistance(const pcl::PointXYZ &p)
  {
    return sqrt(
                pow(double(p._PointXYZ::y), 2) +
                pow(double(-1 * p._PointXYZ::x), 2)
          );
  }
}
