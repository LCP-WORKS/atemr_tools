#ifndef SCANPROCESSOR_H
#define SCANPROCESSOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/bind.hpp>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <boost/thread.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/rolling_window.hpp>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#define WINDOW_SIZE 30
using namespace std::chrono_literals;
using namespace boost::accumulators;
using ROLLINMEANDBL = accumulator_set<double, stats<tag::rolling_mean>>;
template< typename DEFAULT_INITIALIZABLE >
inline void accReset( DEFAULT_INITIALIZABLE& object )
{
  object.DEFAULT_INITIALIZABLE::~DEFAULT_INITIALIZABLE() ;
  ::new ( boost::addressof(object) ) DEFAULT_INITIALIZABLE() ;
}
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace sp
{
  enum PATTERN_TYPE{
    HEXAGONAL = 0,
    HEXAGONAL_90 = 1,
    OCTAGONAL
  };
  class ScanProcessor
  {
  public:
    ScanProcessor(const ros::NodeHandle &, const ros::NodeHandle &);
    void scanCB(const sensor_msgs::LaserScanConstPtr &);
    void run();
    ~ScanProcessor()
    {
      ros::Subscriber *ptrscansub = scan_sub_.release();
      delete ptrscansub;
      scan_sub_.reset();
      ros::Publisher *ptrcloudpub = cloud_pub2_.release();
      delete ptrcloudpub;
      cloud_pub2_.reset();
      tf2_ros::Buffer *ptrbuf = tf_buffer_.release();
      delete ptrbuf;
      tf_buffer_.reset();
      tf2_ros::TransformListener *ptrlistener = listener_.release();
      delete ptrlistener;
      listener_.reset();
    }
  private:
    void processScan(const sensor_msgs::LaserScanConstPtr &);
    void patternSegmentation();
    double getAngle(const pcl::PointXYZ &);
    double getDistance(const pcl::PointXYZ &);

    ros::NodeHandle p_nh_, g_nh_;
    std::string base_frame_, odom_frame_;
    double obstacle_radius_; int pattern_type_;
    boost::atomic_bool can_run_{true};
    std::unique_ptr<ros::Subscriber> scan_sub_;
    std::unique_ptr<ros::Publisher> cloud_pub2_, obst_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> listener_;
    geometry_msgs::TransformStamped base_tf_;
    laser_geometry::LaserProjection projector_;
    pcl::PCLPointCloud2Ptr input_pcl2_, out_pcl2_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr verts_;
    std::unique_ptr<pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2>> outrem_;
    std::unique_ptr<boost::thread> t_;
    std::mutex cld_mtx_;
    std::vector<ROLLINMEANDBL> HEX_;
    std_msgs::Float32MultiArray obst_msg_;
  };
}

#endif // SCANPROCESSOR_H
