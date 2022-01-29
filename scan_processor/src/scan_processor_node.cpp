#include <scan_processor/scanprocessor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_processing");
  ros::NodeHandle g_nh(""), p_nh("~");
  ROS_INFO("Scan Processor is starting");
  sp::ScanProcessor scanproc(p_nh, g_nh);
  scanproc.run();

  return 0;
}
