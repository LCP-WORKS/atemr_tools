#include <reactive_base/reactivebase.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reactive_control_node");
  ros::NodeHandle g_nh(""), private_nh("~");
  try
  {
    rb::ReactiveBase reactbase(private_nh, g_nh);
    reactbase.run();
  } catch (ATEMRException &e)
  {
    ROS_ERROR_STREAM("Error code: " << e.code() << " - Reason: " << e.what());
    exit(1);
  } catch(fl::Exception &e)
  {
    ROS_ERROR_STREAM("Reason: " << e.what());
  }


  return 0;
}
