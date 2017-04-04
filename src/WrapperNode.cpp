#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/CameraStereo.h>
#include <rtabmap/core/OdometryF2M.h>
#include <rtabmap/core/OdometryThread.h>

#include "ros/ros.h"
#include "ros/console.h"

bool running = true;
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "lk_rtabmap_zed_node");

  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  const auto opticalRotation = rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0);

  if (!rtabmap::CameraStereoZed::available()) {
    ROS_ERROR("unable to get zed camera!");
    return -1;
  }
  auto camera = rtabmap::CameraStereoZed(0, 2, 1, 1, true, 0, opticalRotation);
  if (!camera.init()) {
    ROS_ERROR("unable to init camera!");
    return -2;
  }
  rtabmap::CameraThread cameraThread(&camera);

  auto odometry = rtabmap::OdometryF2M();
  rtabmap::OdometryThread odometryThread(&odometry);

  auto rtabmap = rtabmap::Rtabmap();
  rtabmap.init();
  rtabmap::RtabmapThread rtabmapThread(&rtabmap);

  odometryThread.registerToEventsManager();
  rtabmapThread.registerToEventsManager();

  cameraThread.start();
  odometryThread.start();
  rtabmapThread.start();

  while (running) {
    ros::spinOnce();
  }

  rtabmapThread.registerToEventsManager();
  odometryThread.registerToEventsManager();


  cameraThread.kill();
  odometryThread.join(true);
  rtabmapThread.join(true);

  return 0;
}
