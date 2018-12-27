// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <ros/ros.h>

#include <qapplication.h>

#include <string>

#include "ros_bridge/cloud_odom_ros_subscriber.h"
#include "ros_bridge/cloud_odom_ros_publisher.h"

#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "projections/ring_projection.h"
#include "projections/spherical_projection.h"
#include "utils/radians.h"
#include "tclap/CmdLine.h"

using std::string;

using namespace depth_clustering;

using ClustererT = ImageBasedClusterer<LinearImageLabeler<>>;
uint32_t nsec = 0;
uint32_t sec = 0;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "show_objects_node");
  ros::NodeHandle nh;

  int num_beams_arg = 16; //if kitti,change this value to 64
  int angle_arg = 10;

  if(nh.hasParam("num_beams")){
    nh.getParam("num_beams", num_beams_arg);
  }
  if(nh.hasParam("threshold_angle")){
    nh.getParam("threshold_angle", angle_arg);
  }

  Radians angle_tollerance = Radians::FromDegrees(angle_arg);

  std::unique_ptr<ProjectionParams> proj_params_ptr = nullptr;
  switch (num_beams_arg) {
    case 16:
      proj_params_ptr = ProjectionParams::VLP_16();
      break;
    case 32:
      proj_params_ptr = ProjectionParams::HDL_32();
      break;
    case 64:
      proj_params_ptr = ProjectionParams::HDL_64();
      break;
  }
  if (!proj_params_ptr) {
    fprintf(stderr,
            "ERROR: wrong number of beams: %d. Should be in [16, 32, 64].\n",
            num_beams_arg);
    exit(1);
  }

  QApplication application(argc, argv);

  string sub_topic_clouds = "/velodyne_points";
  string pub_topic_clouds = "/depth_clusterer/segmented_cloud";
  string topic_pose = "/ndt_pose";
  string topic_odom = "/cam_to_odom";
  string pub_frame_id = "lidar";

  CloudOdomRosSubscriber subscriber(&nh, *proj_params_ptr, sub_topic_clouds, "", "",&nsec, &sec);
  CloudOdomRosPublisher publisher(&nh, pub_frame_id, pub_topic_clouds,&nsec, &sec);

  int min_cluster_size = 20;
  int max_cluster_size = 100000;

  ClustererT clusterer(angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

  subscriber.AddClient(&clusterer);
  clusterer.AddClient(&publisher);

  fprintf(stderr, "INFO: Running with angle tollerance: %f degrees\n",
          angle_tollerance.ToDegrees());

  subscriber.StartListeningToRos();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto exit_code = application.exec();

  // if we close application, still wait for ros to shutdown
  ros::waitForShutdown();
  return exit_code;
}
