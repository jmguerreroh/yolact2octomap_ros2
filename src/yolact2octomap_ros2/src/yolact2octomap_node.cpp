// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "yolact2octomap_ros2/yolact2octomap_node.hpp"

using namespace std::chrono_literals;

namespace yolact2octomap {

Yolact2Octomap::Yolact2Octomap(const std::string & name): Node(name)
{
	double probHit, probMiss, thresMin, thresMax;
	double voxel_res = 0.1;
  probHit = 0.7;
  probMiss = 0.4;
  thresMin = 0.12;
  thresMax = 0.97;

	octree_ = std::make_shared<octomap::OcTree>(voxel_res);
  octree_->setProbHit(probHit);
  octree_->setProbMiss(probMiss);
  octree_->setClampingThresMin(thresMin);
  octree_->setClampingThresMax(thresMax);

  pub_ = create_publisher<octomap_msgs::msg::Octomap>("/yolact2octomap", 1);

  initOctomap();
}

void Yolact2Octomap::initOctomap() {
  octomap::KeySet cells;
  octomap::KeyRay keyRay;
  //------------
  double x_min = 0.1;
  double y_min = 0.3;
  double z_min = 0.4;
  double x_max = 0.3;
  double y_max = 0.5;
  double z_max = 0.7;
  double step = 0.1;

  for (double y = y_min; y < +(y_min+y_max)+step/2; y+=0.05) {
    for (double z = z_min; z < (z_min+z_max)+step/2; z+=0.05) {
      octomap::point3d point1(x_min, y, z);
      octomap::point3d point2(x_min+x_max, y, z);
      if (octree_->computeRayKeys(point1, point2, keyRay)) {
        cells.insert(keyRay.begin(), keyRay.end());
      }
    }
  }

  //------------
  /*
  octomap::point3d origin(0.0, 0.0, 0.0);
  octomap::point3d target1(3.0, 1.0, 0.1);
  octomap::point3d target2(-3.0, -1.0, 2.0);
  octomap::point3d target3(1.0, 1.0, 1.0);
  // We take some voxels
  if (octree_->computeRayKeys(origin, target1, keyRay)) {
    cells.insert(keyRay.begin(), keyRay.end());
  }
  if (octree_->computeRayKeys(target1, target2, keyRay)) {
    cells.insert(keyRay.begin(), keyRay.end());
  }
  if (octree_->computeRayKeys(origin, target3, keyRay)) {
    cells.insert(keyRay.begin(), keyRay.end());
  }
  */
  // We insert these voxels in the octree
  for(auto cell : cells){
    octree_->updateNode(cell, false);
    octree_->setNodeValue(cell, 1.0, true);
  }
}

void Yolact2Octomap::publishFullOctoMap() {
	octomap_msgs::msg::Octomap map;
  map.header.frame_id = "odom";
  //map.header.stamp = rostime;
  size_t octomapSize = octree_->size();
  if (octomapSize <= 1){
    RCLCPP_WARN(get_logger(),"Nothing to publish, octree is empty");
    return;
  }
  if (octomap_msgs::fullMapToMsg(*octree_, map)){
    pub_->publish(map);
    RCLCPP_INFO(get_logger(), "publishing a octomap of size [%u]", octomapSize);
  }else{
    RCLCPP_ERROR(get_logger(),"Error serializing OctoMap");
  }
}

void Yolact2Octomap::step() {
  publishFullOctoMap();
}

}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
  auto yolact2octomap = std::make_shared<yolact2octomap::Yolact2Octomap>("yolact2octomap_node");
  rclcpp::Rate loop_rate(1000ms); 
  while (rclcpp::ok()) {
    yolact2octomap->step();
    rclcpp::spin_some(yolact2octomap);
    loop_rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}