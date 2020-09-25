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

#ifndef YOLACT2OCTOMAP_H_
#define YOLACT2OCTOMAP_H_

#include "rclcpp/rclcpp.hpp"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <math.h>

namespace yolact2octomap {

class Yolact2Octomap : public rclcpp::Node
{
public:
	Yolact2Octomap(const std::string & name);
	void step();

private:
  void initOctomap();
  void publishFullOctoMap();

	std::shared_ptr<octomap::OcTree> octree_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
};

}

#endif /* YOLACT2OCTOMAP_H_ */
