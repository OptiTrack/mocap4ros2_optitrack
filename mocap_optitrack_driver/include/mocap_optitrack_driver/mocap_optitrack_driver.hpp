// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2019 Intelligent Robotics Lab
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
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>

#ifndef MOCAP_OPTITRACK_DRIVER__MOCAP_OPTITRACK_DRIVER_HPP_
#define MOCAP_OPTITRACK_DRIVER__MOCAP_OPTITRACK_DRIVER_HPP_

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/time.hpp"

#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"
#include "mocap_msgs/msg/rigid_bodies.hpp"
#include "mocap_msgs/msg/rigid_body.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "tf2/buffer_core.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetTypes.h>

namespace mocap_optitrack_driver {

class OptitrackDriverNode : public rclcpp_lifecycle::LifecycleNode {
public:
  OptitrackDriverNode();
  ~OptitrackDriverNode();

  using CallbackReturnT =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State &state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State &state);

  bool connect_optitrack();
  bool disconnect_optitrack();
  void set_settings_optitrack();
  bool stop_optitrack();
  void initParameters();

  void process_frame(sFrameOfMocapData *data);
  void publish_tf_data(sFrameOfMocapData *data, rclcpp::Duration frame_delay);
  void publish_pose_stamped_data(sFrameOfMocapData *data,
                                 rclcpp::Duration frame_delay);
  void publish_pose_cov_stamped_data(sFrameOfMocapData *data,
                                     rclcpp::Duration frame_delay);
  void make_static_transform(); // TODO: make a utility.cpp and put utility
                                // functions in there
  void create_pose_stamped_publishers();
  void create_pose_cov_stamped_publishers();

protected:
  void update_rigid_bodies(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void update_rigid_body_id_map();
  void get_rigid_bodies_from_params();
  bool parse_version(uint8_t *ret_version, std::string str_version);

  NatNetClient *client;

  std::chrono::nanoseconds
  get_optitrack_system_latency(sFrameOfMocapData *data);

  sNatNetClientConnectParams client_params_;
  sServerDescription server_description_;
  sDataDescriptions *data_descriptions_{nullptr};
  sFrameOfMocapData latest_data_;
  sRigidBodyData latest_body_frame_data_;

  std::unordered_map<int, std::string> id_rigid_body_map_;
  std::unordered_map<std::string, int> rigid_body_id_map_;
  std::set<std::string> rigid_bodies_to_publish_;

  rclcpp_lifecycle::LifecyclePublisher<mocap_msgs::msg::Markers>::SharedPtr
      mocap_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap_msgs::msg::RigidBodies>::SharedPtr
      mocap_rigid_body_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<
                            geometry_msgs::msg::PoseStamped>::SharedPtr>
      pose_stamped_publishers_;
  std::map<std::string,
           rclcpp_lifecycle::LifecyclePublisher<
               geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
      pose_cov_stamped_publishers_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr update_rigid_bodies_srv_;

  std::string connection_type_;
  std::string server_address_;
  std::string local_address_;
  std::string multicast_address_;
  uint16_t server_command_port_;
  uint16_t server_data_port_;
  bool pub_PoseStamped_topics_ = false;
  bool pub_PoseWithCovarianceStamped_topics_ = false;
  bool pub_mocap_msgs_rigid_bodies_topic_ = false;
  bool pub_mocap_msgs_markers_topic_ = false;
  bool publish_tf_ = false;
  bool activate_publishing_ = false;
  bool publish_y_up_tf_ = false;
  std::string rb_parent_frame_name_;
  std::string y_up_frame_name_;
  std::string natnet_version_ = "";
  uint8_t natnet_version_ints[4] = {0, 0, 0, 0};
  bool valid_version = false;
  double positional_variance_;
  double rotational_variance_;

  uint64_t frame_number_{0};
};

void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData *data,
                                            void *pUserData);

} // namespace mocap_optitrack_driver

#endif // MOCAP_OPTITRACK_DRIVER__MOCAP_OPTITRACK_DRIVER_HPP_
