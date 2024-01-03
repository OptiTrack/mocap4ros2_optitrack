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

#include <memory>
#include <string>
#include <vector>

#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "mocap_optitrack_driver/mocap_optitrack_driver.hpp"

namespace mocap_optitrack_driver {

using std::placeholders::_1;
using std::placeholders::_2;

OptitrackDriverNode::OptitrackDriverNode()
    : LifecycleNode(
          "mocap_optitrack_driver_node",
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true)) {
  client = new NatNetClient();
  client->SetFrameReceivedCallback(process_frame_callback, this);
}

OptitrackDriverNode::~OptitrackDriverNode() {}

void OptitrackDriverNode::set_settings_optitrack() {
  if (connection_type_ == "Multicast") {
    client_params_.connectionType = ConnectionType::ConnectionType_Multicast;
    client_params_.multicastAddress = multicast_address_.c_str();
  } else if (connection_type_ == "Unicast") {
    client_params_.connectionType = ConnectionType::ConnectionType_Unicast;
  } else {
    RCLCPP_FATAL(get_logger(),
                 "Unknown connection type -- options are Multicast, Unicast");
    rclcpp::shutdown();
  }

  // TODO: set bitstream version. See sConnectionOptions in NatNetTypes.h
  // TODO: Verify bitstream after setting SendMessageAndWait("Bitstream"...

  client_params_.serverAddress = server_address_.c_str();
  client_params_.localAddress = local_address_.c_str();
  client_params_.serverCommandPort = server_command_port_;
  client_params_.serverDataPort = server_data_port_;
  if (valid_version) {
    client_params_.BitstreamVersion[0] = natnet_version_ints[3];
    client_params_.BitstreamVersion[1] = natnet_version_ints[2];
    client_params_.BitstreamVersion[2] = natnet_version_ints[1];
    client_params_.BitstreamVersion[3] = natnet_version_ints[0];
    RCLCPP_INFO_STREAM(
        get_logger(), "Requesting NatNet version: "
                          << unsigned(client_params_.BitstreamVersion[3]) << "."
                          << unsigned(client_params_.BitstreamVersion[2]) << "."
                          << unsigned(client_params_.BitstreamVersion[1]) << "."
                          << unsigned(client_params_.BitstreamVersion[0]));
  }
}

void OptitrackDriverNode::update_rigid_bodies(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;

  if (data_descriptions_) {
    NatNet_FreeDescriptions(data_descriptions_);
  }
  RCLCPP_INFO(get_logger(), "Requesting Data Descriptions for Rigid Bodies");

  int result = client->GetDataDescriptionList(&data_descriptions_);
  if (result != ErrorCode::ErrorCode_OK) {
    response->success = false;
    return;
  }
  // populate rigid body id map
  update_rigid_body_id_map();
  get_rigid_bodies_from_params();
  response->success = true;
}

void OptitrackDriverNode::update_rigid_body_id_map() {
  for (int i = 0; i < data_descriptions_->nDataDescriptions; ++i) {
    if (data_descriptions_->arrDataDescriptions[i].type ==
        DataDescriptors::Descriptor_RigidBody) {
      sRigidBodyDescription *rigid_body =
          data_descriptions_->arrDataDescriptions[i].Data.RigidBodyDescription;
      id_rigid_body_map_[rigid_body->ID] = rigid_body->szName;
      rigid_body_id_map_[rigid_body->szName] = rigid_body->ID;
    }
  }
}

void OptitrackDriverNode::get_rigid_bodies_from_params() {
  const std::vector<std::string> prefix{"rigid_bodies"};
  const auto result =
      this->get_node_parameters_interface()->list_parameters(prefix, 0);

  for (const std::string &prefix : result.prefixes) {
    std::string temp_name;
    if (!get_parameter<std::string>(prefix + ".name", temp_name)) {
      RCLCPP_WARN_STREAM(get_logger(),
                         "Unable to find 'name' sub-parameter in: " << prefix);
      continue;
    }
    if (rigid_body_id_map_.count(temp_name) > 0) {
      rigid_bodies_to_publish_.insert(temp_name);
    } else {
      RCLCPP_WARN_STREAM(
          get_logger(),
          "Unable to find '"
              << temp_name
              << "' on NatNet server. Check Assets tab in Motive.");
    }
  }
  RCLCPP_INFO(get_logger(),
              "Able to publish tf of the following rigid bodies:");
  for (const auto &str : rigid_bodies_to_publish_) {
    RCLCPP_INFO_STREAM(get_logger(), "\t" << str);
  }
}

bool OptitrackDriverNode::stop_optitrack() {
  RCLCPP_INFO(get_logger(), "Disconnecting from optitrack DataStream SDK");

  return true;
}

void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData *data,
                                            void *pUserData) {
  static_cast<OptitrackDriverNode *>(pUserData)->process_frame(data);
}

std::chrono::nanoseconds
OptitrackDriverNode::get_optitrack_system_latency(sFrameOfMocapData *data) {
  const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

  if (bSystemLatencyAvailable) {
    const double clientLatencySec =
        client->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp);
    const double clientLatencyMillisec = clientLatencySec * 1000.0;
    const double transitLatencyMillisec =
        client->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

    const double largeLatencyThreshold = 100.0;
    if (clientLatencyMillisec >= largeLatencyThreshold) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 500,
                           "Optitrack system latency >%.0f ms: [Transmission: "
                           "%.0fms, Total: %.0fms]",
                           largeLatencyThreshold, transitLatencyMillisec,
                           clientLatencyMillisec);
    }

    return round<std::chrono::nanoseconds>(std::chrono::duration<float>{
        clientLatencySec}); // if this line breaks, remove the #include <cmath>
  } else {
    RCLCPP_WARN_ONCE(get_logger(), "Optitrack's system latency not available");
    return std::chrono::nanoseconds::zero();
  }
}

void OptitrackDriverNode::process_frame(sFrameOfMocapData *data) {
  if (get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  frame_number_++;
  rclcpp::Duration frame_delay =
      rclcpp::Duration(get_optitrack_system_latency(data));

  std::map<int, std::vector<mocap_msgs::msg::Marker>> marker2rb;

  // Markers
  if (mocap_markers_pub_->get_subscription_count() > 0 and
      pub_mocap_msgs_markers_topic_ and activate_publishing_) {
    mocap_msgs::msg::Markers msg;
    msg.header.stamp = now() - frame_delay;
    msg.header.frame_id = "map";
    msg.frame_number = frame_number_;

    for (int i = 0; i < data->nLabeledMarkers; i++) {
      bool Unlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
      bool ActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);
      sMarker &marker_data = data->LabeledMarkers[i];
      int modelID, markerID;
      NatNet_DecodeID(marker_data.ID, &modelID, &markerID);

      mocap_msgs::msg::Marker marker;
      marker.id_type = mocap_msgs::msg::Marker::USE_INDEX;
      marker.marker_index = i;
      marker.translation.x = marker_data.x;
      marker.translation.y = marker_data.y;
      marker.translation.z = marker_data.z;
      if (ActiveMarker || Unlabeled) {
        msg.markers.push_back(marker);
      } else {
        marker2rb[modelID].push_back(marker);
      }
    }
    mocap_markers_pub_->publish(msg);
  }

  if (mocap_rigid_body_pub_->get_subscription_count() > 0 and
      pub_mocap_msgs_rigid_bodies_topic_) {
    mocap_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.stamp = now() - frame_delay;
    msg_rb.header.frame_id = "map";
    msg_rb.frame_number = frame_number_;

    for (int i = 0; i < data->nRigidBodies; i++) {
      mocap_msgs::msg::RigidBody rb;

      rb.rigid_body_name = std::to_string(data->RigidBodies[i].ID);
      rb.pose.position.x = data->RigidBodies[i].x;
      rb.pose.position.y = data->RigidBodies[i].y;
      rb.pose.position.z = data->RigidBodies[i].z;
      rb.pose.orientation.x = data->RigidBodies[i].qx;
      rb.pose.orientation.y = data->RigidBodies[i].qy;
      rb.pose.orientation.z = data->RigidBodies[i].qz;
      rb.pose.orientation.w = data->RigidBodies[i].qw;
      rb.markers = marker2rb[data->RigidBodies[i].ID];

      msg_rb.rigidbodies.push_back(rb);
    }

    mocap_rigid_body_pub_->publish(msg_rb);
  }

  if (publish_tf_ and activate_publishing_) {
    publish_tf_data(data, frame_delay);
  }
  if (pub_PoseStamped_topics_ and activate_publishing_) {
    publish_pose_stamped_data(data, frame_delay);
  }
  if (pub_PoseWithCovarianceStamped_topics_ and activate_publishing_) {
    publish_pose_cov_stamped_data(data, frame_delay);
  }
}

void OptitrackDriverNode::publish_tf_data(sFrameOfMocapData *data,
                                          rclcpp::Duration frame_delay) {
  for (int i = 0; i < data->nRigidBodies; i++) {
    if (id_rigid_body_map_.count(data->RigidBodies[i].ID) > 0 &&
        rigid_bodies_to_publish_.count(
            id_rigid_body_map_[data->RigidBodies[i].ID]) > 0) {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now() - frame_delay;
      t.header.frame_id = rb_parent_frame_name_;
      t.child_frame_id = id_rigid_body_map_[data->RigidBodies[i].ID];

      t.transform.translation.x = data->RigidBodies[i].x;
      t.transform.translation.y = data->RigidBodies[i].y;
      t.transform.translation.z = data->RigidBodies[i].z;
      t.transform.rotation.x = data->RigidBodies[i].qx;
      t.transform.rotation.y = data->RigidBodies[i].qy;
      t.transform.rotation.z = data->RigidBodies[i].qz;
      t.transform.rotation.w = data->RigidBodies[i].qw;

      tf_broadcaster_->sendTransform(t);
    }
  }
}

void OptitrackDriverNode::publish_pose_stamped_data(
    sFrameOfMocapData *data, rclcpp::Duration frame_delay) {
  for (int i = 0; i < data->nRigidBodies; i++) {
    if (id_rigid_body_map_.count(data->RigidBodies[i].ID) > 0 &&
        rigid_bodies_to_publish_.count(
            id_rigid_body_map_[data->RigidBodies[i].ID]) > 0) {
      auto pose_stamped_msg =
          std::make_unique<geometry_msgs::msg::PoseStamped>();

      pose_stamped_msg->header.stamp = this->get_clock()->now() - frame_delay;
      pose_stamped_msg->header.frame_id =
          id_rigid_body_map_[data->RigidBodies[i].ID];

      pose_stamped_msg->pose.position.x = data->RigidBodies[i].x;
      pose_stamped_msg->pose.position.y = data->RigidBodies[i].y;
      pose_stamped_msg->pose.position.z = data->RigidBodies[i].z;
      pose_stamped_msg->pose.orientation.x = data->RigidBodies[i].qx;
      pose_stamped_msg->pose.orientation.y = data->RigidBodies[i].qy;
      pose_stamped_msg->pose.orientation.z = data->RigidBodies[i].qz;
      pose_stamped_msg->pose.orientation.w = data->RigidBodies[i].qw;

      pose_stamped_publishers_[id_rigid_body_map_[data->RigidBodies[i].ID]]
          ->publish(std::move(pose_stamped_msg));
    }
  }
}

void OptitrackDriverNode::publish_pose_cov_stamped_data(
    sFrameOfMocapData *data, rclcpp::Duration frame_delay) {
  for (int i = 0; i < data->nRigidBodies; i++) {
    if (id_rigid_body_map_.count(data->RigidBodies[i].ID) > 0 &&
        rigid_bodies_to_publish_.count(
            id_rigid_body_map_[data->RigidBodies[i].ID]) > 0) {
      auto pose_cov_stamped_msg =
          std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

      pose_cov_stamped_msg->header.stamp =
          this->get_clock()->now() - frame_delay;
      pose_cov_stamped_msg->header.frame_id =
          id_rigid_body_map_[data->RigidBodies[i].ID];

      pose_cov_stamped_msg->pose.pose.position.x = data->RigidBodies[i].x;
      pose_cov_stamped_msg->pose.pose.position.y = data->RigidBodies[i].y;
      pose_cov_stamped_msg->pose.pose.position.z = data->RigidBodies[i].z;
      pose_cov_stamped_msg->pose.pose.orientation.x = data->RigidBodies[i].qx;
      pose_cov_stamped_msg->pose.pose.orientation.y = data->RigidBodies[i].qy;
      pose_cov_stamped_msg->pose.pose.orientation.z = data->RigidBodies[i].qz;
      pose_cov_stamped_msg->pose.pose.orientation.w = data->RigidBodies[i].qw;

      pose_cov_stamped_msg->pose.covariance[0] = positional_variance_;
      pose_cov_stamped_msg->pose.covariance[7] = positional_variance_;
      pose_cov_stamped_msg->pose.covariance[14] = positional_variance_;
      pose_cov_stamped_msg->pose.covariance[21] = rotational_variance_;
      pose_cov_stamped_msg->pose.covariance[28] = rotational_variance_;
      pose_cov_stamped_msg->pose.covariance[35] = rotational_variance_;

      pose_cov_stamped_publishers_[id_rigid_body_map_[data->RigidBodies[i].ID]]
          ->publish(std::move(pose_cov_stamped_msg));
    }
  }
}

/**
 * A static transform from XY ground plane Z up to ZX ground plane Y up
 * This allows for visualization to match motive output
 * This new transform has a child that is the optitrack frame of reference
 */
void OptitrackDriverNode::make_static_transform() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = y_up_frame_name_;
  t.child_frame_id = rb_parent_frame_name_;

  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  // XYZ -> ZXY
  t.transform.rotation.x = 0.5;
  t.transform.rotation.y = 0.5;
  t.transform.rotation.z = 0.5;
  t.transform.rotation.w = 0.5;

  tf_static_broadcaster_->sendTransform(t);
}

void OptitrackDriverNode::create_pose_stamped_publishers() {
  for (const auto &rigid_body : rigid_bodies_to_publish_) {
    std::string str = rigid_body;
    std::replace(str.begin(), str.end(), ' ', '_');
    auto pub = create_publisher<geometry_msgs::msg::PoseStamped>(
        "pose_stamped/" + str, rclcpp::QoS(1000));
    pose_stamped_publishers_[rigid_body] = pub;
  }
}

void OptitrackDriverNode::create_pose_cov_stamped_publishers() {
  for (std::string rigid_body : rigid_bodies_to_publish_) {
    std::string str = rigid_body;
    std::replace(str.begin(), str.end(), ' ', '_');
    auto pub = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "pose_with_cov_stamped/" + str, rclcpp::QoS(1000));
    pose_cov_stamped_publishers_[rigid_body] = pub;
  }
}

using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// The next Callbacks are used to manage behavior in the different states of the
// lifecycle node.
CallbackReturnT
OptitrackDriverNode::on_configure(const rclcpp_lifecycle::State &state) {
  (void)state;
  initParameters();

  if (pub_mocap_msgs_markers_topic_) {
    mocap_markers_pub_ = create_publisher<mocap_msgs::msg::Markers>(
        "markers", rclcpp::QoS(1000));
  }
  if (pub_mocap_msgs_rigid_bodies_topic_) {
    mocap_rigid_body_pub_ = create_publisher<mocap_msgs::msg::RigidBodies>(
        "rigid_bodies", rclcpp::QoS(1000));
  }
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

  std::string node_name(this->get_name());

  update_rigid_bodies_srv_ = this->create_service<std_srvs::srv::Trigger>(
      node_name + "/update_rigid_bodies",
      std::bind(&OptitrackDriverNode::update_rigid_bodies, this,
                std::placeholders::_1, std::placeholders::_2));

  if (!connect_optitrack()) { // check output and if false try
                              // CallbackReturnT::FAILURE
    RCLCPP_WARN(get_logger(), "Unable to connect. Performing Error Proccesing "
                              "and Entering UNCONFIGURED state");
    return CallbackReturnT::ERROR;
  }

  update_rigid_body_id_map();
  if (publish_y_up_tf_) {
    make_static_transform();
  }

  get_rigid_bodies_from_params();

  if (pub_PoseStamped_topics_) {
    create_pose_stamped_publishers();
  }
  if (pub_PoseWithCovarianceStamped_topics_) {
    create_pose_cov_stamped_publishers();
  }

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_activate(const rclcpp_lifecycle::State &state) {
  (void)state;
  mocap_markers_pub_->on_activate();
  mocap_rigid_body_pub_->on_activate();
  for (auto pub : pose_stamped_publishers_) {
    pub.second->on_activate();
  }
  for (auto pub : pose_cov_stamped_publishers_) {
    pub.second->on_activate();
  }
  activate_publishing_ = true;
  RCLCPP_INFO(get_logger(), "Activated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_deactivate(const rclcpp_lifecycle::State &state) {
  (void)state;
  mocap_markers_pub_->on_deactivate();
  mocap_rigid_body_pub_->on_deactivate();
  activate_publishing_ = false;
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_cleanup(const rclcpp_lifecycle::State &state) {
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  if (disconnect_optitrack()) {
    return CallbackReturnT::SUCCESS;
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
OptitrackDriverNode::on_shutdown(const rclcpp_lifecycle::State &state) {
  (void)state;
  RCLCPP_INFO(get_logger(), "Shut down!\n");

  if (disconnect_optitrack()) {
    return CallbackReturnT::SUCCESS;
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
OptitrackDriverNode::on_error(const rclcpp_lifecycle::State &state) {
  (void)state;
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]",
              get_current_state().label().c_str());

  disconnect_optitrack();

  return CallbackReturnT::SUCCESS;
}

bool OptitrackDriverNode::connect_optitrack() {
  RCLCPP_INFO(get_logger(),
              "Trying to connect to Optitrack NatNET SDK at %s ...",
              server_address_.c_str());

  client->Disconnect();
  set_settings_optitrack();

  // TODO: Catch the error state of not connecting and return to Unconfigured
  // state
  if (client->Connect(client_params_) == ErrorCode::ErrorCode_OK) {
    RCLCPP_INFO(get_logger(), "... connected!");

    memset(&server_description_, 0, sizeof(server_description_));
    client->GetServerDescription(&server_description_);
    if (!server_description_.HostPresent) {
      RCLCPP_DEBUG(get_logger(),
                   "Unable to connect to server. Host not present.");
      return false;
    }

    if (client->GetDataDescriptionList(&data_descriptions_) != ErrorCode_OK ||
        !data_descriptions_) {
      RCLCPP_WARN(get_logger(),
                  "[Client] Unable to retrieve Data Descriptions.\n");
    }

    RCLCPP_INFO(get_logger(), "\n[Client] Server application info:\n");
    RCLCPP_INFO(get_logger(), "Application: %s (ver. %d.%d.%d.%d)\n",
                server_description_.szHostApp,
                server_description_.HostAppVersion[0],
                server_description_.HostAppVersion[1],
                server_description_.HostAppVersion[2],
                server_description_.HostAppVersion[3]);
    RCLCPP_INFO(get_logger(), "NatNet Version: %d.%d.%d.%d\n",
                server_description_.NatNetVersion[0],
                server_description_.NatNetVersion[1],
                server_description_.NatNetVersion[2],
                server_description_.NatNetVersion[3]);
    RCLCPP_INFO(get_logger(), "Client IP:%s\n", client_params_.localAddress);
    RCLCPP_INFO(get_logger(), "Server IP:%s\n", client_params_.serverAddress);
    RCLCPP_INFO(get_logger(), "Server Name:%s\n",
                server_description_.szHostComputerName);

    void *pResult;
    int nBytes = 0;

    if (client->SendMessageAndWait("FrameRate", &pResult, &nBytes) ==
        ErrorCode_OK) {
      float fRate = *(static_cast<float *>(pResult));
      RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f\n", fRate);
    } else {
      RCLCPP_DEBUG(get_logger(), "Error getting frame rate.\n");
    }
  } else {
    RCLCPP_INFO(get_logger(), "... not connected :( ");
    return false;
  }

  return true;
}

bool OptitrackDriverNode::disconnect_optitrack() {
  void *response;
  int nBytes;
  if (client->SendMessageAndWait("Disconnect", &response, &nBytes) ==
      ErrorCode_OK) {
    client->Disconnect();
    RCLCPP_INFO(get_logger(), "[Client] Disconnected");
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "[Client] Disconnect not successful..");
    return false;
  }
}

void OptitrackDriverNode::initParameters() {
  bool valid_params = true;
  valid_params &=
      get_parameter<std::string>("connection_type", connection_type_);
  valid_params &= get_parameter<std::string>("server_address", server_address_);
  valid_params &= get_parameter<std::string>("local_address", local_address_);
  valid_params &=
      get_parameter<std::string>("multicast_address", multicast_address_);
  valid_params &=
      get_parameter<uint16_t>("server_command_port", server_command_port_);
  valid_params &=
      get_parameter<uint16_t>("server_data_port", server_data_port_);

  if (!valid_params) {
    RCLCPP_ERROR(get_logger(), "Not all required Parameters were set");
  }

  // TF related params
  if (!get_parameter<bool>("publish_tf", publish_tf_)) {
    publish_tf_ = false;
  }
  if (publish_tf_) {
    get_parameter<std::string>("rb_parent_frame_name", rb_parent_frame_name_);
  }
  if (!get_parameter<bool>("publish_y_up_tf", publish_y_up_tf_)) {
    publish_y_up_tf_ = false;
  }
  if (publish_y_up_tf_) {
    get_parameter<std::string>("y_up_frame_name", y_up_frame_name_);
  }
  if (get_parameter<std::string>("natnet_version", natnet_version_) &&
      natnet_version_ != "") {
    RCLCPP_INFO(get_logger(), "parsing natnet version");
    valid_version = parse_version(natnet_version_ints, natnet_version_);
  }

  // general topic publishing params
  get_parameter<bool>("pub_PoseStamped_topics", pub_PoseStamped_topics_);
  // RCLCPP_INFO_STREAM(get_logger(), "pub_Pose_topics");
  get_parameter<bool>("pub_PoseWithCovarianceStamped_topics",
                      pub_PoseWithCovarianceStamped_topics_);
  get_parameter<bool>("pub_mocap_msgs_rigid_bodies_topic",
                      pub_mocap_msgs_rigid_bodies_topic_);
  get_parameter<bool>("pub_mocap_msgs_markers_topic",
                      pub_mocap_msgs_markers_topic_);

  if (pub_PoseWithCovarianceStamped_topics_) {
    get_parameter<double>("positional_variance", positional_variance_);
    get_parameter<double>("rotational_variance", rotational_variance_);
  }
}

bool OptitrackDriverNode::parse_version(uint8_t *ret_version,
                                        std::string str_version) {
  std::istringstream iss(str_version);
  std::string octet;
  int count = 0;

  while (std::getline(iss, octet, '.')) {
    try {
      ret_version[count] = (uint8_t)std::stoi(octet);
      count++;
    } catch (const std::exception &e) {
      RCLCPP_WARN_STREAM(get_logger(), "Invalid NatNet Version: " << e.what());
      RCLCPP_WARN_STREAM(
          get_logger(),
          " initializing NatNet client without specifying version");
      return false;
    }
  }

  if (count != 4) {
    RCLCPP_WARN_STREAM(
        get_logger(),
        "Invalid NatNet Version: incorrect count of version numbers");
    RCLCPP_WARN_STREAM(
        get_logger(), " initializing NatNet client without specifying version");
    return false;
  }

  return true;
}

} // namespace mocap_optitrack_driver
