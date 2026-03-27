/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adore_dynamics_adapters.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_map/map.hpp"
#include "adore_map/map_loader.hpp"
#include "adore_map/route.hpp"
#include "adore_map/traffic_light.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "adore_math/polygon.h"
#include "adore_ros2_msgs/msg/infrastructure_info.hpp"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/traffic_signals.hpp"
#include "adore_ros2_msgs/msg/visualizable_object.hpp"
#include <adore_map_adapters.hpp>

#include "planning/multi_agent_PID.hpp"
#include "planning/multi_agent_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;

namespace adore
{

class DecisionMakerInfrastructure : public rclcpp::Node
{
private:

  bool                      needs_replan( const std::optional<map::Route>& route, const dynamics::VehicleStateDynamic& start_state ) const;
  std::optional<map::Route> make_valid_route( const dynamics::VehicleStateDynamic& start_state,
                                              const std::optional<math::Point2d>&  goal ) const;

  void                            update_routes_for_participants();
  dynamics::TrafficParticipantSet plan_with_pid();
  dynamics::TrafficParticipantSet plan_with_multi_agent_planner();

  rclcpp::TimerBase::SharedPtr                                           main_timer;
  rclcpp::Publisher<ParticipantSetAdapter>::SharedPtr                    publisher_planned_traffic;
  rclcpp::Publisher<adore_ros2_msgs::msg::VisualizableObject>::SharedPtr publisher_infrastructure_position;
  rclcpp::Publisher<adore_ros2_msgs::msg::InfrastructureInfo>::SharedPtr publisher_infrastructure_info;
  rclcpp::Publisher<MapAdapter>::SharedPtr publisher_local_map;

  using StateSubscriber = rclcpp::Subscription<ParticipantAdapter>::SharedPtr;
  std::unordered_map<std::string, StateSubscriber> traffic_participant_subscribers;

  void publish_infrastructure_transform();
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_transform_broadcaster;

  std::shared_ptr<map::Map>              road_map = nullptr;
  adore::dynamics::TrafficParticipantSet latest_traffic_participant_set;
  std::string                            overview;

  std::string traffic_participant_in_topic = "traffic_participant";

  enum class PlannerBackend
  {
    MultiAgentPid,
    MultiAgentPlanner
  } planner_backend;

public:

  bool should_publish_local_map = false; // Since publishing of local map is mostly used for visualization, it should be possible to toggle off
  double time_of_last_local_map_publication_seconds; // To not publish a large local map objects, decision maker infrastructure tracks time since last publication
  double time_gab_between_map_publications_seconds = 2; // Only publish once every 2 seconds

  double              dt                  = 0.1;
  double              local_map_size      = 50;
  double              max_participant_age = 1.0;
  adore::math::Pose2d infrastructure_pose;
  bool                debug             = false;
  double              max_route_length  = 100.0;
  double              route_replan_dist = 10.0;

  void run();
  void update_state();
  void create_subscribers();
  void create_publishers();
  void load_parameters();
  void print_init_info();
  void print_debug_info();
  void plan_traffic();
  void update_dynamic_subscriptions();


  /******************************* PUBLISHER RELATED FUNCTIONS ************************************************************/
  void publish_infrastructure_position();
  void publish_infrastructure_info();

  /******************************* SUBSCRIBER RELATED FUNCTIONS************************************************************/
  void traffic_participant_callback( const dynamics::TrafficParticipant& msg, const std::string& vehicle_namespace );

  explicit DecisionMakerInfrastructure( const rclcpp::NodeOptions& options );
};

} // namespace adore
