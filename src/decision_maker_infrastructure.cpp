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

#include "decision_maker_infrastructure.hpp"

#include <chrono>
#include <iostream>
#include "adore_ros2_msgs/msg/map.hpp"
#include <adore_dynamics_conversions.hpp>
#include <adore_map_adapters.hpp>

#include <dynamics/vehicle_state.hpp>
#include <planning/planning_helpers.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace adore
{

DecisionMakerInfrastructure::DecisionMakerInfrastructure( const rclcpp::NodeOptions& options ) :
  Node( "decision_maker_infrastructure", options )
{
  load_parameters();
  create_subscribers();
  create_publishers();
  print_init_info();
}

void
DecisionMakerInfrastructure::run()
{
  const auto start_time = std::chrono::steady_clock::now();

  plan_traffic();

  const auto   end_time        = std::chrono::steady_clock::now();
  const double elapsed_time_ms = std::chrono::duration<double, std::milli>( end_time - start_time ).count();

  if( debug )
  {
    RCLCPP_DEBUG( get_logger(), "plan_traffic() took %.3f ms", elapsed_time_ms );
    print_debug_info();
  }

  publish_infrastructure_position();

  if ( road_map && should_publish_local_map )
  {
    if ( now().seconds() - time_of_last_local_map_publication_seconds > time_gab_between_map_publications_seconds )
    {
      auto local_map_ptr = std::make_shared<map::Map>( road_map->get_submap( infrastructure_pose, local_map_size, local_map_size) );
      publisher_local_map->publish( *local_map_ptr );
      time_of_last_local_map_publication_seconds = now().seconds();
    }
  }

  publish_infrastructure_transform();
  update_dynamic_subscriptions(); 
}

bool
DecisionMakerInfrastructure::needs_replan( const std::optional<map::Route>& route, const dynamics::VehicleStateDynamic& start_state ) const
{
  if( !route )
    return true;
  const double s = route->get_s( start_state );
  return route->reference_line.empty() || ( route->get_length() > 0.0 && s >= route->get_length() - route_replan_dist );
}

std::optional<map::Route>
DecisionMakerInfrastructure::make_valid_route( const dynamics::VehicleStateDynamic& start_state,
                                               const std::optional<math::Point2d>&  goal ) const
{
  if( goal )
  {
    map::Route route( start_state, *goal, road_map );
    if( !route.reference_line.empty() )
      return route;
  }
  else
  {
    map::Route route = map::get_default_route( start_state, max_route_length, road_map );
    if( !route.reference_line.empty() )
      return route;
  }
  return std::nullopt;
}

void
DecisionMakerInfrastructure::update_routes_for_participants()
{
  for( auto& [id, participant] : latest_traffic_participant_set.participants )
  {
    if( participant.classification == dynamics::PEDESTRIAN )
      continue;
    if( !needs_replan( participant.route, participant.state ) )
      continue;

    auto route = make_valid_route( participant.state, participant.goal_point );
    if( route )
      participant.route = std::move( *route );
    else
      participant.route.reset();
  }
}

dynamics::TrafficParticipantSet
DecisionMakerInfrastructure::plan_with_pid()
{
  planner::MultiAgentPID pid_planner;

  // This planner modifies the set in-place
  pid_planner.plan_trajectories( latest_traffic_participant_set );

  return latest_traffic_participant_set;
}

dynamics::TrafficParticipantSet
DecisionMakerInfrastructure::plan_with_multi_agent_planner()
{
  planner::MultiAgentPlanner planner_instance;

  planner_instance.set_parameters( {
    {                       "dt",   dt },
    {            "horizon_steps", 30.0 },
    {                "max_speed", 10.0 },
    { "max_lateral_acceleration",  2.0 }
  } );

  auto planned = planner_instance.plan_all_participants( latest_traffic_participant_set, road_map );

  // Attach local submap to routes
  auto submap = road_map->get_submap( infrastructure_pose, local_map_size, local_map_size );
  for( auto& [id, participant] : planned.participants )
  {
    if( participant.route )
    {
      participant.route->map = std::make_shared<map::Map>( submap );
    }
  }

  return planned;
}

void
DecisionMakerInfrastructure::plan_traffic()
{
  if( !road_map )
  {
    std::cerr << "Decision maker infrastructure has no road map!" << std::endl;
    return;
  }

  latest_traffic_participant_set.remove_old_participants( max_participant_age, now().seconds() );


  if( latest_traffic_participant_set.participants.empty() )
    return;

  update_routes_for_participants();

  dynamics::TrafficParticipantSet planned;
  switch( planner_backend )
  {
    case PlannerBackend::MultiAgentPid:
      planned = plan_with_pid();
      break;
    case PlannerBackend::MultiAgentPlanner:
      planned = plan_with_multi_agent_planner();
      break;
  }

  publisher_planned_traffic->publish( planned );
}

void
DecisionMakerInfrastructure::create_subscribers()
{
  update_dynamic_subscriptions();
  main_timer = create_wall_timer( 100ms, std::bind( &DecisionMakerInfrastructure::run, this ) );
}

void
DecisionMakerInfrastructure::create_publishers()
{
  publisher_planned_traffic         = create_publisher<ParticipantSetAdapter>( "/infrastructure/planned_traffic", 1 );
  publisher_infrastructure_position = create_publisher<adore_ros2_msgs::msg::VisualizableObject>( "infrastructure_position", 1 );
  publisher_infrastructure_info     = create_publisher<adore_ros2_msgs::msg::InfrastructureInfo>( "infrastructure_info", 1 );

  if ( should_publish_local_map )
  {
    publisher_local_map = create_publisher<MapAdapter>( "local_map", 1 );
  }
}

void
DecisionMakerInfrastructure::load_parameters()
{
  traffic_participant_in_topic = declare_parameter<std::string>( "traffic_participant_in_topic", "/traffic_participant" );

  should_publish_local_map = declare_parameter<bool>("should_publish_local_map", false);

  dt = declare_parameter<double>( "dt", 0.1 );

  std::string map_file_location = declare_parameter<std::string>( "map file", "" );
  road_map                      = std::make_shared<map::Map>( map::MapLoader::load_from_file( map_file_location ) );

  infrastructure_pose.x   = declare_parameter<double>( "infrastructure_position_x", 0.0 );
  infrastructure_pose.y   = declare_parameter<double>( "infrastructure_position_y", 0.0 );
  infrastructure_pose.yaw = declare_parameter<double>( "infrastructure_yaw", 0.0 );

  max_participant_age = declare_parameter<double>( "max_participant_age", 0.5 );

  // New route-related parameters (with reasonable defaults)
  max_route_length  = declare_parameter<double>( "max_route_length", 200.0 );
  route_replan_dist = declare_parameter<double>( "route_replan_dist", 5.0 );
  local_map_size    = declare_parameter<double>( "local_map_size", 50.0 );

  debug = declare_parameter<bool>( "debug", false );

  // Planner backend selection
  const std::string planner_backend_str = declare_parameter<std::string>( "planner_backend", "multi_agent_pid" );

  if( planner_backend_str == "multi_agent_pid" )
  {
    planner_backend = PlannerBackend::MultiAgentPid;
  }
  else if( planner_backend_str == "multi_agent_planner" )
  {
    planner_backend = PlannerBackend::MultiAgentPlanner;
  }
  else
  {
    RCLCPP_WARN( get_logger(), "Unknown planner_backend '%s', falling back to 'multi_agent_pid'.", planner_backend_str.c_str() );
    planner_backend = PlannerBackend::MultiAgentPlanner;
  }

  tf_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( *this );

  // Validity area polygon from flat vector<double> [x0,y0,x1,y1,...]
  std::vector<double> validity_area_points;
  validity_area_points = declare_parameter<std::vector<double>>( "validity_polygon", validity_area_points );

  if( validity_area_points.size() >= 6 )
  {
    math::Polygon2d validity_area{ validity_area_points };
    latest_traffic_participant_set.validity_area = validity_area;
  }
}

void
DecisionMakerInfrastructure::print_init_info()
{
  RCLCPP_INFO( get_logger(), "DecisionMakerInfrastructure node initialized." );
}

void
DecisionMakerInfrastructure::print_debug_info()
{
  const double current_time_seconds = now().seconds();

  RCLCPP_DEBUG( get_logger(), "------- Decision Maker Infrastructure Debug Information -------" );
  RCLCPP_DEBUG( get_logger(), "Current Time: %.3f seconds", current_time_seconds );

  if( road_map )
  {
    RCLCPP_DEBUG( get_logger(), "Local map data available." );
  }
  else
  {
    RCLCPP_DEBUG( get_logger(), "No local map data." );
  }

  RCLCPP_DEBUG( get_logger(), "Traffic participants: %zu", latest_traffic_participant_set.participants.size() );
  RCLCPP_DEBUG( get_logger(), "------- ============================== -------" );
}

void
DecisionMakerInfrastructure::publish_infrastructure_position()
{
  adore_ros2_msgs::msg::VisualizableObject obj;
  obj.x               = infrastructure_pose.x;
  obj.y               = infrastructure_pose.y;
  obj.yaw             = infrastructure_pose.yaw;
  obj.z               = 0.0;
  obj.model           = "low_poly_trailer_model.dae";
  obj.header.frame_id = "world";

  publisher_infrastructure_position->publish( obj );

  adore_ros2_msgs::msg::InfrastructureInfo infrastructure_info_msg;
  infrastructure_info_msg.position_x = infrastructure_pose.x;
  infrastructure_info_msg.position_y = infrastructure_pose.y;
  infrastructure_info_msg.yaw        = infrastructure_pose.yaw;

  adore_ros2_msgs::msg::Polygon2d validity_area_msg;
  if( latest_traffic_participant_set.validity_area.has_value() )
  {
    for( const auto& point : latest_traffic_participant_set.validity_area->points )
    {
      adore_ros2_msgs::msg::Point2d p;
      p.x = point.x;
      p.y = point.y;
      validity_area_msg.points.push_back( p );
    }

    infrastructure_info_msg.validity_area = validity_area_msg;
  }

  publisher_infrastructure_info->publish( infrastructure_info_msg );
}

void
DecisionMakerInfrastructure::traffic_participant_callback( const dynamics::TrafficParticipant& msg, const std::string& vehicle_namespace )
{
  latest_traffic_participant_set.update_traffic_participants( msg );
}

void
DecisionMakerInfrastructure::update_dynamic_subscriptions()
{
  auto topic_names_and_types = get_topic_names_and_types();

  const std::string topic_name    = "v2x_traffic_participant";
  const std::string escaped_topic = std::regex_replace( topic_name, std::regex( R"([.^$|()\\[\]{}*+?])" ), R"(\$&)" );

  std::regex valid_topic_regex( "^/([^/]+)/" + escaped_topic + "$" );
  std::regex valid_type_regex( R"(^adore_ros2_msgs/msg/TrafficParticipant$)" );

  for( const auto& topic : topic_names_and_types )
  {
    const std::string&              found_topic_name = topic.first;
    const std::vector<std::string>& types            = topic.second;

    std::smatch match;
    if( std::regex_match( found_topic_name, match, valid_topic_regex )
        && std::any_of( types.begin(), types.end(),
                        [&]( const std::string& type ) { return std::regex_match( type, valid_type_regex ); } ) )
    {
      std::string vehicle_namespace = match[1].str();

      // Already subscribed?
      if( traffic_participant_subscribers.count( vehicle_namespace ) > 0 )
      {
        continue;
      }

      auto subscription = create_subscription<ParticipantAdapter>( found_topic_name, 10,
                                                                   [this, vehicle_namespace]( const dynamics::TrafficParticipant& msg ) {
                                                                     traffic_participant_callback( msg, vehicle_namespace );
                                                                   } );

      traffic_participant_subscribers[vehicle_namespace] = subscription;

      RCLCPP_INFO( get_logger(), "Subscribed to new vehicle namespace: %s", vehicle_namespace.c_str() );
    }
  }
}

void DecisionMakerInfrastructure::publish_infrastructure_transform()
{
  dynamics::VehicleStateDynamic infrastructure_state;
  infrastructure_state.x = infrastructure_pose.x;
  infrastructure_state.y = infrastructure_pose.y;
  infrastructure_state.yaw_angle = infrastructure_pose.yaw;

  auto infrastructure_frame = dynamics::conversions::vehicle_state_to_transform( infrastructure_state, now(), get_namespace() );
  tf_transform_broadcaster->sendTransform( infrastructure_frame );
}

} // namespace adore

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::DecisionMakerInfrastructure )
