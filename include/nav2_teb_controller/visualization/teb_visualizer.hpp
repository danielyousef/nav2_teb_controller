#pragma once

#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_teb_controller/core/footprint.hpp"
#include "nav2_teb_controller/core/teb_utils.hpp"
#include "nav2_teb_controller/core/timed_elastic_band.hpp"
#include "nav2_teb_controller/homotopy/teb_candidate.hpp"
#include "nav2_teb_controller/homotopy/visibility_graph.hpp"

namespace nav2_teb_controller {

class TEBVisualizer {
public:
  explicit TEBVisualizer(rclcpp_lifecycle::LifecycleNode::SharedPtr node) : node_(node) {}

  /// @brief Publishes the local plan (TEB as nav_msgs::Path)
  void publishLocalPlan(const TimedElasticBand &teb, const std::string &frame_id) {
    if (local_plan_pub_->get_subscription_count() == 0)
      return;

    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = frame_id;

    for (std::size_t i = 0; i < teb.sizePoses(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = teb.pose(i).x();
      ps.pose.position.y = teb.pose(i).y();
      tf2::Quaternion q;
      q.setRPY(0, 0, teb.pose(i).theta());
      ps.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(ps);
    }
    local_plan_pub_->publish(path);
  }

  /// @brief Publishes the current lookahead window of the global plan
  void publishLookaheadPlan(const nav_msgs::msg::Path &transformed_plan) {
    if (lookahead_plan_pub_->get_subscription_count() > 0)
      lookahead_plan_pub_->publish(transformed_plan);
    if (lookahead_goal_pub_->get_subscription_count() > 0)
      lookahead_goal_pub_->publish(transformed_plan.poses.back());
  }

  /// @brief Publishes TEB poses as PoseArray
  void publishTEBPoses(const TimedElasticBand &teb, const std::string &frame_id) {
    if (teb_poses_pub_->get_subscription_count() == 0)
      return;

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = node_->now();
    pose_array.header.frame_id = frame_id;

    for (std::size_t i = 0; i < teb.sizePoses(); ++i) {
      geometry_msgs::msg::Pose p;
      p.position.x = teb.pose(i).x();
      p.position.y = teb.pose(i).y();
      tf2::Quaternion q;
      q.setRPY(0, 0, teb.pose(i).theta());
      p.orientation = tf2::toMsg(q);
      pose_array.poses.push_back(p);
    }
    teb_poses_pub_->publish(pose_array);
  }

  /// @brief Publishes obstacles as MarkerArray from costmap_converter ObstacleArrayMsg
  void publishObstacles(costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obstacles,
                        const std::string &frame_id) {
    if (!obstacles)
      return;
    if (obstacles_pub_->get_subscription_count() == 0)
      return;
    if (obstacles->obstacles.empty())
      return;

    visualization_msgs::msg::MarkerArray marker_array;
    const auto stamp = node_->now();

    int id = 0;
    for (const auto &obs : obstacles->obstacles) {
      if (obs.polygon.points.empty())
        continue;

      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id;
      m.ns = "teb_obstacles";
      m.id = id++;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.lifetime = rclcpp::Duration::from_seconds(0.5);
      m.pose.position.z = 0.1;
      m.pose.orientation.w = 1.0;

      if (obs.polygon.points.size() == 1) {
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.pose.position.x = (double)obs.polygon.points[0].x;
        m.pose.position.y = (double)obs.polygon.points[0].y;
        m.scale.x = 0.1;
        m.scale.y = 0.1;
        m.scale.z = 0.2;
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 0.8;
      } else {
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = 0.05;
        m.color.r = 1.0;
        m.color.g = 0.5;
        m.color.b = 0.0;
        m.color.a = 0.8;
        for (const auto &pt : obs.polygon.points) {
          geometry_msgs::msg::Point p;
          p.x = (double)pt.x;
          p.y = (double)pt.y;
          p.z = 0.1;
          m.points.push_back(p);
        }
        if (!m.points.empty())
          m.points.push_back(m.points.front());
      }
      marker_array.markers.push_back(m);
    }
    obstacles_pub_->publish(marker_array);
  }

  void publishCurvatureRadii(const TimedElasticBand &teb, const std::string &frame_id) {
    if (radius_markers_pub_->get_subscription_count() == 0)
      return;

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.clear();  // Clear alte Marker

    // Parameter für saubere Darstellung
    constexpr double kappa_min = 0.1;       // R > 20m → keine Anzeige
    constexpr double radius_max_viz = 5.0;  // Max Pfeillänge 5m
    constexpr double R_min_robot = 0.0;     // Engste Kurve Robot
    constexpr double arrow_scale = 0.2;     // Pfeil = 12% Radius

    for (size_t i = 1; i < teb.sizePoses() - 1; ++i) {
      const PoseSE2 &p_before = teb.pose(i - 1);
      const PoseSE2 &pose_mid = teb.pose(i);
      const PoseSE2 &p_after = teb.pose(i + 1);

      // 1. κ berechnen
      double kappa = computeCurvature(p_before, pose_mid, p_after);

      // 2. Filter: Gerade Segmente ausblenden
      if (fabs(kappa) < kappa_min)
        continue;

      // 3. Radius limitieren
      double radius = std::clamp(1.0 / fabs(kappa), R_min_robot, radius_max_viz);

      // 4. Radius-Markierung (Kamm-Pfeil)

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = node_->now();
      marker.ns = "teb_radii";
      marker.id = static_cast<int>(i);
      // marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0.1);  // Kurzlebig

      // Position am Mittelpunkt, leicht erhöht
      marker.pose.position.x = pose_mid.x();
      marker.pose.position.y = pose_mid.y();
      marker.pose.position.z = radius * arrow_scale;  // Mitte des Zylinders
      // marker.pose.position.z = 0.15;
      marker.pose.orientation.w = 1.0;

      // Marker Orientierung
      // double mid_heading = pose_mid.theta();
      // double arrow_yaw = mid_heading + M_PI / 2.0;  // 90° nach rechts
      // if (kappa > 0) arrow_yaw -= M_PI;  // Links kurven → Pfeil nach innen
      // tf2::Quaternion quat;
      // quat.setRPY(0, 0, arrow_yaw);
      // marker.pose.orientation = tf2::toMsg(quat);
      // Orientierung irrelevant für Zylinder
      marker.pose.orientation.w = 1.0;

      // Pfeil-Größe proportional zum Radius
      // marker.scale.x = radius * arrow_scale;  // Länge
      // marker.scale.y = 0.04;                  // Breite
      // marker.scale.z = 0.04;                  // Höhe
      // Höhe = 2 * Radius (von -R bis +R)
      marker.scale.z = 2.0 * radius * arrow_scale;  // 10% Skalierung
      marker.scale.x = 0.08;                        // Dicke
      marker.scale.y = 0.08;

      // Farbe: Rot=eng (gefährlich), Grün=weit (sicher)
      double hue = (radius - R_min_robot) / (radius_max_viz - R_min_robot);
      marker.color.r = 1.0 - hue;
      marker.color.g = hue;
      marker.color.b = 0.0;
      marker.color.a = 0.85;

      marker_array.markers.push_back(marker);
    }
    radius_markers_pub_->publish(marker_array);
  }

  void publishFootprint(const PoseSE2 &current_pose, const Footprint &footprint,
                        const std::string &frame_id) {
    if (footprint_markers_pub_->get_subscription_count() == 0)
      return;

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.clear();

    const double px = current_pose.x();
    const double py = current_pose.y();
    const double ct = std::cos(current_pose.theta());
    const double st = std::sin(current_pose.theta());

    const auto &circles = footprint.circles();
    for (int i = 0; i < static_cast<int>(circles.size()); ++i) {
      const auto &c = circles[i];
      const double wx = px + ct * c.offset.x() - st * c.offset.y();
      const double wy = py + st * c.offset.x() + ct * c.offset.y();

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = node_->now();
      marker.ns = "teb_footprint";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0.1);
      marker.pose.position.x = wx;
      marker.pose.position.y = wy;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0 * c.radius;
      marker.scale.y = 2.0 * c.radius;
      marker.scale.z = 0.05;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.4f;
      marker_array.markers.push_back(marker);
    }
    footprint_markers_pub_->publish(marker_array);
  }

  /// @brief Publish visibility graph nodes and edges for RViz debugging
  void publishVisibilityGraph(const VisibilityGraph &graph, const std::string &frame_id) {
    if (vis_graph_pub_->get_subscription_count() == 0)
      return;

    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = node_->now();

    const auto &nodes = graph.nodes();
    const auto &obstacle_map = graph.obstacleMap();

    // Edges as LINE_LIST
    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = frame_id;
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "vis_graph_edges";
    edge_marker.id = 0;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    edge_marker.scale.x = 0.02;
    edge_marker.color.r = 0.3f;
    edge_marker.color.g = 0.6f;
    edge_marker.color.b = 1.0f;
    edge_marker.color.a = 0.5f;

    std::set<std::pair<int,int>> added_edges;
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
      for (const auto &e : graph.edges(i)) {
        auto key = std::make_pair(std::min(e.from_id, e.to_id), std::max(e.from_id, e.to_id));
        if (added_edges.insert(key).second) {
          geometry_msgs::msg::Point p1, p2;
          p1.x = nodes[e.from_id].pos.x();
          p1.y = nodes[e.from_id].pos.y();
          p2.x = nodes[e.to_id].pos.x();
          p2.y = nodes[e.to_id].pos.y();
          edge_marker.points.push_back(p1);
          edge_marker.points.push_back(p2);
        }
      }
    }
    markers.markers.push_back(edge_marker);

    // Nodes as SPHERE markers
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = stamp;
      marker.ns = "vis_graph_nodes";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker.pose.position.x = nodes[i].pos.x();
      marker.pose.position.y = nodes[i].pos.y();
      marker.pose.position.z = 0.05;
      marker.pose.orientation.w = 1.0;

      if (nodes[i].is_start) {
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
      } else if (nodes[i].is_goal) {
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
      } else {
        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;
      }
      markers.markers.push_back(marker);
    }

    // If ESDF is available, overlay a distance contour
    if (obstacle_map && obstacle_map->isInitialized()) {
      visualization_msgs::msg::Marker grid_marker;
      grid_marker.header.frame_id = frame_id;
      grid_marker.header.stamp = stamp;
      grid_marker.ns = "vis_graph_esdf";
      grid_marker.id = 0;
      grid_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      grid_marker.action = visualization_msgs::msg::Marker::ADD;
      grid_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      grid_marker.scale.x = obstacle_map->resolution();
      grid_marker.scale.y = obstacle_map->resolution();
      grid_marker.scale.z = 0.01;

      const double ox = obstacle_map->originX();
      const double oy = obstacle_map->originY();
      const double res = obstacle_map->resolution();
      const int step = std::max(1, static_cast<int>(res > 0 ? 0.2 / res : 4));

      for (unsigned y = 0; y < obstacle_map->sizeY(); y += step) {
        for (unsigned x = 0; x < obstacle_map->sizeX(); x += step) {
          double wx = ox + (x + 0.5) * res;
          double wy = oy + (y + 0.5) * res;
          auto q = obstacle_map->query(wx, wy);
          if (q.distance > 0.5 || q.distance < -0.1)
            continue;

          geometry_msgs::msg::Point pt;
          pt.x = wx;
          pt.y = wy;
          pt.z = 0.0;
          grid_marker.points.push_back(pt);

          std_msgs::msg::ColorRGBA color;
          if (q.distance < 0.0) {
            color.r = 1.0f;
            color.g = 0.0f;
            color.b = 0.0f;
            color.a = 0.6f;
          } else {
            double t = q.distance / 0.5;
            color.r = static_cast<float>(t);
            color.g = static_cast<float>(1.0 - t);
            color.b = 0.0f;
            color.a = 0.3f;
          }
          grid_marker.colors.push_back(color);
        }
      }
      markers.markers.push_back(grid_marker);
    }

    vis_graph_pub_->publish(markers);
  }

  /// @brief Publish all HCP candidate paths in different colors
  void publishHCPCandidates(const std::vector<TebCandidate::Ptr> &candidates,
                            const std::string &frame_id) {
    if (hcp_candidates_pub_->get_subscription_count() == 0)
      return;

    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = node_->now();

    for (size_t i = 0; i < candidates.size(); ++i) {
      const auto &teb = candidates[i]->teb;
      if (teb.sizePoses() < 2)
        continue;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = stamp;
      marker.ns = "hcp_candidates";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker.scale.x = 0.04;
      marker.pose.orientation.w = 1.0;

      // Color by index along HSV
      double hue = static_cast<double>(i) / std::max(candidates.size(), size_t(1));
      marker.color.r = static_cast<float>(sin(hue * 2.0 * M_PI) * 0.5 + 0.5);
      marker.color.g = static_cast<float>(sin((hue + 1.0/3.0) * 2.0 * M_PI) * 0.5 + 0.5);
      marker.color.b = static_cast<float>(sin((hue + 2.0/3.0) * 2.0 * M_PI) * 0.5 + 0.5);
      marker.color.a = candidates[i]->is_feasible ? 0.9f : 0.3f;

      for (size_t j = 0; j < teb.sizePoses(); ++j) {
        geometry_msgs::msg::Point pt;
        pt.x = teb.pose(j).x();
        pt.y = teb.pose(j).y();
        marker.points.push_back(pt);
      }
      markers.markers.push_back(marker);
    }
    hcp_candidates_pub_->publish(markers);
  }

  nav2_util::CallbackReturn on_configure() {
    local_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
    lookahead_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("teb_lookahead_plan", 1);
    lookahead_goal_pub_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>("teb_lookahead_goal", 1);
    teb_poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("teb_poses", 1);
    obstacles_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>("teb_obstacles", 1);
    radius_markers_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>("teb_radius_markers", 1);
    footprint_markers_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>("footprint_markers", 1);
    vis_graph_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>("vis_graph", 1);
    hcp_candidates_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>("hcp_candidates", 1);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate() {
    local_plan_pub_->on_activate();
    lookahead_plan_pub_->on_activate();
    lookahead_goal_pub_->on_activate();
    teb_poses_pub_->on_activate();
    obstacles_pub_->on_activate();
    radius_markers_pub_->on_activate();
    footprint_markers_pub_->on_activate();
    vis_graph_pub_->on_activate();
    hcp_candidates_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr lookahead_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      lookahead_goal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr teb_poses_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      obstacles_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      radius_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      footprint_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      vis_graph_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      hcp_candidates_pub_;
};

}  // namespace nav2_teb_controller
