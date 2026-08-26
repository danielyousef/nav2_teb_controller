#pragma once

#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <nav2_teb_controller/obstacles/esdf.hpp>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_teb_controller {
class PlannerBase {
public:
  virtual ~PlannerBase() = default;
  virtual bool plan(const nav_msgs::msg::Path &, const geometry_msgs::msg::Twist &) = 0;
  virtual bool hasDiverged() = 0;
  virtual void clear() = 0;
};

template <typename TEBType>
class PlannerInterface : public PlannerBase {
public:
  [[nodiscard]] virtual const TEBType &getTEB() const = 0;
  [[nodiscard]] virtual double getCost() const = 0;
  /// Summed chi2 over the efficiency-category edges (time optimal + shortest path +
  /// smoothness). Defaults to the total cost for planners without a category split.
  [[nodiscard]] virtual double getEfficiencyCost() const { return getCost(); }
  virtual void setFeedback(const ackermann_msgs::msg::AckermannDrive &feedback) = 0;
  virtual void updateObstacleContainer(
      costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obstacle_array = nullptr) = 0;
  virtual void setObstacleMap(const ObstacleMap2D *esdf) = 0;
  virtual void setFixedGoal(bool fix) = 0;
};
}  // namespace nav2_teb_controller