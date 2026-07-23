#include "nav2_teb_controller/teb_controller.hpp"

#include <tf2/time.h>

#include <memory>

#include "nav2_teb_controller/teb_profiler.hpp"

namespace nav2_teb_controller {

void TEBController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                              std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TEBController: failed to lock node in configure()");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare + load all parameters via generated ParamListener
  param_listener_ =
      std::make_shared<teb_controller::ParamListener>(node->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // String → Level mappen
  const std::string log_level_str = params_.FollowPath.log_level;
  const std::map<std::string, rclcpp::Logger::Level> level_map = {
      {"debug", rclcpp::Logger::Level::Debug}, {"info", rclcpp::Logger::Level::Info},
      {"warn", rclcpp::Logger::Level::Warn},   {"error", rclcpp::Logger::Level::Error},
      {"fatal", rclcpp::Logger::Level::Fatal},
  };
  const auto level = level_map.contains(log_level_str) ? level_map.at(log_level_str)
                                                       : rclcpp::Logger::Level::Info;
  logger_.set_level(level);
  rclcpp::get_logger("optimal_planner").set_level(level);

  // Tricycle steering angle subscription
  tricycle_state_sub_ = node->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/tricycle_state", rclcpp::QoS(1).best_effort(),
      [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        last_ackermann_cmd_ = *msg;
      });

  // Init Costmap converter
  intra_proc_node_ = std::make_shared<rclcpp::Node>("costmap_converter", node->get_namespace(),
                                                    rclcpp::NodeOptions());
  initCostmapConverter();

  // All sub-systems take const ref into params_.FollowPath — no copying
  // Footprint
  bool use_local_costmap = params_.FollowPath.robot.footprint.use_local_costmap;
  std::string model = params_.FollowPath.robot.footprint.model;
  std::string points = params_.FollowPath.robot.footprint.points;
  footprint_ = Footprint(use_local_costmap, model, points);
  RCLCPP_INFO(logger_, "%s", footprint_.toString().c_str());
  // ESDF
  const double esdf_hz = params_.FollowPath.obstacles.costmap_converter_rate;
  esdf_update_period_ = rclcpp::Duration::from_seconds(1.0 / esdf_hz);
  // Visualization
  const double visu_hz = params_.FollowPath.visualization.publish_rate;
  visualize_update_period_ = rclcpp::Duration::from_seconds(1.0 / visu_hz);
  // Planner
  if (params_.FollowPath.hcp.activate) {
    RCLCPP_ERROR(logger_,
                 "TEBController: Homotopy class planning is NOT implemented yet. "
                 "Disable 'hcp.activate' — falling back to direct planner is not possible either, "
                 "the HCP placeholder will fail at plan() time.");
    auto p = std::make_shared<DiscreteTEBPlanner>(params_, footprint_, costmap_ros_.get());
    auto gs = std::make_shared<VisibilityGraphSearch>();
    auto hcp = std::make_unique<HomotopyClassPlanner>(params_, footprint_, costmap_ros_.get());
    hcp->setBasePlanner(p);
    hcp->setGraphSearch(gs);
    hcp->setObstacleMap(&esdf_);
    teb_planner_ = hcp.get();   // PlannerInterface
    planner_ = std::move(hcp);  // PlannerBase
  } else {
    auto p = std::make_unique<DiscreteTEBPlanner>(params_, footprint_, costmap_ros_.get());
    p->setObstacleMap(&esdf_);
    teb_planner_ = p.get();   // PlannerInterface
    planner_ = std::move(p);  // PlannerBase
  }
  // Visu
  visualizer_ = std::make_unique<TEBVisualizer>(node);
  visualizer_->on_configure();

  // Path handler + band controller
  path_handler_ = std::make_unique<PathHandler>(params_, *tf_);
  // TODO: reintroduce a createBandController factory once stanley/lyapunov controllers land.
  if (params_.FollowPath.path_tracker.type == "feedforward") {
    band_controller_ = std::make_unique<FeedForwardController>();
  } else {
    RCLCPP_WARN(logger_, "Unsupported path_tracker.type '%s'; falling back to 'feedforward'.",
                params_.FollowPath.path_tracker.type.c_str());
    band_controller_ = std::make_unique<FeedForwardController>();
  }
  band_controller_->configure(params_);
}

void TEBController::activate() {
  visualizer_->on_activate();
  RCLCPP_INFO(logger_, "TEBController activated");
}

void TEBController::deactivate() {
  RCLCPP_INFO(logger_, "TEBController deactivated");
}

void TEBController::cleanup() {
  RCLCPP_INFO(logger_, "TEBController cleaned up");
}

void TEBController::setPlan(const nav_msgs::msg::Path &path) {
  global_plan_ = path;
  planner_->clear();
  RCLCPP_INFO(logger_, "New global plan received (%zu poses)", path.poses.size());
  RCLCPP_INFO(logger_, "Force re-init.");
}

geometry_msgs::msg::TwistStamped TEBController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * /*goal_checker*/) {
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getGlobalFrameID();
  auto goal_pose = global_plan_.poses.back();
  // Report the completed profiling window at the top of the next tick so that every
  // block (including "total") of all 100 ticks has been fully recorded (off-by-one fix:
  // previously the report fired while the current tick's "total" block was still alive).
  {
    const std::string profile_report = PROFILE_REPORT();
    if (!profile_report.empty()) {
      RCLCPP_INFO(logger_, "[PROFILE]\n%s", profile_report.c_str());
    }
  }
  PROFILE_TICK();
  PROFILE_BLOCK("total");

  // 1. Refresh dynamic parameters
  if (param_listener_->is_old(params_))
    params_ = param_listener_->get_params();  // later, needs_full_rebuild_ = true;

  // 2. Extract local lookahead window
  nav_msgs::msg::Path transformed_plan;
  int goal_idx = 0;
  {
    PROFILE_BLOCK("prune_trim");
    if (!path_handler_->prepareLocalPlan(
            global_plan_, pose, velocity, clock_->now(), *costmap_ros_->getCostmap(),
            costmap_ros_->getGlobalFrameID(), transformed_plan, goal_idx)) {
      return cmd_vel;
    }
  }
  {
    const auto &back = transformed_plan.poses.empty() ? geometry_msgs::msg::PoseStamped{}
                                                      : transformed_plan.poses.back();
    const auto &gf =
        global_plan_.poses.empty() ? geometry_msgs::msg::PoseStamped{} : global_plan_.poses.back();
    RCLCPP_DEBUG(logger_,
                 "[TEBController] step2: transformed_plan=%zu poses, emitted goal_idx=%d, "
                 "global_plan_=%zu poses, transformed.back=(%.2f,%.2f) global_final=(%.2f,%.2f)",
                 transformed_plan.poses.size(), goal_idx, global_plan_.poses.size(),
                 back.pose.position.x, back.pose.position.y, gf.pose.position.x,
                 gf.pose.position.y);
  }
  if (transformed_plan.poses.empty()) {
    RCLCPP_INFO(logger_, "TEBController: Empty plan.");
    return cmd_vel;
  }

  // Update obstacles
  costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obstacles_ptr;
  {
    PROFILE_BLOCK("obstacles");
    if (costmap_converter_)
      obstacles_ptr = costmap_converter_->getObstacles();
    teb_planner_->updateObstacleContainer(obstacles_ptr);
  }
  // Update ESDF
  {
    PROFILE_BLOCK("esdf_update");
    const rclcpp::Time now = clock_->now();
    if ((now - last_esdf_update_) >= esdf_update_period_) {
      std::unique_lock lock(*costmap_ros_->getCostmap()->getMutex());
      esdf_.update(*costmap_ros_->getCostmap());
      last_esdf_update_ = now;
    }
  }

  // update via-points container
  // updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);

  // check if we should enter any backup mode and apply settings
  // configureBackupModes(transformed_plan, goal_idx);

  // 3. Plan, wart start or reinit
  bool success;
  {
    PROFILE_BLOCK("planner_plan");
    bool final_goal = (goal_idx == ((int)global_plan_.poses.size() - 1)) ||
                      params_.FollowPath.optimizer.fix_goal;
    RCLCPP_DEBUG(logger_,
                 "[TEBController] step3: goal_idx=%d global_plan_=%zu final_goal=%d "
                 "(fix_goal=%d, idx_eq_size_minus1=%d)",
                 goal_idx, global_plan_.poses.size(), (int)final_goal,
                 (int)params_.FollowPath.optimizer.fix_goal,
                 (int)(goal_idx == ((int)global_plan_.poses.size() - 1)));
    teb_planner_->setFixedGoal(final_goal);
    teb_planner_->setFeedback(last_ackermann_cmd_.drive);
    success = planner_->plan(transformed_plan, velocity);
  }
  if (!success || planner_->hasDiverged()) {
    planner_->clear();
    RCLCPP_INFO(logger_, "TEBController: Planner failed.");
    return cmd_vel;
  }

  // 4. Get TEB
  const auto &teb = teb_planner_->getTEB();

  // 5. Check for collision
  int index;
  {
    PROFILE_BLOCK("feasibility");
    const double feasibility_check = params_.FollowPath.obstacles.feasibility_check;
    index = checkFeasibility(teb, esdf_, footprint_, feasibility_check);
  }
  const bool stop_cmd = index >= 0;

  // 5. Visualize (throttled to the configured publish rate)
  {
    PROFILE_BLOCK("visualize");
    const rclcpp::Time now = clock_->now();
    if ((now - last_visualize_time_) >= visualize_update_period_) {
      last_visualize_time_ = now;
      const std::string frame_id = costmap_ros_->getGlobalFrameID();
      visualizer_->publishLocalPlan(teb, frame_id);
      visualizer_->publishLookaheadPlan(transformed_plan);
      visualizer_->publishTEBPoses(teb, frame_id);
      visualizer_->publishObstacles(obstacles_ptr, frame_id);
      visualizer_->publishCurvatureRadii(teb, frame_id);
      visualizer_->publishFootprint(teb.pose(std::max(index, 0)), footprint_, frame_id);
    }
  }

  if (params_.FollowPath.hcp.activate) {
    if (auto *hcp = dynamic_cast<const HomotopyClassPlanner *>(planner_.get())) {
      if (auto *vgs =
              dynamic_cast<const VisibilityGraphSearch *>(hcp->getGraphSearch())) {
        visualizer_->publishVisibilityGraph(vgs->getVisibilityGraph(), frame_id);
      }
      visualizer_->publishHCPCandidates(hcp->getCandidates(), frame_id);
    }
  }

  // 6. Get velocity command
  {
    PROFILE_BLOCK("velocity_output");
    const bool activate = params_.FollowPath.path_tracker.activate;
    if (activate && !stop_cmd) {
      const double dt = (clock_->now() - last_cmd_vel_.header.stamp).nanoseconds() / 1e9;
      cmd_vel.twist = band_controller_->computeCommand(
          teb, pose, velocity, last_ackermann_cmd_.drive.steering_angle, dt);
    }
    last_cmd_vel_ = cmd_vel;
  }

  return cmd_vel;
}

void TEBController::setSpeedLimit(const double &speed_limit, const bool &percentage) {
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;

  RCLCPP_DEBUG(logger_, "TEBController speed limit set to %.2f (%s)", speed_limit_,
               speed_limit_is_percentage_ ? "percentage" : "absolute");
}

void TEBController::initCostmapConverter() {
  std::string odom_topic = "odom";  // node_.get_parameter();
  std::string plugin_name = params_.FollowPath.obstacles.costmap_converter_plugin;
  double update_freq = params_.FollowPath.obstacles.costmap_converter_rate;
  bool extra_thread = params_.FollowPath.obstacles.costmap_converter_spin_thread;
  if (plugin_name.empty()) {
    RCLCPP_INFO(logger_, "No costmap converter plugin specified. "
                         "All occupied costmap cells are treated as point obstacles.");
    return;
  }
  try {
    auto *costmap = costmap_ros_->getCostmap();
    costmap_converter_ = costmap_converter_loader_.createSharedInstance(plugin_name);
    std::string converter_name = costmap_converter_loader_.getName(plugin_name);
    RCLCPP_INFO(logger_, "library path : %s",
                costmap_converter_loader_.getClassLibraryPath(plugin_name).c_str());
    std::replace(converter_name.begin(), converter_name.end(), ':', '/');

    costmap_converter_->setOdomTopic(odom_topic);
    costmap_converter_->initialize(intra_proc_node_);
    costmap_converter_->setCostmap2D(costmap);
    const auto rate = std::make_shared<rclcpp::Rate>(update_freq);
    costmap_converter_->startWorker(rate, costmap, extra_thread);
    RCLCPP_INFO(logger_, "Costmap conversion plugin %s loaded.", plugin_name.c_str());
  } catch (pluginlib::PluginlibException &ex) {
    RCLCPP_INFO(logger_,
                "The specified costmap converter plugin cannot be loaded. Error message: %s",
                ex.what());
    costmap_converter_.reset();
  }
}

}  // namespace nav2_teb_controller
