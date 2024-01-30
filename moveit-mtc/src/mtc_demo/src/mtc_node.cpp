// ROS2 functionality
#include <rclcpp/rclcpp.hpp>

// Functionality to interface with the robot model and collision objects
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Components of the MoveIt Task Constructor that are used in the example
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

// Not used in the basic example, later used for pose generation
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

// Get a logger for the new node
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

// Create a namespace alias 
namespace mtc = moveit::task_constructor;

// Main class
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

// Class Constructor - initialices the node
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

// Getter to get the base interface
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// Class Method - Setting up the planning scene
void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

// Class Method - Interfaces with the MoveIt Task Constructor task object
void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  // Publish solution to be visualized in RViz
  task_.introspection().publishSolution(*task_.solutions().front());

  // Excute the plan - action server interface plugin with Rviz
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

// Class Method - Creates a MoveIt Task Constructor object and set some properties
mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  // Make a stage (generator stage) and add to our task
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // Define the type of robot motor

  //PipelinePlanner - defaults to OMPL
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

  //JoingInterpolation - simpler planner that interpolates from start to goal, used for simple motions
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  //CartesianPath - used to move the end effector in a straight line in Cartesian space - USED IN THIS EXAMPLE
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // TASK DEFINITION //

  // Make a stage (propagator stage) and add to our task
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // Make a stage (connector stage) and add to our task
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // Pointer to a MTC stage object
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

  // Define a SerialContainer to hold all the PICKING stages
  {
  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" }); // properties from the parent task
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" }); // initialite properties

  // Make an stage (propagator stage) and add to our Container
  {
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
  stage->properties().set("marker_ns", "approach_object");
  stage->properties().set("link", hand_frame);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.15);

  // Set hand forward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame;
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
  }

  {
  // Sample grasp pose
  auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  stage->setPreGraspPose("open");
  stage->setObject("object");
  stage->setAngleDelta(M_PI / 12);
  stage->setMonitoredStage(current_state_ptr);  // Hook into current state

  // Define the transform using Eigen transformation matrix
  Eigen::Isometry3d grasp_frame_transform;
  Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  grasp_frame_transform.linear() = q.matrix();
  grasp_frame_transform.translation().z() = 0.1;

  // Compute IK
  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  grasp->insert(std::move(wrapper));
  }

  {
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
  stage->allowCollisions("object",
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        true);
  grasp->insert(std::move(stage));
  }

  {
  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("close");
  grasp->insert(std::move(stage));
  }

  {
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage->attachObject("object", hand_frame);
  attach_object_stage = stage.get();
  grasp->insert(std::move(stage));
  }

  {
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "lift_object");

  // Set upward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
  }

  task.add(std::move(grasp));
  }

  {
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                { hand_group_name, interpolation_planner } });
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));
  }

  {
  auto place = std::make_unique<mtc::SerialContainer>("place object");
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

  {
  // Sample place pose
  auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "place_pose");
  stage->setObject("object");

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "object";
  target_pose_msg.pose.position.y = 0.5;
  target_pose_msg.pose.orientation.w = 1.0;
  stage->setPose(target_pose_msg);
  stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

  // Compute IK
  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(2);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame("object");
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  place->insert(std::move(wrapper));
  }

  {
  auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("open");
  place->insert(std::move(stage));
  }

  {
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
  stage->allowCollisions("object",
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        false);
  place->insert(std::move(stage));
  }

  {
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage->detachObject("object", hand_frame);
  place->insert(std::move(stage));
  }

  {
  auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "retreat");

  // Set retreat direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.x = -0.5;
  stage->setDirection(vec);
  place->insert(std::move(stage));
  }

  task.add(std::move(place));
  }

  {
  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setGoal("ready");
  task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  // Initialize the ROS 2 context with command line arguments
  rclcpp::init(argc, argv);

  // Create NodeOptions to configure the behavior of the ROS 2 node
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // Create an instance of the MTCTaskNode class using the provided options
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // Create a MultiThreadedExecutor for handling multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;

  // Create a separate thread for spinning the executor
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    // Add the MTCTaskNode to the executor
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    
    // Spin (process callbacks) until shutdown is called
    executor.spin();

    // Remove the MTCTaskNode from the executor when spinning is done
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Call custom setup method on the MTCTaskNode
  mtc_task_node->setupPlanningScene();

  // Call another custom method on the MTCTaskNode
  mtc_task_node->doTask();

  // Wait for the spinning thread to finish
  spin_thread->join();

  // Shutdown the ROS 2 context, releasing resources and cleaning up
  rclcpp::shutdown();

  // Return 0 to indicate successful execution
  return 0;
}