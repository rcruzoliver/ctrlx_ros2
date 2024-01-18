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

  // Make a stage (propagator stage) and add to our task
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

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