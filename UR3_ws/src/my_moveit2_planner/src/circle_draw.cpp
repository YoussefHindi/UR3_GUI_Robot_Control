#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

  double x_center_h = 0.28; //(x)
	double y_center_k = 0.35;  //(y)
	double radius_r = 0.02;  //radius

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

bool limit_check(double x, double y, double r)
{
  double hypotenuse = sqrt(fabs((x*x)+(y*y)));
  if (hypotenuse >0.45)
  {
    RCLCPP_ERROR(LOGGER, "robot centre ouside of limits: [x: %f, y: %f, hypotenuse: %f ]", x, y, hypotenuse);
    return false;
  }
  else if (((r+x)>0.45)||((r+y)>0.45)||((y-r)<-0.45)||((x-r)<-0.45))
  {
    RCLCPP_ERROR(LOGGER, "circle radius above robots limits: [x radius: %f, y radius: %f, -ve x: %f, -ve y: %f]", x+r, y+r, x-r, y-r);
    RCLCPP_ERROR(LOGGER, "robots max limits limits: %f", 0.45);
    return false;
  }
  else 
  {
    RCLCPP_WARN(LOGGER, "robot centre within limits: [x: %f, y: %f]", x, y);
    return true;
  }
}

int main(int argc, char **argv) {

	rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  move_group_arm.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

	//circle equation (x-h)2+(y-k)2 =r2

  // robot x1 limits 0.125<x<0.45
  // robot x2 limits -0.125<x<-0.45

  // robot y limits 0.125<y<0.125
  // robot y limits -0.125<y<-0.45

  // y limits < sqrt(x2+y2)<0.45

  // h limits 
  // k limits 

  // robot z limits 0.15<z<
  // max z where x & y limits move freely  = 0.1

  /*
  steps 

  -set centre of the circle in x and its range from 0.125 -> 0.45
  or from -0.125 -> -0.45 
  -based on x set calculate the y 
  -based on the 2 values calculate the radius r

  eg. x= 0.28
  
  using this eq we calculate max y:
    sqrt(fabs(0.45(2)-0.28(2)))= y(max value)

    y(max value) = 0.35 

  then we calculate max r 

    r+x < 0.45 
    r=0.45-x
    AND
    r+y < 0.45
    r=0.45-y

    THEN TAKE THE MIN R

  then you are done 

  */
	
	
 	
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;

  // Home Position

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = (x_center_h-radius_r);
  target_pose1.position.y = (y_center_k);
  target_pose1.position.z = 0.1;
  move_group_arm.setPoseTarget(target_pose1);

  bool limit = limit_check(x_center_h, y_center_k, radius_r);
  if (!limit)
  {
    rclcpp::shutdown();
    return 0;
  }


move_group_arm.startStateMonitor();

moveit::core::RobotStatePtr current_state = move_group_arm.getCurrentState();
current_state->update();

  geometry_msgs::msg::PoseStamped current_pose = move_group_arm.getCurrentPose();

  RCLCPP_INFO(LOGGER, "End Effector current Position: [x: %f, y: %f, z: %f]", 
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

// Print the position and orientation
if(current_pose.pose.position.x == target_pose1.position.x && current_pose.pose.position.y == target_pose1.position.y && current_pose.pose.position.z == target_pose1.position.z){
    
}
else {
    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

     move_group_arm.execute(my_plan_arm);
  rclcpp::sleep_for(std::chrono::seconds(2));

 current_pose = move_group_arm.getCurrentPose();
}

RCLCPP_INFO(LOGGER, "End Effector AFTER MOVING TO HOME Position: [x: %f, y: %f, z: %f]", 
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
	
	// (k-r)<y<(k+r)	range of y
	// (h-r) <x< (h+r) range of x

	// y=k(+-)sqrt(r2-(x-h)2)
	// x=h(+-)sqrt(r2-(y-k)2)

	// Upper semi-circle 
  	std::vector<geometry_msgs::msg::Pose> circle_waypoints;
    moveit_msgs::msg::RobotTrajectory circle_trajectory1;

		for (double i=(x_center_h-radius_r); i < (x_center_h+radius_r)+eef_step; i+=eef_step)
		{
			double x=i;
			double y= (y_center_k)+sqrt(fabs((radius_r*radius_r)-((x-x_center_h)*(x-x_center_h))));
			
			target_pose1.position.x = x;
			target_pose1.position.y = y;

      RCLCPP_INFO(LOGGER, "INCREMENTATION: [x: %f, y: %f]", 
              target_pose1.position.x,
              target_pose1.position.y);

			circle_waypoints.push_back(target_pose1);
		}

    // double fraction = move_group_arm.computeCartesianPath(
		// 		circle_waypoints, eef_step, jump_threshold, circle_trajectory1);
    // move_group_arm.execute(circle_trajectory1);

    // RCLCPP_INFO(LOGGER, "first semi circle \n ");
		//   rclcpp::sleep_for(std::chrono::seconds(1));
    
    // lower semicircle
    // std::vector<geometry_msgs::msg::Pose> circle_waypoints2;
    // moveit_msgs::msg::RobotTrajectory circle_trajectory2;
		
		for (double i=(x_center_h+radius_r); i > (x_center_h-radius_r)-eef_step; i-=eef_step)
		{
		double x=i;
		double y= (y_center_k)-sqrt(fabs((radius_r*radius_r)-((x-x_center_h)*(x-x_center_h))));
																		
		target_pose1.position.x = x;
		target_pose1.position.y = y;

    RCLCPP_INFO(LOGGER, "INCREMENTATION: [x: %f, y: %f]", 
              target_pose1.position.x,
              target_pose1.position.y);

		circle_waypoints.push_back(target_pose1);
		}

		double fraction = move_group_arm.computeCartesianPath(
				circle_waypoints, eef_step, jump_threshold, circle_trajectory1);

		move_group_arm.execute(circle_trajectory1);

    current_pose = move_group_arm.getCurrentPose();

    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(LOGGER, "End Effector AFTER CIRCLE Position: [x: %f, y: %f, z: %f]", 
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
	
	rclcpp::shutdown();
	return 0;

}