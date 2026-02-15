#include "MyNode.h"
#include <QProcess>
#include <iostream>
#include <cstdlib>  // For system()

MyNode::MyNode(QObject *parent)
    : QObject(parent)
{
}

void MyNode::startRosNode()
{
 
       const char* command = "bash -c 'cd ../../../UR3_ws && source install/setup.bash && ros2 launch my_moveit2_planner draw_rectangle.launch.py'";


    // Execute the command
    int result = system(command);

    // Check the result
    if (result == 0) {
        std::cout << "Command executed successfully." << std::endl;
    } else {
        std::cerr << "Command execution failed." << std::endl;
    }
}

void MyNode::makeCircle()
{
		const char* command = "bash -c 'cd ../../../UR3_ws && source install/setup.bash && ros2 launch my_moveit2_planner draw_circle.launch.py'";
		// Execute the command
    int result = system(command);

    // Check the result
    if (result == 0) {
        std::cout << "Command executed successfully." << std::endl;
    } else {
        std::cerr << "Command execution failed." << std::endl;
    }
}

