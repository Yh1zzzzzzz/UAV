#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
float normalHeight = 0.7;
class GoalNode {
public:
    float position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w;

    GoalNode(float x, float y, float z = normalHeight, float x1 = 0 ,float y1 = 0, float z1 = 0, float w1 = 1,0) {
        this->position_x = x;
        this->position_y = y;
        this->position_z = z;
        this->orientation_x = x1;
        this->orientation_y = y1;
        this->orientation_z = z1;
        this->orientation_w = w1;
    }
};

class PX4Controller {
private:
    ros::NodeHandle nh;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    mavros_msgs::State current_state;
// 回调函数用于储存当前状态
    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }

public:
    PX4Controller() {
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &PX4Controller::state_cb, this);
    }

    void arm() {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
        } else {
            ROS_ERROR("Arming failed");
        }
    }

    void setOffboardMode() {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
            ROS_INFO("Offboard enabled");
        } else {
            ROS_ERROR("Offboard mode failed");
        }
    }

    void sendMoveCommand(float x, float y, float z, float x1, float y1, float z1, float w1) {
        geometry_msgs::PoseStamped goal;
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation.x = x1;
        goal.pose.orientation.y = y1;
        goal.pose.orientation.z = z1;
        goal.pose.orientation.w = w1;
        local_pos_pub.publish(pose);
    }

    mavros_msgs::State getCurrentState() {
        return current_state;
    }
};

class DroneController {
private:
    PX4Controller px4Controller;

public:
    void moveToGoal(const GoalNode& goal) {
        px4Controller.sendMoveCommand(goal.x, goal.y, goal.z, goal.orientation_x, goal.orientation_y, goal.orientation_z, goal.orientation_w);
    }

    void armDrone() {
        px4Controller.arm();
    }

    void enableOffboardMode() {
        px4Controller.setOffboardMode();
    }

    mavros_msgs::State getCurrentState() {
        return px4Controller.getCurrentState();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "testNode");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    DroneController droneController;

    // Wait for FCU connection
    while (ros::ok() && droneController.getCurrentState().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Create a goal
    GoalNode goal(10.0, 5.0, 3.0);

    // Arm and set to offboard mode
    droneController.armDrone();

    //这中间需要加等待

    droneController.enableOffboardMode();

    // Send initial setpoints before starting offboard mode
    for (int i = 0; i < 100; ++i) {
        droneController.moveToGoal(goal);
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        droneController.moveToGoal(goal);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
