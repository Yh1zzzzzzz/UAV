#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandTOL.h>
#include <vector>
#include <cmath>
#include<string.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <wiringPi.h>

float normalHeight = 0.7;
double roll = 0.0;
double pitch = 0.0;
double yaw = 3.1415926;
class GoalNode {
public:
    float position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w;
    geometry_msgs::PoseStamped selfpose;
    bool reverse;
    bool stay;
    GoalNode(float x, float y, float z = normalHeight, bool reversep = false, bool s = true, float x1 = 0, float y1 = 0, float z1 = 0, float w1 = 1) {
        this->position_x = x;
        this->position_y = y;
        this->position_z = z;
        this->orientation_x = x1;
        this->orientation_y = y1;
        this->orientation_z = z1;
        this->orientation_w = w1;
        this->reverse = reversep;
        this->stay = s;
        selfpose.pose.position.x = x;
        selfpose.pose.position.y = y;
        selfpose.pose.position.z = z;
        selfpose.pose.orientation.x = x1;
        selfpose.pose.orientation.y = y1;
        selfpose.pose.orientation.z = z1;
        selfpose.pose.orientation.w = w1;
    }
};

class PX4Controller {
private:
    ros::NodeHandle nh;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient takeoff_cl;
    ros::Subscriber state_sub;
    ros::Subscriber position_sub;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_position;

    void state_cb(const mavros_msgs::State::ConstPtr &msg) {
        current_state = *msg;
    }

    void position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        current_position = *msg;
    }

public:
    PX4Controller(ros::NodeHandle &nh) : nh(nh) {
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &PX4Controller::state_cb, this);
        position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10,
                                                                &PX4Controller::position_cb, this);
        takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
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

    void disarm() {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = false;
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle disarmed");
        } else {
            ROS_ERROR("Disarming failed");
        }
    }

    void takeoff() {
        mavros_msgs::CommandTOL srv_takeoff;
        srv_takeoff.request.altitude = 0.8;
        srv_takeoff.request.latitude = 0;
        srv_takeoff.request.longitude = 0;
        srv_takeoff.request.min_pitch = 0;
        srv_takeoff.request.yaw = 0;
        if (takeoff_cl.call(srv_takeoff)) {
            ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
        } else {
            ROS_ERROR("Failed Takeoff");
        }
    }
//
    void setGuidedMode() {
        mavros_msgs::SetMode guided_set_mode;
        guided_set_mode.request.base_mode = 0;
        guided_set_mode.request.custom_mode = "OFFBOARD";
        if (set_mode_client.call(guided_set_mode) && guided_set_mode.response.mode_sent) {
            ROS_INFO("Guided enabled");
        } else {
            ROS_ERROR("Guided mode failed");
        }
    }

    void setLandMode() {
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";
        if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
            ROS_INFO("Land mode enabled");
        } else {
            ROS_ERROR("Land mode failed");
        }
    }

    void sendMoveCommand(float x, float y, float z, float x1, float y1, float z1, float w1) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation.x = x1;
        goal.pose.orientation.y = y1;
        goal.pose.orientation.z = z1;
        goal.pose.orientation.w = w1;
        local_pos_pub.publish(goal);
    }

    mavros_msgs::State getCurrentState() {
        return current_state;
    }

    geometry_msgs::PoseStamped getCurrentPosition() {
        return current_position;
    }
};

class DroneController {
public:
    PX4Controller px4Controller;
    std::vector<GoalNode> waypoints;
    int current_waypoint_index;
    int num_goal;
    tf2::Quaternion quaternion;
public:
    DroneController(ros::NodeHandle &nh) : px4Controller(nh), current_waypoint_index(0) {

    quaternion.setRPY(roll, pitch, yaw);
    quaternion = quaternion.normalize();
    }

    geometry_msgs::PoseStamped GetCurrentPos() {
        geometry_msgs::PoseStamped current_position = px4Controller.getCurrentPosition();
        return current_position;
    }

    void setWaypoints(const std::vector<GoalNode> &new_waypoints) {
        waypoints = new_waypoints;
        num_goal = waypoints.size();
    }
    void prehandle() {
        int size = waypoints.size();
        for (int i = 0; i < size; i +=1) {
            if(waypoints[i].reverse) {
                    geometry_msgs::PoseStamped tempPose = waypoints[i].selfpose;
                    tempPose.pose.orientation = tf2::toMsg(quaternion);
                    waypoints[i].selfpose.orientation = tempPose.pose.orientation;
                    waypoints[i].orientation_x =  tempPose.pose.orientation.x;
                    waypoints[i].orientation_y = tempPose.pose.orientation.y;
                    waypoints[i].orientation_z = tempPose.pose.orientation.z;
                    waypoints[i].orientation_w = tempPose.pose.orientation.w;
                
            }
        }
    
    }
    void moveToNextGoal() {
        if (waypoints.empty()) return;

         GoalNode goal = waypoints[current_waypoint_index];
        geometry_msgs::PoseStamped current_position = px4Controller.getCurrentPosition();

        // 检查无人机是否到达目标点
        if (std::abs(current_position.pose.position.x - goal.position_x) < 0.15 &&
            std::abs(current_position.pose.position.y - goal.position_y) < 0.15 &&
            std::abs(current_position.pose.position.z - goal.position_z) < 0.10)
             {

            //1秒时间  照亮二维码
                ros::Time lastRequest2 = ros::Time::now();
                while(ros::Time::now() - lastRequest2 < ros::Duration(1.0) && goal.stay) {
                    px4Controller.sendMoveCommand(goal.position_x, goal.position_y, goal.position_z,
                                       goal.orientation_x, goal.orientation_y, goal.orientation_z,
                                       goal.orientation_w);
                    ros::spinOnce();
                    ros::Duration(0.01).sleep();
                    //digitalWrite(1, LOW);

                 }
                //digitalWrite(1, HIGH);

                //停留3s
                ros::Time lastRequest3 = ros::Time::now();
                while(ros::Time::now() - lastRequest3 < ros::Duration(3.0) && ) {
                    px4Controller.sendMoveCommand(goal.position_x, goal.position_y, goal.position_z,
                                       goal.orientation_x, goal.orientation_y, goal.orientation_z,
                                       goal.orientation_w);
                    ros::spinOnce();
                    ros::Duration(0.01).sleep();
                }

            // 到达目标点，更新到下一个目标点
            current_waypoint_index = (current_waypoint_index + 1) % waypoints.size();
            num_goal -= 1;
            ROS_INFO("Reach Goal Node");
             }

        // 发送移动指令
         GoalNode next_goal = waypoints[current_waypoint_index];
        if (next_goal.reverse)
        {
                geometry_msgs::PoseStamped tempPose2= goal.selfpose;
                tempPose2.pose.orientation = tf2::toMsg(quaternion);
                next_goal.orientation_x =  tempPose2.pose.orientation.x;
                next_goal.orientation_y = tempPose2.pose.orientation.y;
                next_goal.orientation_z = tempPose2.pose.orientation.z;
                next_goal.orientation_w = tempPose2.pose.orientation.w;
        }

        px4Controller.sendMoveCommand(next_goal.selfpose.pose.position.x, next_goal.selfpose.pose.position.y, next_goal.selfpose.pose.position.z,
                                      next_goal.selfpose.pose.orientation.x, next_goal.selfpose.pose.orientation.y,
                                      next_goal.selfpose.pose.orientation.z,
                                      next_goal.selfpose.pose.orientation.w);
    }

    bool IsGoalCompleted() {
        return num_goal <= 0;
    }

    void armDrone() {
        px4Controller.arm();
    }

    void takeOff() {
        px4Controller.takeoff();
    }

    void disarmDrone() {
        px4Controller.disarm();
    }

    void enableGuidedMode() {
        px4Controller.setGuidedMode();
    }

    mavros_msgs::State getCurrentState() {
        return px4Controller.getCurrentState();
    }

    void enableLandMode() {
        px4Controller.setLandMode();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "testNode");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);
    //pinMode(1, OUTPUT);

    DroneController droneController(nh);
    //等待飞控链接
    while (ros::ok() && !droneController.getCurrentState().connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected.");
	//digitalWrite(1, LOW);
    // 飞行轨迹——2X3矩形
    //    GoalNode(float x, float y, float z = normalHeight, bool reversep = false, bool s = true
    // X Y Z  是否反向  是否打开激光
    std::vector<GoalNode> waypoints = {
        GoalNode(0, 0, 1.4, false, true),
        GoalNode(-1, 0, 1.4, false, true),
        GoalNode(-1, 0, 0.7, false, true),
        GoalNode(-1.5, 0, 0.7, false, true),
        GoalNode(-1.5, 0, 1.4, false, true),
        GoalNode(-2, 0, 1.4, false, true),
        GoalNode(-2, 0, 0.7, false, true),
        GoalNode(-2.5, 0, 0.7, false, true),
        GoalNode(-2.5, 1.2, 0.7, false, true),
        GoalNode(-2, 1.2, 0.7, true, true),
        GoalNode(-2, 1.2, 1.4, true, true),
        GoalNode(-1.5, 1.2, 1.4, true, true),
        GoalNode(-1.5, 1.2, 0.7, true, true)

        //GoalNode(0, 0, 1.5),
        //GoalNode(0, 0, 1.5)
        //GoalNode(0, 3),
        //GoalNode(2, 3),
        //GoalNode(2, 0)
    };
    droneController.prehandle;
//
    int i = waypoints.size();

    droneController.setWaypoints(waypoints);

        for (int i = 1; i < 100; i ++) {
        droneController.moveToNextGoal();
        ros::spinOnce();
        rate.sleep();
    }
    droneController.enableGuidedMode();
    ROS_INFO("enableGuidedMode");
    // while (ros::ok() && droneController.getCurrentState().mode != "GUIDED") {
    //     ROS_INFO("Waiting for GUIDED mode...");
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    droneController.armDrone();
    ROS_INFO("armDrone");


    // while (ros::ok() && !droneController.getCurrentState().armed) {
    //     ROS_INFO("Waiting for arming...");
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    //
    //droneController.takeOff();
    // sleep(10);
    //设置期望点
    ROS_INFO("takeOff");

    while (ros::ok() && !droneController.IsGoalCompleted()) {
        droneController.moveToNextGoal();
        ros::spinOnce();
        rate.sleep();
        //ROS_INFO("fly to goal Node");
    }
    ROS_INFO("start 5s sleep");
    // ros::Duration(5.0).sleep();
    // ROS_INFO("end 5s sleep");
    ////
    // while (ros::ok() && !droneController.IsGoalCompleted()) {
    //     droneController.moveToNextGoal();
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    //
///
    //结束之后等待20s飞回起始点降落

    // ros::Duration(20.0).sleep();
    // //开始降落
    droneController.enableLandMode();
    ROS_INFO("enableLandMode");


    // while (ros::ok() && droneController.GetCurrentPos().pose.position.z > 0.3) {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // // Disarm the drone after landing
    // droneController.disarmDrone();
    return 0;
}s
