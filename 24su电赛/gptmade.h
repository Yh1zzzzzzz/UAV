#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

class GoalNode {
public:
    float x, y, z;

    GoalNode(float x, float y, float z) : x(x), y(y), z(z) {}
};

class KDTree {
private:
    struct Node {
        GoalNode goal;
        std::shared_ptr<Node> left;
        std::shared_ptr<Node> right;

        Node(const GoalNode& goal) : goal(goal), left(nullptr), right(nullptr) {}
    };

    std::shared_ptr<Node> root;

    std::shared_ptr<Node> insertRec(std::shared_ptr<Node> node, const GoalNode& goal, int depth) {
        if (!node) return std::make_shared<Node>(goal);

        int axis = depth % 3;
        if (axis == 0) {
            if (goal.x < node->goal.x)
                node->left = insertRec(node->left, goal, depth + 1);
            else
                node->right = insertRec(node->right, goal, depth + 1);
        } else if (axis == 1) {
            if (goal.y < node->goal.y)
                node->left = insertRec(node->left, goal, depth + 1);
            else
                node->right = insertRec(node->right, goal, depth + 1);
        } else {
            if (goal.z < node->goal.z)
                node->left = insertRec(node->left, goal, depth + 1);
            else
                node->right = insertRec(node->right, goal, depth + 1);
        }

        return node;
    }

    void nearestRec(std::shared_ptr<Node> node, const GoalNode& target, int depth, GoalNode& best, float& bestDist) const {
        if (!node) return;

        float dist = std::sqrt(std::pow(node->goal.x - target.x, 2) +
                               std::pow(node->goal.y - target.y, 2) +
                               std::pow(node->goal.z - target.z, 2));

        if (dist < bestDist) {
            bestDist = dist;
            best = node->goal;
        }

        int axis = depth % 3;
        std::shared_ptr<Node> nextBranch = nullptr;
        std::shared_ptr<Node> oppositeBranch = nullptr;

        if (axis == 0) {
            if (target.x < node->goal.x) {
                nextBranch = node->left;
                oppositeBranch = node->right;
            } else {
                nextBranch = node->right;
                oppositeBranch = node->left;
            }
        } else if (axis == 1) {
            if (target.y < node->goal.y) {
                nextBranch = node->left;
                oppositeBranch = node->right;
            } else {
                nextBranch = node->right;
                oppositeBranch = node->left;
            }
        } else {
            if (target.z < node->goal.z) {
                nextBranch = node->left;
                oppositeBranch = node->right;
            } else {
                nextBranch = node->right;
                oppositeBranch = node->left;
            }
        }

        nearestRec(nextBranch, target, depth + 1, best, bestDist);

        if ((axis == 0 && std::fabs(target.x - node->goal.x) < bestDist) ||
            (axis == 1 && std::fabs(target.y - node->goal.y) < bestDist) ||
            (axis == 2 && std::fabs(target.z - node->goal.z) < bestDist)) {
            nearestRec(oppositeBranch, target, depth + 1, best, bestDist);
        }
    }

public:
    KDTree() : root(nullptr) {}

    void insert(const GoalNode& goal) {
        root = insertRec(root, goal, 0);
    }

    GoalNode nearest(const GoalNode& target) const {
        if (!root) throw std::runtime_error("KDTree is empty");

        GoalNode best = root->goal;
        float bestDist = std::numeric_limits<float>::max();
        nearestRec(root, target, 0, best, bestDist);
        return best;
    }
};

class PX4Controller {
private:
    ros::NodeHandle nh;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    mavros_msgs::State current_state;

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

    void sendMoveCommand(float x, float y, float z) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        local_pos_pub.publish(pose);
    }

    mavros_msgs::State getCurrentState() {
        return current_state;
    }
};

class DroneController {
private:
    PX4Controller px4Controller;
    std::vector<GoalNode> goalList;
    KDTree kdTree;

public:
    void addGoalNode(const GoalNode& goal) {
        goalList.push_back(goal);
        kdTree.insert(goal);
    }

    void moveToGoal(const GoalNode& goal) {
        px4Controller.sendMoveCommand(goal.x, goal.y, goal.z);
    }

    void moveToNearestGoal(const GoalNode& current) {
        GoalNode nearest = kdTree.nearest(current);
        moveToGoal(nearest);
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
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    DroneController droneController;

    // Wait for FCU connection
    while (ros::ok() && !droneController.getCurrentState().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Add goal nodes to the KDTree
    droneController.addGoalNode(GoalNode(10.0, 5.0, 3.0));
    droneController.addGoalNode(GoalNode(20.0, 10.0, 5.0));
    droneController.addGoalNode(GoalNode(5.0, 3.0, 2.0));

    // Arm and set to offboard mode
    droneController.armDrone();
    droneController.enableOffboardMode();

    // Send initial setpoints before starting offboard mode
    for (int i = 0; i < 100; ++i) {
        droneController.moveToNearestGoal(GoalNode(0.0, 0.0, 0.0));
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        droneController.moveToNearestGoal(GoalNode(0.0, 0.0, 0.0));
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
//
// Created by 18051 on 24-7-16.
//

#ifndef GPTMADE_H
#define GPTMADE_H



class gptmade {

};



#endif //GPTMADE_H
