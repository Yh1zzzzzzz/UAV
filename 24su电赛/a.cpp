/***************************************************************************************************************************
* terminal_control.cpp
*
* Author: Qyp
*
* Update Time: 2020.11.4
*
* Introduction:  test function for sending ControlCommand.msg
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>

#include <serial/serial.h>
#include <std_msgs/String.h>

#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

#include "controller_test.h"
#include "KeyboardEvent.h"
#include <center.h> // 导入自定义消息类型

#define VEL_XY_STEP_SIZE 0.1
#define VEL_Z_STEP_SIZE 0.1
#define YAW_STEP_SIZE 0.08
#define TRA_WINDOW 2000
#define NODE_NAME "terminal_control"

using namespace std;

//即将发布的command
prometheus_msgs::ControlCommand Command_to_pub;
//轨迹容器
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
geometry_msgs::PoseStamped goal;
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;    // 无人机状态
//Eigen::Matrix3f R_Body_to_ENU;              // 无人机机体系至惯性系转换矩阵   为什么注释掉！！！！！

//---------------------------------------Track---------------------------------------------
float kp_land[3] = {2,2,2};         //控制参数 - 比例参数
landing::center landmark;

/*--------------------------------------回调函数--------------------------------------------*/
void center_cb(const landing::center::ConstPtr &msg)
{
    landmark = *msg;
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    //R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);
}


float time_trajectory = 0.0;
// 轨迹追踪总时长，键盘控制时固定时长，指令输入控制可调
float trajectory_total_time = 50.0;

//发布
ros::Publisher move_pub;   //旧发布指令
ros::Publisher ref_trajectory_pub;
ros::Publisher goal_pub ;

//函数功能：将数据经由串口发送出去
//入口参数1：[serial::Serial &ser]：     串口类名称
//入口参数2：[std::string &serial_msg]:  要通过串口发送出去的字符串
int serial_write(serial::Serial &ser, std::string &serial_msg)
{
    ser.write(serial_msg);
    return 0;
}

//函数功能：将从串口接收到的数据保存到数组中
//入口参数1：[serial::Serial &ser]：     串口类名称
//入口参数2：[std::string &serial_msg]:  从串口读取的字符串
int serial_read(serial::Serial &ser, std::string &serial_msg)
{
    serial_msg = ser.read(ser.available());
    return 0;
}
////解锁前输入校准量，全局变量
double celi_pre_x[4] ={0,0,0,0};
double celi_pre_y[4]= {0,0,0,0};
double blk_celi_pre_x[6] ={0,0,0,0,0,0};
double blk_celi_pre_y[6]= {0,0,0,0,0,0};
double door_celi_pre_x[2] ={0,0};
double door_celi_pre_y[2]= {0,0};

/*--------投放目标点，全局变量----------
0：第一个投放点
1：第二个投放点
2：第三个投放点
3：降落点
*/
double num_x[4] = {0,     0.5,    1.5,     2.5};
double num_y[4] = {0,     1.8,    2.6,     3.1};

/*---------五个投放点+降落点，全局变量----------
0：第一个投放点
1：第二个投放点
2：第三个投放点
3：第四个投放点
4：第五个投放点
5：降落点
*/
double point_x[6]={0,0,0,0,0,0};
double point_y[6]={0,0,0,0,0,0};

//最后穿走廊的点
 double  landmark_for_corridor_x[6]={0,0,0,0,0,0};
 double  landmark_for_corridor_y[6]={0,0,0,0,0,0};

//实例化一个serial类
serial::Serial ser;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void mainloop1();
void mainloop2();
void move_circle(int n, double center_x, double center_y);
void generate_com(int Move_mode, float state_desired[4]);
void Draw_in_rviz(const prometheus_msgs::PositionReference &pos_ref, bool draw_trajectory);
void timerCallback(const ros::TimerEvent &e)
{
    cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<" << endl;
    cout << "ENTER key to control the drone: " << endl;
    cout << "1 for Arm, Space for Takeoff, L for Land, H for Hold, 0 for Disarm, 8/9 for Trajectory tracking" << endl;
    cout << "Move mode is fixed (XYZ_VEL,BODY_FRAME): w/s for body_x, a/d for body_y, k/m for z, q/e for body_yaw" << endl;
    cout << "CTRL-C to quit." << endl;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "terminal_control");
    ros::NodeHandle nh;

    //　【发布】　控制指令
    move_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    //    【发布】 目标点发送指令
    goal_pub =  nh.advertise<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 10);
    //　【发布】　参考轨迹
    ref_trajectory_pub = nh.advertise<nav_msgs::Path>("/prometheus/reference_trajectory", 10);

    ros::Subscriber center_sub = nh.subscribe<landing::center>("center", 10, center_cb);

    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //初始化串口相关设置serial
    ser.setPort("/dev/ttyUSB0");                               //设置打开的串口名称:这里打开一个虚拟串口
    ser.setBaudrate(9600);                                     //设置串口的波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); //创建timeout
    ser.setTimeout(to);                                        //设置串口的timeout

    //打开串口serial
    try
    {
        ser.open(); //打开串口
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Unable to open port."); //打开串口失败，打印日志信息，然后结束程序
        return -1;
    }

    //判断串口是否成功打开serial
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port is opened.\n"); //成功打开串口，打印日志信息
    }
    else
    {
        return -1; //打开串口失败，打印日志信息，然后结束程序
    }

    ros::Rate loop_rate(50); //指定循环频率50

    //用于控制器测试的类，功能例如：生成圆形轨迹，８字轨迹等
    Controller_Test Controller_Test; // 打印参数
    Controller_Test.printf_param();

    // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
    Command_to_pub.Command_ID = 0;
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
    Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_to_pub.Reference_State.position_ref[0] = 0;
    Command_to_pub.Reference_State.position_ref[1] = 0;
    Command_to_pub.Reference_State.position_ref[2] = 0;
    Command_to_pub.Reference_State.velocity_ref[0] = 0;
    Command_to_pub.Reference_State.velocity_ref[1] = 0;
    Command_to_pub.Reference_State.velocity_ref[2] = 0;
    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;
    Command_to_pub.Reference_State.yaw_ref = 0;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    //cout.setf(ios::showpos);


    // 选择通过终端输入控制或键盘控制
    int Remote_Mode;
    cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<" << endl;
    cout << "Please choose the Remote Mode: 0 for command input control, 1 for keyboard input control" << endl;
    cin >> Remote_Mode;



    if (Remote_Mode == 0)
    {
        cout << "Command input control mode" << endl;
        mainloop1();
    }
    else if (Remote_Mode == 1)
    {
        ros::Timer timer = nh.createTimer(ros::Duration(30.0), timerCallback);
        cout << "Keyboard input control mode" << endl;
        mainloop2();
    }

    return 0;
}

void mainloop1()
{
    int Control_Mode = 0;
    int Move_mode = 0;
    int Move_frame = 0;
    int Move_num = 0;
    int Trjectory_mode = 0;
    float state_desired[4];
    Controller_Test Controller_Test;
    double goal_x = 0;
    double goal_y1 = 0;
    double goal_y2 = 0;
    double celi_x[4] ={0,0,0,0};
    double celi_y[4]= {0,0,0,0};
    double blk_celi_x[6] ={-0.7,-1.4,-1.4,-1.4,0,0};
    double blk_celi_y[6]= {0.2,0.4,1.5,2,0,0};
    double door_celi_x[2] ={0,0};
    double door_celi_y[2]= {0,0};
    double goalone_x = 0;
    double goalone_y = 0;
    double goaltwo_x = 0;
    double goaltwo_y = 0;
    double err_x;
    double err_y;
    double celi_err_x;
    double celi_err_y;
    double last_err_x;
    double last_err_y;
    double now_err_x = 1;
    double now_err_y = 1;
    double now_err_z = 1.2;
    int time_delay = 1;
    uint8_t ifeverfind = 0;
    double findpositionx,findpositiony;

    double routes_x[10][4] = {
	{point_x[0],   point_x[1],    point_x[2],    point_x[5]},
	{point_x[0],   point_x[1],    point_x[3],    point_x[5]},
	{point_x[0],   point_x[1],    point_x[4],    point_x[5]},
	{point_x[0],   point_x[2],    point_x[3],    point_x[5]},
	{point_x[0],   point_x[2],    point_x[4],    point_x[5]},
	{point_x[0],   point_x[3],    point_x[4],    point_x[5]},
	{point_x[1],   point_x[2],    point_x[3],    point_x[5]},
	{point_x[1],   point_x[2],    point_x[4],    point_x[5]},
	{point_x[1],   point_x[3],    point_x[4],    point_x[5]},
	{point_x[2],   point_x[3],    point_x[4],    point_x[5]}};
    double routes_y[10][4] = {
	{point_y[0],   point_y[1],    point_y[2],    point_y[5]},
	{point_y[0],   point_y[1],    point_y[3],    point_y[5]},
	{point_y[0],   point_y[1],    point_y[4],    point_y[5]},
	{point_y[0],   point_y[2],    point_y[3],    point_y[5]},
	{point_y[0],   point_y[2],    point_y[4],    point_y[5]},
	{point_y[0],   point_y[3],    point_y[4],    point_y[5]},
	{point_y[1],   point_y[2],    point_y[3],    point_y[5]},
	{point_y[1],   point_y[2],    point_y[4],    point_y[5]},
	{point_y[1],   point_y[3],    point_y[4],    point_y[5]},
	{point_y[2],   point_y[3],    point_y[4],    point_y[5]}};


    int path_set;


    unsigned char all_data0[1] = {0x30};
    unsigned char all_data1[1] = {0x31};
    unsigned char all_data2[1] = {0x32};
    unsigned char all_data3[1] = {0x33};

    cout << "--------------Please choose the path-----------" <<endl;
    cout << "0: for point 1-2-3.         1: for point 1-2-4.         2: for point 1-2-5.         3: for point 1-3-4" <<endl;
    cout << "4: for point 1-3-5.         5: for point 1-4-5.         6: for point 2-3-4.         7: for point 2-3-5" <<endl;
    cout << "8: for point 2-3-4.         9: for point 3-4-5.          " <<endl;
    cin >> path_set;
     if (path_set < 0 || path_set > 9)
    {
        path_set = 0;
    }

    for (int i = 0; i < 4; ++i) {
        num_x[i] = routes_x[path_set][i];
        num_y[i] = routes_y[path_set][i];
    }

    cout << "point 1 :"<< num_x[0]<<"   "<<num_y[0]<<endl;
    cout << "point 2 :"<< num_x[1]<<"   "<<num_y[1]<<endl;
    cout << "point 3 :"<< num_x[2]<<"   "<<num_y[2]<<endl;

    while (ros::ok())
    {

        cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<" << endl;
        cout << "Please choose the Command.Mode: 0 for Idle, 1 for Takeoff, 2 for Hold, 3 for Land, 4 for Move, 5 for Disarm, 6 for User_Mode1, 7 for User_Mode2" << endl;
        cout << "Input 999 to switch to cloard mode and arm the drone (ONLY for simulation, please use RC in experiment!!!)" << endl;
        cin >> Control_Mode;

        if (Control_Mode == prometheus_msgs::ControlCommand::Move)
        {
            // cout << "Please choose the Command.Reference_State.Move_mode: 0 for XYZ_POS, 1 for XY_POS_Z_VEL, 2 for XY_VEL_Z_POS, 3 for XYZ_VEL, 5 for TRAJECTORY"<<endl;
            Move_mode = 0; //0表示xyz位置控制

            if (Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
            {
                cout << "For safety, please move the drone near to the trajectory start point firstly!!!" << endl;
                cout << "Please choose the trajectory type: 0 for Circle, 1 for Eight Shape, 2 for Step, 3 for Line" << endl;
                cin >> Trjectory_mode;
                cout << "Input the trajectory_total_time:" << endl;
                cin >> trajectory_total_time;
            }
            else
            {

                cout << "Please choose the Command for num control" << endl;

                cin >> Move_num;
                /*
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.yaw_ref = 999;
                move_pub.publish(Command_to_pub);
                Command_to_pub.Reference_State.yaw_ref = 0.0;
                */

                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub.publish(Command_to_pub);
                sleep(4);

                Move_frame = 0;

                if (Move_num == 110)
                {
                    /* 第一个投递点*/
                    state_desired[0] =  num_x[0];
                    state_desired[1] = num_y[0];
                    state_desired[2] = 1.1;
                    state_desired[3] = 0;

                    goal.header.stamp =ros::Time::now();
                    goal.header.frame_id = "map";
                    goal.pose.position.x = state_desired[0];
                    goal.pose.position.y = state_desired[1];
                    goal.pose.position.z = 0.5;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;
                    goal.pose.orientation.w = 1.0;
                    goal_pub.publish(goal);

                    cout << "moving to point_1 ..."<<endl;
	                while((abs(_DroneState.position[0] - state_desired[0]) > 0.15) || (abs(_DroneState.position[1] - state_desired[1]) > 0.15))
	                {
	                        ros::Duration(0.1).sleep();
                            ros::spinOnce();
	                }

		            ifeverfind = 0;
                    for (int i = 0; i < 50; i++)
                    {
                        if(landmark.iffind)
                        {
                            err_x = landmark.height / 2.0 - landmark.x;
                            err_y = landmark.width / 2.0 - landmark.y;
                            celi_err_x = err_x / 1400;
                            celi_err_y = err_y / 1400;

                            if (abs(err_x) < 20 && abs(err_y) < 20 && err_x != 0)
                            {
                                state_desired[0] = _DroneState.position[0] + 0.2;
                                state_desired[1] = _DroneState.position[1] + 0.06;
                                break;
                            }

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Command_ID   = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
                            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XY_VEL_Z_POS;   //xy velocity z position
                            Command_to_pub.Reference_State.velocity_ref[0] = kp_land[0] * celi_err_x;
                            Command_to_pub.Reference_State.velocity_ref[1] = kp_land[1] * celi_err_y;
                            Command_to_pub.Reference_State.position_ref[2]     = 1.1;
                            move_pub.publish(Command_to_pub);
                            //这里为什么不sleep()？？？？？？
			                ifeverfind = 1;
			                findpositionx = _DroneState.position[0];
			                findpositiony = _DroneState.position[1];
                        }

                        else if((i >= 10)&&(i < 13) && !landmark.iffind)
                        {
                            move_circle(3,_DroneState.position[0],_DroneState.position[1]);
                        }

                        else if((i > 15) && (ifeverfind == 0))//此时已经执行了盘旋？
                        {
                            cout << "target not found" << endl;
                            state_desired[2] = 1.3;
                            state_desired[3] = 0;

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Reference_State.Move_mode = Move_mode;
                            Command_to_pub.Reference_State.Move_frame = Move_frame;
                            Command_to_pub.Reference_State.time_from_start = -1;
                            generate_com(Move_mode, state_desired);
                            move_pub.publish(Command_to_pub);
                            sleep(2);
                            break;
                        }

                        else if(ifeverfind)
			            {
                            state_desired[0] = findpositionx;
                            state_desired[1] = findpositiony;
                            state_desired[2] = 1.3;
                            state_desired[3] = 0;

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Reference_State.Move_mode = Move_mode;
                            Command_to_pub.Reference_State.Move_frame = Move_frame;
                            Command_to_pub.Reference_State.time_from_start = -1;
                            generate_com(Move_mode, state_desired);
                            move_pub.publish(Command_to_pub);
			            }
                        ros::Duration(0.5).sleep();
                        ros::spinOnce();
                    }

                    state_desired[2] = 0.15;
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    Command_to_pub.Reference_State.Move_mode = Move_mode;
                    Command_to_pub.Reference_State.Move_frame = Move_frame;
                    Command_to_pub.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);
                    move_pub.publish(Command_to_pub);

                    while(_DroneState.position[2] > 0.2)
                    {
                        ros::Duration(0.05).sleep();
                        ros::spinOnce();
                    }

                    err_x = state_desired[0] - _DroneState.position[0];
                    err_y = state_desired[1] - _DroneState.position[1];

                    while((abs(err_x) > 0.1) || (abs(err_y) > 0.1))
                    {
                        Command_to_pub.header.stamp = ros::Time::now();
                        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                        Command_to_pub.source = NODE_NAME;
                        Command_to_pub.Reference_State.Move_mode = Move_mode;
                        Command_to_pub.Reference_State.Move_frame = Move_frame;
                        Command_to_pub.Reference_State.time_from_start = -1;
                        generate_com(Move_mode, state_desired);
                        move_pub.publish(Command_to_pub);
                        ros::Duration(0.05).sleep();
                        ros::spinOnce();
                        err_x = state_desired[0] - _DroneState.position[0];
                        err_y = state_desired[1] - _DroneState.position[1];
		            }

		            //ros::Duration(2).sleep();
                    ser.write(all_data1,1);
		            cout << "-------------serial working pag111----------" << endl;
                    sleep(2);

                    //返回正常高度
                    state_desired[0] = _DroneState.position[0];
                    state_desired[1] = _DroneState.position[1];
                    state_desired[2] = 1.1;
                    state_desired[3] = 0;

                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    Command_to_pub.Reference_State.Move_mode = Move_mode;
                    Command_to_pub.Reference_State.Move_frame = Move_frame;
                    Command_to_pub.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);
                    move_pub.publish(Command_to_pub);

                    sleep(4);



                    /* 第二个投递点*/
                    state_desired[0] =  num_x[1];
                    state_desired[1] = num_y[1];
                    state_desired[2] = 1.1;
                    state_desired[3] = 0;

                    goal.header.stamp =ros::Time::now();
                    goal.header.frame_id = "map";
                    goal.pose.position.x = state_desired[0];
                    goal.pose.position.y = state_desired[1];
                    goal.pose.position.z = 0.5;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;
                    goal.pose.orientation.w = 1.0;
                    goal_pub.publish(goal);

                    cout << "moving0 to point_2 ..."<<endl;
	                while((abs(_DroneState.position[0] - state_desired[0]) > 0.15) || (abs(_DroneState.position[1] - state_desired[1]) > 0.15))
	                {
	                        ros::Duration(0.1).sleep();
                            ros::spinOnce();
	                }

		            ifeverfind = 0;
                    for (int i = 0; i < 50; i++)
                    {
                        if(landmark.iffind)
                        {
                            err_x = landmark.height / 2.0 - landmark.x;
                            err_y = landmark.width / 2.0 - landmark.y;
                            celi_err_x = err_x / 1400;
                            celi_err_y = err_y / 1400;

                            if (abs(err_x) < 20 && abs(err_y) < 20 && err_x != 0)
                            {
                                state_desired[0] = _DroneState.position[0] + 0.15;
                                state_desired[1] = _DroneState.position[1] - 0.06;
                                //state_desired[0]
                                break;
                            }

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Command_ID   = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
                            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XY_VEL_Z_POS;   //xy velocity z position
                            Command_to_pub.Reference_State.velocity_ref[0] = kp_land[0] * celi_err_x;
                            Command_to_pub.Reference_State.velocity_ref[1] = kp_land[1] * celi_err_y;
                            Command_to_pub.Reference_State.position_ref[2]     = 1.1;
                            move_pub.publish(Command_to_pub);

			                ifeverfind = 1;
                            findpositionx = _DroneState.position[0];
                            findpositiony = _DroneState.position[1];
                        }

                        else if((i >= 10)&&(i < 13) && !landmark.iffind)
                        {
                            move_circle(3,_DroneState.position[0],_DroneState.position[1]);
                        }

                        else if((i > 15) && (ifeverfind == 0))
                        {
                            cout << "target not found" << endl;
                            state_desired[2] = 1.3;
                            state_desired[3] = 0;

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Reference_State.Move_mode = Move_mode;
                            Command_to_pub.Reference_State.Move_frame = Move_frame;
                            Command_to_pub.Reference_State.time_from_start = -1;
                            generate_com(Move_mode, state_desired);
                            move_pub.publish(Command_to_pub);
                            sleep(2);
                            break;
                        }

                        else if(ifeverfind)
                        {
                            state_desired[0] = findpositionx;
                            state_desired[1] = findpositiony;
                            state_desired[2] = 1.3;
                            state_desired[3] = 0;

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Reference_State.Move_mode = Move_mode;
                            Command_to_pub.Reference_State.Move_frame = Move_frame;
                            Command_to_pub.Reference_State.time_from_start = -1;
                            generate_com(Move_mode, state_desired);
                            move_pub.publish(Command_to_pub);
			            }

                        ros::Duration(0.5).sleep();
                        ros::spinOnce();
                    }

                    state_desired[2] = 0.15;
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    Command_to_pub.Reference_State.Move_mode = Move_mode;
                    Command_to_pub.Reference_State.Move_frame = Move_frame;
                    Command_to_pub.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);
                    move_pub.publish(Command_to_pub);

                    while(_DroneState.position[2] > 0.2)
                    {
                        ros::Duration(0.5).sleep();
                        ros::spinOnce();
                    }

		            err_x = state_desired[0] - _DroneState.position[0];
    		        err_y = state_desired[1] - _DroneState.position[1];

		            while((abs(err_x) > 0.1) || (abs(err_y) > 0.1))
		            {
                        Command_to_pub.header.stamp = ros::Time::now();
                        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                        Command_to_pub.source = NODE_NAME;
                        Command_to_pub.Reference_State.Move_mode = Move_mode;
                        Command_to_pub.Reference_State.Move_frame = Move_frame;
                        Command_to_pub.Reference_State.time_from_start = -1;
                        generate_com(Move_mode, state_desired);
                        move_pub.publish(Command_to_pub);
                        ros::Duration(0.05).sleep();
                        ros::spinOnce();
                        err_x = state_desired[0] - _DroneState.position[0];
                        err_y = state_desired[1] - _DroneState.position[1];
		            }
   		            //sleep(2);
                    ser.write(all_data2,1);
		            cout << "-------------serial working pag222----------" << endl;
                    sleep(2);

                    state_desired[0] = _DroneState.position[0];
                    state_desired[1] = _DroneState.position[1];
                    state_desired[2] = 1.1;
                    state_desired[3] = 0;

                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    Command_to_pub.Reference_State.Move_mode = Move_mode;
                    Command_to_pub.Reference_State.Move_frame = Move_frame;
                    Command_to_pub.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);
                    move_pub.publish(Command_to_pub);
                    ros::spinOnce();

                    sleep(4);




                    /* 第三个投递点*/
                    state_desired[0] =  num_x[2];
                    state_desired[1] = num_y[2];
                    state_desired[2] = 1.1;
                    state_desired[3] = 0;

                    goal.header.stamp =ros::Time::now();
                    goal.header.frame_id = "map";
                    goal.pose.position.x = state_desired[0];
                    goal.pose.position.y = state_desired[1];
                    goal.pose.position.z = 0.5;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;
                    goal.pose.orientation.w = 1.0;
                    goal_pub.publish(goal);

                    cout << "moving0 to point_3 ..."<<endl;
	                while((abs(_DroneState.position[0] - state_desired[0]) > 0.15) || (abs(_DroneState.position[1] - state_desired[1]) > 0.15))
	                {
	                        ros::Duration(0.1).sleep();
                            ros::spinOnce();
	                }

		            ifeverfind = 0;
                    for (int i = 0; i < 50; i++)
                    {
                        if(landmark.iffind)
                        {
                            err_x = landmark.height / 2.0 - landmark.x;
                            err_y = landmark.width / 2.0 - landmark.y;
                            celi_err_x = err_x / 1400;
                            celi_err_y = err_y / 1400;

                            if (abs(err_x) < 20 && abs(err_y) < 20 && err_x != 0)
                            {
                                state_desired[0] = _DroneState.position[0] + 0.1;
                                state_desired[1] = _DroneState.position[1] + 0.06;
                                //state_desired[0]
                                break;
                            }

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Command_ID   = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
                            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XY_VEL_Z_POS;   //xy velocity z position
                            Command_to_pub.Reference_State.velocity_ref[0] = kp_land[0] * celi_err_x;
                            Command_to_pub.Reference_State.velocity_ref[1] = kp_land[1] * celi_err_y;
                            Command_to_pub.Reference_State.position_ref[2]     = 1.1;
                            move_pub.publish(Command_to_pub);

                            ifeverfind = 1;
                            findpositionx = _DroneState.position[0];
                            findpositiony = _DroneState.position[1];
                        }

                        else if((i >= 10)&&(i < 13) && !landmark.iffind)
                        {
                            move_circle(3,_DroneState.position[0],_DroneState.position[1]);
                        }

                        else if((i > 15) && (ifeverfind == 0))
                        {
                            cout << "target not found" << endl;
                            state_desired[2] = 1.3;
                            state_desired[3] = 0;

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Reference_State.Move_mode = Move_mode;
                            Command_to_pub.Reference_State.Move_frame = Move_frame;
                            Command_to_pub.Reference_State.time_from_start = -1;
                            generate_com(Move_mode, state_desired);
                            move_pub.publish(Command_to_pub);
                            sleep(2);
                            break;
                        }

			            else if(ifeverfind)
			            {
                            state_desired[0] = findpositionx;
                            state_desired[1] = findpositiony;
                            state_desired[2] = 1.3;
                            state_desired[3] = 0;

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Reference_State.Move_mode = Move_mode;
                            Command_to_pub.Reference_State.Move_frame = Move_frame;
                            Command_to_pub.Reference_State.time_from_start = -1;
                            generate_com(Move_mode, state_desired);
                            move_pub.publish(Command_to_pub);
			            }

                        ros::Duration(0.5).sleep();
                        ros::spinOnce();
                    }

                    state_desired[2] = 0.15;
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    Command_to_pub.Reference_State.Move_mode = Move_mode;
                    Command_to_pub.Reference_State.Move_frame = Move_frame;
                    Command_to_pub.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);
                    move_pub.publish(Command_to_pub);

                    while(_DroneState.position[2] > 0.2)
                    {
                        ros::Duration(0.5).sleep();
                        ros::spinOnce();
                    }

		            err_x = state_desired[0] - _DroneState.position[0];
    		        err_y = state_desired[1] - _DroneState.position[1];
		            while((abs(err_x) > 0.1) || (abs(err_y) > 0.1))
		            {
                        Command_to_pub.header.stamp = ros::Time::now();
                        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                        Command_to_pub.source = NODE_NAME;
                        Command_to_pub.Reference_State.Move_mode = Move_mode;
                        Command_to_pub.Reference_State.Move_frame = Move_frame;
                        Command_to_pub.Reference_State.time_from_start = -1;
                        generate_com(Move_mode, state_desired);
                        move_pub.publish(Command_to_pub);
                        ros::Duration(0.05).sleep();
                        ros::spinOnce();
                        err_x = state_desired[0] - _DroneState.position[0];
                        err_y = state_desired[1] - _DroneState.position[1];
		            }
		            //sleep(2);
                    ser.write(all_data3,1);
		            cout << "-------------serial working page333----------" << endl;
                    sleep(2);

                    state_desired[0] = _DroneState.position[0];
                    state_desired[1] = _DroneState.position[1];
                    state_desired[2] = 1.1;
                    state_desired[3] = 0;

                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    Command_to_pub.Reference_State.Move_mode = Move_mode;
                    Command_to_pub.Reference_State.Move_frame = Move_frame;
                    Command_to_pub.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);
                    move_pub.publish(Command_to_pub);
                    ros::spinOnce();

                    sleep(4);

                    //------开始进行穿走廊-------------
                    //------------点1------------------
                    state_desired[0] =  landmark_for_corridor_x[0];
                    state_desired[1] = landmark_for_corridor_y[0];
                    state_desired[2] = 0.7;
                    state_desired[3] = 0;

                    goal.header.stamp =ros::Time::now();
                    goal.header.frame_id = "map";
                    goal.pose.position.x = state_desired[0];
                    goal.pose.position.y = state_desired[1];
                    goal.pose.position.z = 0.7;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;
                    goal.pose.orientation.w = 1.0;
                    goal_pub.publish(goal);

                    cout << "moving to  corridor  point_1 ..."<<endl;
	                while((abs(_DroneState.position[0] - state_desired[0]) > 0.15) || (abs(_DroneState.position[1] - state_desired[1]) > 0.15))
	                {
	                        ros::Duration(0.1).sleep();
                            ros::spinOnce();
	                }
                    //------------点2------------------
                    state_desired[0] =  landmark_for_corridor_x[1];
                    state_desired[1] = landmark_for_corridor_y[1];
                    state_desired[2] = 0.7;
                    state_desired[3] = 0;

                    goal.header.stamp =ros::Time::now();
                    goal.header.frame_id = "map";
                    goal.pose.position.x = state_desired[0];
                    goal.pose.position.y = state_desired[1];
                    goal.pose.position.z = 0.7;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;
                    goal.pose.orientation.w = 1.0;
                    goal_pub.publish(goal);

                    cout << "moving0 to corridor   point_2 ..."<<endl;
	                while((abs(_DroneState.position[0] - state_desired[0]) > 0.15) || (abs(_DroneState.position[1] - state_desired[1]) > 0.15))
	                {
	                        ros::Duration(0.1).sleep();
                            ros::spinOnce();
	                }
                    //------------点3------------------
                    state_desired[0] =  landmark_for_corridor_x[2];
                    state_desired[1] = landmark_for_corridor_y[2];
                    state_desired[2] = 0.7;
                    state_desired[3] = 0;

                    goal.header.stamp =ros::Time::now();
                    goal.header.frame_id = "map";
                    goal.pose.position.x = state_desired[0];
                    goal.pose.position.y = state_desired[1];
                    goal.pose.position.z = 0.7;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;
                    goal.pose.orientation.w = 1.0;
                    goal_pub.publish(goal);

                    cout << "moving0 to  corridor point_3 ..."<<endl;
	                while((abs(_DroneState.position[0] - state_desired[0]) > 0.15) || (abs(_DroneState.position[1] - state_desired[1]) > 0.15))
	                {
	                        ros::Duration(0.1).sleep();
                            ros::spinOnce();
	                }
                    //------------点4------------------
                    state_desired[0] =  landmark_for_corridor_x[3];
                    state_desired[1] = landmark_for_corridor_y[3];
                    state_desired[2] = 0.7;
                    state_desired[3] = 0;

                    goal.header.stamp =ros::Time::now();
                    goal.header.frame_id = "map";
                    goal.pose.position.x = state_desired[0];
                    goal.pose.position.y = state_desired[1];
                    goal.pose.position.z = 0.7;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;
                    goal.pose.orientation.w = 1.0;
                    goal_pub.publish(goal);

                    cout << "moving0 to corridor point_4 ..."<<endl;
	                while((abs(_DroneState.position[0] - state_desired[0]) > 0.15) || (abs(_DroneState.position[1] - state_desired[1]) > 0.15))
	                {
	                        ros::Duration(0.1).sleep();
                            ros::spinOnce();
	                }
                    //------------点5------------------
                    state_desired[0] =  landmark_for_corridor_x[4];
                    state_desired[1] = landmark_for_corridor_y[4];
                    state_desired[2] = 0.7;
                    state_desired[3] = 0;

                    goal.header.stamp =ros::Time::now();
                    goal.header.frame_id = "map";
                    goal.pose.position.x = state_desired[0];
                    goal.pose.position.y = state_desired[1];
                    goal.pose.position.z = 0.7;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;
                    goal.pose.orientation.w = 1.0;
                    goal_pub.publish(goal);

                    cout << "moving0 to  corridor point_5 ..."<<endl;
	                while((abs(_DroneState.position[0] - state_desired[0]) > 0.15) || (abs(_DroneState.position[1] - state_desired[1]) > 0.15))
	                {
	                        ros::Duration(0.1).sleep();
                            ros::spinOnce();
	                }
                    //------------点6 降落点------------------
                    state_desired[0] =  landmark_for_corridor_x[5];
                    state_desired[1] = landmark_for_corridor_y[5];
                    state_desired[2] = 0.7;
                    state_desired[3] = 0;

                    goal.header.stamp =ros::Time::now();
                    goal.header.frame_id = "map";
                    goal.pose.position.x = state_desired[0];
                    goal.pose.position.y = state_desired[1];
                    goal.pose.position.z = 0.7;
                    goal.pose.orientation.x = 0.0;
                    goal.pose.orientation.y = 0.0;
                    goal.pose.orientation.z = 0.0;
                    goal.pose.orientation.w = 1.0;
                    goal_pub.publish(goal);

                    cout << "moving0 to corridor point_6 ..."<<endl;
	                while((abs(_DroneState.position[0] - state_desired[0]) > 0.15) || (abs(_DroneState.position[1] - state_desired[1]) > 0.15))
	                {
	                        ros::Duration(0.1).sleep();
                            ros::spinOnce();
	                }

                    ifeverfind = 0;
                    for (int i = 0; i < 50; i++)
                    {
                        if(landmark.iffind)
                        {
                            err_x = landmark.height / 2.0 - landmark.x;
                            err_y = landmark.width / 2.0 - landmark.y;
                            celi_err_x = err_x / 1400;
                            celi_err_y = err_y / 1400;
                            if (abs(err_x) < 20 && abs(err_y) < 20 && err_x != 0)
                            {
                                state_desired[0] = _DroneState.position[0] + 0.15;
                                state_desired[1] = _DroneState.position[1] ;
                                //state_desired[0]
                                break;
                            }

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Command_ID   = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
                            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XY_VEL_Z_POS;   //xy velocity z position
                            Command_to_pub.Reference_State.velocity_ref[0] = kp_land[0] * celi_err_x;
                            Command_to_pub.Reference_State.velocity_ref[1] = kp_land[1] * celi_err_y;
                            Command_to_pub.Reference_State.position_ref[2]     = 1.1;
                            move_pub.publish(Command_to_pub);

			                ifeverfind = 1;
			                findpositionx = _DroneState.position[0];
			                findpositiony = _DroneState.position[1];
                        }

                        else if((i >= 10)&&(i < 13) && !landmark.iffind)
                        {
                            move_circle(3,_DroneState.position[0],_DroneState.position[1]);
                        }

                        else if((i > 15) && (ifeverfind == 0))
                        {
                            cout << "target not found" << endl;
                            state_desired[0] = num_x[3];
                            state_desired[1] = num_y[3];
                            state_desired[2] = 1.3;
                            state_desired[3] = 0;

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Reference_State.Move_mode = Move_mode;
                            Command_to_pub.Reference_State.Move_frame = Move_frame;
                            Command_to_pub.Reference_State.time_from_start = -1;
                            generate_com(Move_mode, state_desired);
                            move_pub.publish(Command_to_pub);
                            sleep(2);
                            break;
                        }

                        else if(ifeverfind)
                        {
                            state_desired[0] = findpositionx;
                            state_desired[1] = findpositiony;
                            state_desired[2] = 1.3;
                            state_desired[3] = 0;

                            Command_to_pub.header.stamp = ros::Time::now();
                            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                            Command_to_pub.source = NODE_NAME;
                            Command_to_pub.Reference_State.Move_mode = Move_mode;
                            Command_to_pub.Reference_State.Move_frame = Move_frame;
                            Command_to_pub.Reference_State.time_from_start = -1;
                            generate_com(Move_mode, state_desired);
                            move_pub.publish(Command_to_pub);
                        }

                        ros::Duration(0.5).sleep();
                        ros::spinOnce();
                    }

                    state_desired[2] = 0.15;
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    Command_to_pub.Reference_State.Move_mode = Move_mode;
                    Command_to_pub.Reference_State.Move_frame = Move_frame;
                    Command_to_pub.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);
                    move_pub.publish(Command_to_pub);

                    while(_DroneState.position[2] > 0.2)
                    {
                        ros::Duration(0.5).sleep();
                        ros::spinOnce();
                    }

		            err_x = state_desired[0] - _DroneState.position[0];
    		        err_y = state_desired[1] - _DroneState.position[1];

		            while((abs(err_x) > 0.1) || (abs(err_y) > 0.1))
		            {
                        Command_to_pub.header.stamp = ros::Time::now();
                        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                        Command_to_pub.source = NODE_NAME;
                        Command_to_pub.Reference_State.Move_mode = Move_mode;
                        Command_to_pub.Reference_State.Move_frame = Move_frame;
                        Command_to_pub.Reference_State.time_from_start = -1;
                        generate_com(Move_mode, state_desired);
                        move_pub.publish(Command_to_pub);
                        ros::Duration(0.05).sleep();
                        ros::spinOnce();
			            err_x = state_desired[0] - _DroneState.position[0];
    		            err_y = state_desired[1] - _DroneState.position[1];
		            }

                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    Command_to_pub.Reference_State.Move_mode = Move_mode;
                    Command_to_pub.Reference_State.Move_frame = Move_frame;
                    Command_to_pub.Reference_State.time_from_start = -1;
                    generate_com(Move_mode, state_desired);
                    move_pub.publish(Command_to_pub);
                    sleep(4);

                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Disarm;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    move_pub.publish(Command_to_pub);


                }
            }
        }
        else if (Control_Mode == 999)
        {
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.yaw_ref = 999;
            move_pub.publish(Command_to_pub);
            Command_to_pub.Reference_State.yaw_ref = 0.0;
        }

        switch (Control_Mode)
        {
        case prometheus_msgs::ControlCommand::Idle:
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);
            break;

        case prometheus_msgs::ControlCommand::Takeoff:
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);
            break;

        case prometheus_msgs::ControlCommand::Hold:
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Hold;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);
            break;

        case prometheus_msgs::ControlCommand::Land:
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);
            break;

        case prometheus_msgs::ControlCommand::Move:
            if (Move_mode == prometheus_msgs::PositionReference::TRAJECTORY)
            {
                time_trajectory = 0.0;

                while (time_trajectory < trajectory_total_time)
                {
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;

                    if (Trjectory_mode == 0)
                    {
                        Command_to_pub.Reference_State = Controller_Test.Circle_trajectory_generation(time_trajectory);
                    }
                    else if (Trjectory_mode == 1)
                    {
                        Command_to_pub.Reference_State = Controller_Test.Eight_trajectory_generation(time_trajectory);
                    }
                    else if (Trjectory_mode == 2)
                    {
                        Command_to_pub.Reference_State = Controller_Test.Step_trajectory_generation(time_trajectory);
                    }
                    else if (Trjectory_mode == 3)
                    {
                        Command_to_pub.Reference_State = Controller_Test.Line_trajectory_generation(time_trajectory);
                    }

                    move_pub.publish(Command_to_pub);
                    time_trajectory = time_trajectory + 0.01;

                    cout << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << endl;

                    Draw_in_rviz(Command_to_pub.Reference_State, true);

                    ros::Duration(0.01).sleep();
                }
            }
            else
            {
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode = Move_mode;
                Command_to_pub.Reference_State.Move_frame = Move_frame;
                Command_to_pub.Reference_State.time_from_start = -1;
                generate_com(Move_mode, state_desired);

                move_pub.publish(Command_to_pub);
            }
            break;

        case prometheus_msgs::ControlCommand::Disarm:
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Disarm;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);
            break;

        case prometheus_msgs::ControlCommand::User_Mode1:
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::User_Mode1;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);
            break;

        case prometheus_msgs::ControlCommand::User_Mode2:
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::User_Mode2;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);
            break;
        }

        cout << "....................................................." << endl;

        sleep(1.0);
    }
}

void mainloop2()
{
    KeyboardEvent keyboardcontrol;
    Controller_Test Controller_Test;

    char key_now;
    char key_last;

    cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<" << endl;
    cout << "ENTER key to control the drone: " << endl;
    cout << "1 for Arm, Space for Takeoff, L for Land, H for Hold, 0 for Disarm, 8/9 for Trajectory tracking" << endl;
    cout << "Move mode is fixed (XYZ_VEL,BODY_FRAME): w/s for body_x, a/d for body_y, k/m for z, q/e for body_yaw" << endl;
    cout << "CTRL-C to quit." << endl;

    while (ros::ok())
    {
        keyboardcontrol.RosWhileLoopRun();
        key_now = keyboardcontrol.GetPressedKey();
        switch (key_now)
        {

        //悬停, 应当只发送一次, 不需要循环发送
        case U_KEY_NONE:

            if (key_last != U_KEY_NONE)
            {
                //to be continued.
            }
            sleep(0.5);

            break;

        // 数字1（非小键盘数字）：解锁及切换到OFFBOARD模式
        case U_KEY_1:
            cout << " " << endl;
            cout << "Arm and Switch to OFFBOARD." << endl;

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.yaw_ref = 999;
            move_pub.publish(Command_to_pub);
            sleep(1.0);
            break;

        // 空格：起飞
        case U_KEY_SPACE:
            cout << " " << endl;
            cout << "Switch to Takeoff Mode." << endl;

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.Reference_State.yaw_ref = 0.0;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);

            sleep(1.0);

            break;

        // 键盘L：降落
        case U_KEY_L:
            cout << " " << endl;
            cout << "Switch to Land Mode." << endl;

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);

            break;

        // 键盘0（非小键盘数字）：紧急停止
        case U_KEY_0:
            cout << " " << endl;
            cout << "Switch to Disarm Mode." << endl;

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Disarm;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);

            break;

        //起飞要维持起飞的模式?
        case U_KEY_T:
            cout << " " << endl;
            cout << "Switch to Takeoff Mode." << endl;

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            move_pub.publish(Command_to_pub);

            sleep(2.0);

            break;

        //起飞要维持起飞的模式?
        case U_KEY_H:
            cout << " " << endl;
            cout << "Switch to Hold Mode." << endl;

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Hold;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.position_ref[0] = 0;
            Command_to_pub.Reference_State.position_ref[1] = 0;
            Command_to_pub.Reference_State.position_ref[2] = 0;
            Command_to_pub.Reference_State.velocity_ref[0] = 0;
            Command_to_pub.Reference_State.velocity_ref[1] = 0;
            Command_to_pub.Reference_State.velocity_ref[2] = 0;
            Command_to_pub.Reference_State.acceleration_ref[0] = 0;
            Command_to_pub.Reference_State.acceleration_ref[1] = 0;
            Command_to_pub.Reference_State.acceleration_ref[2] = 0;
            move_pub.publish(Command_to_pub);

            sleep(1.0);

            break;

        // 向前匀速运动
        case U_KEY_W:

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_to_pub.Reference_State.velocity_ref[0] += VEL_XY_STEP_SIZE;
            move_pub.publish(Command_to_pub);

            cout << " " << endl;
            cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] " << endl;

            sleep(0.1);

            break;

        // 向后匀速运动
        case U_KEY_S:

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_to_pub.Reference_State.velocity_ref[0] -= VEL_XY_STEP_SIZE;
            move_pub.publish(Command_to_pub);
            cout << " " << endl;
            cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] " << endl;

            sleep(0.1);

            break;

        // 向左匀速运动
        case U_KEY_A:

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_to_pub.Reference_State.velocity_ref[1] += VEL_XY_STEP_SIZE;
            move_pub.publish(Command_to_pub);

            cout << " " << endl;
            cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] " << endl;

            sleep(0.1);

            break;

        // 向右匀速运动
        case U_KEY_D:

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_to_pub.Reference_State.velocity_ref[1] -= VEL_XY_STEP_SIZE;
            move_pub.publish(Command_to_pub);

            cout << " " << endl;
            cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] " << endl;

            sleep(0.1);

            break;

        // 向上运动
        case U_KEY_K:

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_to_pub.Reference_State.velocity_ref[2] += VEL_Z_STEP_SIZE;
            move_pub.publish(Command_to_pub);

            cout << " " << endl;
            cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] " << endl;

            sleep(0.1);

            break;

        // 向下运动
        case U_KEY_M:

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_to_pub.Reference_State.velocity_ref[2] -= VEL_Z_STEP_SIZE;
            move_pub.publish(Command_to_pub);

            cout << " " << endl;
            cout << "Current Velocity [X Y Z]: " << Command_to_pub.Reference_State.velocity_ref[0] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[1] << " [m/s] " << Command_to_pub.Reference_State.velocity_ref[2] << " [m/s] " << endl;

            sleep(0.1);

            break;

        // 偏航运动，左转 （这个里偏航控制的是位置 不是速度）
        case U_KEY_Q:

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_to_pub.Reference_State.yaw_ref = YAW_STEP_SIZE;
            move_pub.publish(Command_to_pub);

            cout << " " << endl;
            cout << "Increase the Yaw angle." << endl;

            sleep(0.1);

            break;

        // 偏航运动，右转
        case U_KEY_E:

            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
            Command_to_pub.Reference_State.yaw_ref = YAW_STEP_SIZE;
            move_pub.publish(Command_to_pub);

            cout << " " << endl;
            cout << "Decrease the Yaw angle." << endl;

            sleep(0.1);

            break;

        // 圆形追踪
        case U_KEY_9:
            time_trajectory = 0.0;
            trajectory_total_time = 50.0;
            // 需要设置
            while (time_trajectory < trajectory_total_time)
            {
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;

                Command_to_pub.Reference_State = Controller_Test.Circle_trajectory_generation(time_trajectory);

                move_pub.publish(Command_to_pub);
                time_trajectory = time_trajectory + 0.01;

                cout << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << endl;

                Draw_in_rviz(Command_to_pub.Reference_State, true);

                ros::Duration(0.01).sleep();
            }
            break;

        // 8字追踪
        case U_KEY_8:
            time_trajectory = 0.0;
            trajectory_total_time = 50.0;
            // 需要设置
            while (time_trajectory < trajectory_total_time)
            {
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;

                Command_to_pub.Reference_State = Controller_Test.Eight_trajectory_generation(time_trajectory);

                move_pub.publish(Command_to_pub);
                time_trajectory = time_trajectory + 0.01;

                cout << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << endl;

                Draw_in_rviz(Command_to_pub.Reference_State, true);

                ros::Duration(0.01).sleep();
            }
            break;
        }

        key_last = key_now;
        ros::spinOnce();
        sleep(0.1);
    }
}

void generate_com(int Move_mode, float state_desired[4])
{
    //# Move_mode 2-bit value:
    //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
    //#                   xy position     xy velocity
    //# z position        0b00(0)       0b10(2)
    //# z velocity 0b01(1)       0b11(3)

    if (Move_mode == prometheus_msgs::PositionReference::XYZ_ACC)
    {
        cout << "ACC control not support yet." << endl;
    }

    if ((Move_mode & 0b10) == 0) //xy channel
    {
        Command_to_pub.Reference_State.position_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.position_ref[1] = state_desired[1];
        Command_to_pub.Reference_State.velocity_ref[0] = 0;
        Command_to_pub.Reference_State.velocity_ref[1] = 0;
    }
    else
    {
        Command_to_pub.Reference_State.position_ref[0] = 0;
        Command_to_pub.Reference_State.position_ref[1] = 0;
        Command_to_pub.Reference_State.velocity_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if ((Move_mode & 0b01) == 0) //z channel
    {
        Command_to_pub.Reference_State.position_ref[2] = state_desired[2];
        Command_to_pub.Reference_State.velocity_ref[2] = 0;
    }
    else
    {
        Command_to_pub.Reference_State.position_ref[2] = 0;
        Command_to_pub.Reference_State.velocity_ref[2] = state_desired[2];
    }

    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;

    Command_to_pub.Reference_State.yaw_ref = state_desired[3] / 180.0 * M_PI;
}

void Draw_in_rviz(const prometheus_msgs::PositionReference &pos_ref, bool draw_trajectory)
{
    geometry_msgs::PoseStamped reference_pose;

    reference_pose.header.stamp = ros::Time::now();
    reference_pose.header.frame_id = "world";

    reference_pose.pose.position.x = pos_ref.position_ref[0];
    reference_pose.pose.position.y = pos_ref.position_ref[1];
    reference_pose.pose.position.z = pos_ref.position_ref[2];

    if (draw_trajectory)
    {
        posehistory_vector_.insert(posehistory_vector_.begin(), reference_pose);
        if (posehistory_vector_.size() > TRA_WINDOW)
        {
            posehistory_vector_.pop_back();
        }

        nav_msgs::Path reference_trajectory;
        reference_trajectory.header.stamp = ros::Time::now();
        reference_trajectory.header.frame_id = "world";
        reference_trajectory.poses = posehistory_vector_;
        ref_trajectory_pub.publish(reference_trajectory);
    }
    else
    {
        posehistory_vector_.clear();

        nav_msgs::Path reference_trajectory;
        reference_trajectory.header.stamp = ros::Time::now();
        reference_trajectory.header.frame_id = "world";
        reference_trajectory.poses = posehistory_vector_;
        ref_trajectory_pub.publish(reference_trajectory);
    }
}


void move_circle(int n, double center_x, double center_y)
{
    double target_x,target_y;
    target_x = center_x + 0.1 * n;
    target_y = center_y;
    for(int i = 1; i <= n; i++)
    {
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
        Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_to_pub.Reference_State.position_ref[0]     = center_x + 0.1 * i;
        Command_to_pub.Reference_State.position_ref[1]     = center_y;
        Command_to_pub.Reference_State.position_ref[2]     = 1.2;
        Command_to_pub.Reference_State.yaw_ref             = 0.0;
        move_pub.publish(Command_to_pub);

        for(int j = 0; j < i*6; j++)
        {
            ros::Duration(0.1 ).sleep();
            ros::spinOnce();
            if(landmark.iffind)
            {
                target_x = _DroneState.position[0];
                target_y = _DroneState.position[1];
                break;
            }
        }

        if(landmark.iffind)
        {
            break;
        }


        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
        Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_to_pub.Reference_State.position_ref[0]     = center_x + 0.1 * i;
        Command_to_pub.Reference_State.position_ref[1]     = center_y + 0.1 * i;
        Command_to_pub.Reference_State.position_ref[2]     = 1.2;
        Command_to_pub.Reference_State.yaw_ref             = 0.0;
        move_pub.publish(Command_to_pub);

        for(int j = 0; j < i*6; j++)
        {
            ros::Duration(0.1 ).sleep();
            ros::spinOnce();
            if(landmark.iffind)
            {
                target_x = _DroneState.position[0];
                target_y = _DroneState.position[1];
                break;
            }
        }

        if(landmark.iffind)
        {
            break;
        }

        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
        Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_to_pub.Reference_State.position_ref[0]     = center_x - 0.1 * i;
        Command_to_pub.Reference_State.position_ref[1]     = center_y + 0.1 * i;
        Command_to_pub.Reference_State.position_ref[2]     = 1.2;
        Command_to_pub.Reference_State.yaw_ref             = 0.0;
        move_pub.publish(Command_to_pub);

        for(int j = 0; j < i*6; j++)
        {
            ros::Duration(0.1 ).sleep();
            ros::spinOnce();
            if(landmark.iffind)
            {
                target_x = _DroneState.position[0];
                target_y = _DroneState.position[1];
                break;
            }
        }

        if(landmark.iffind)
        {
            break;
        }

        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
        Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_to_pub.Reference_State.position_ref[0]     = center_x - 0.1 * i;
        Command_to_pub.Reference_State.position_ref[1]     = center_y - 0.1 * i;
        Command_to_pub.Reference_State.position_ref[2]     = 1.2;
        Command_to_pub.Reference_State.yaw_ref             = 0.0;
        move_pub.publish(Command_to_pub);

        for(int j = 0; j < i*6; j++)
        {
            ros::Duration(0.1 ).sleep();
            ros::spinOnce();
            if(landmark.iffind)
            {
                target_x = _DroneState.position[0];
                target_y = _DroneState.position[1];
                break;
            }
        }

        if(landmark.iffind)
        {
            break;
        }

        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
        Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_to_pub.Reference_State.position_ref[0]     = center_x + 0.1 * i;
        Command_to_pub.Reference_State.position_ref[1]     = center_y - 0.1 * i;
        Command_to_pub.Reference_State.position_ref[2]     = 1.2;
        Command_to_pub.Reference_State.yaw_ref             = 0.0;
        move_pub.publish(Command_to_pub);

        for(int j = 0; j < i*6; j++)
        {
            ros::Duration(0.1 ).sleep();
            ros::spinOnce();
            if(landmark.iffind)
            {
                target_x = _DroneState.position[0];
                target_y = _DroneState.position[1];
                break;
            }
        }

        if(landmark.iffind)
        {
            break;
        }

    }

    Command_to_pub.header.stamp = ros::Time::now();
    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;
    Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_to_pub.Reference_State.position_ref[0]     = target_x;
    Command_to_pub.Reference_State.position_ref[1]     = target_y;
    Command_to_pub.Reference_State.position_ref[2]     = 1.2;
    Command_to_pub.Reference_State.yaw_ref             = 0.0;
    move_pub.publish(Command_to_pub);

}

