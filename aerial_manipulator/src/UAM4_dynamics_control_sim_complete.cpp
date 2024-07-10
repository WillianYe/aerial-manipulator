#include "ros/ros.h"
#include <iostream>
#include <Eigen/Eigen>
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"

//发布转子速度要用到的
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

//接收IMU和位位姿数据要用到的
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

//接收关节位置和速度以及发布力矩要用到的
#include <sensor_msgs/JointState.h>
#include<cstdlib>
#include<ctime>


#define pi 3.141592653589793
using namespace std;

double control_rate=200;
double control_period=1/control_rate;
unsigned long step_control=1;
double pretime=3;      //准备时间，在这个时间之内先让关节控制到初始角度，在这之后才控制螺旋浆
bool ifperchsuccess=0;

////////广义关节期望值 (5个关节中只有theta3是人为指定)
double theta3_goal;
Eigen::Vector3d P_IP_d;
Eigen::Vector3d P_IE_d;
Eigen::Vector3d zyx_d;

Eigen::Vector3d P_IB_d;
Eigen::Matrix3d R_IB_d;
//其他全局变量
Eigen::MatrixXd tao(11,1);   
Eigen::MatrixXd taox(11,1);  
Eigen::MatrixXd theta_cache(5,1);
Eigen::MatrixXd theta_goal_cache(5,1);
Eigen::MatrixXd Thrust_torque(6,1);      //1x1 r1动力组驱动力，3×1 r2动力组3维驱动力，1×1 r1动力组驱动扭力，1×1 r2动力组驱动扭力 ，更多解释见后面
Eigen::MatrixXd Vx(11,1);
Eigen::MatrixXd taox_ext(11,1);////
Eigen::MatrixXd Vq(11,1);
//运动学与动力学参数
double Lb=0.3;
double L3=0.3;
double Lr=0.1; 
double Lp=0.02;
double Cm=8.54858e-06;
double Ct_Cm=0.02;          //拉力系数与反扭矩系数的比例
double m1=0.15;
double m2=0.015;
double m5=0.15;
double m4=0.015;
double m3=0.2;
double mb=0.35;
double I11=double(1)/12*m1*(0.16*0.08), I12=double(1)/12*m1*(0.08*0.08), I13=double(1)/12*m1*(0.16*0.08);      
double I21=double(1)/12*m2*(0.02*0.02), I22=double(1)/12*m2*(0.03*0.02), I23=double(1)/12*m2*(0.03*0.02);      
double I31=double(1)/12*m3*(0.02*0.02), I32=double(1)/12*m3*(0.3*0.02),  I33=double(1)/12*m3*(0.3*0.02);       
double I41=double(1)/12*m4*(0.02*0.02), I42=double(1)/12*m4*(0.03*0.02), I43=double(1)/12*m4*(0.03*0.02);      
double I51=double(1)/12*m5*(0.16*0.08), I52=double(1)/12*m5*(0.08*0.08), I53=double(1)/12*m5*(0.08*0.16);      
double Ib1=double(1)/12*mb*(0.02*0.02), Ib2=double(1)/12*mb*(0.3*0.02),  Ib3=double(1)/12*mb*(0.3*0.02);      //惯性张量对角元素




//////////////////////////////////回调函数
struct pose_msg_t{
    // 位置
    double x,y,z;
    // 四元数
    double qx,qy,qz,qw;
	double vx,vy,vz;
    double wx,wy,wz;
}pose_msg,pose_p_msg,pose_e_msg;

struct imu_data_t{
    // x,y,z方向的角速度
    double wx,wy,wz;
}imu_data;
struct arm_data_t{
    // x,y,z方向的角速度
    double pos[5],vel[5];
}arm_data;

void set_pose(const nav_msgs::Odometry::ConstPtr& msg) {

    pose_msg.x = msg->pose.pose.position.x;
    pose_msg.y = msg->pose.pose.position.y;
    pose_msg.z = msg->pose.pose.position.z;
    pose_msg.qx = msg->pose.pose.orientation.x;
    pose_msg.qy = msg->pose.pose.orientation.y;
    pose_msg.qz = msg->pose.pose.orientation.z;
    pose_msg.qw = msg->pose.pose.orientation.w;
    pose_msg.vx = msg->twist.twist.linear.x;
    pose_msg.vy = msg->twist.twist.linear.y;
    pose_msg.vz= msg->twist.twist.linear.z;
    pose_msg.wx = msg->twist.twist.angular.x;
    pose_msg.wy = msg->twist.twist.angular.y;
    pose_msg.wz= msg->twist.twist.angular.z; 

}
void set_imu(const sensor_msgs::Imu::ConstPtr& msg) {

    imu_data.wx = msg->angular_velocity.x;
    imu_data.wy = msg->angular_velocity.y;
    imu_data.wz = msg->angular_velocity.z;
}
void set_p_pose(const nav_msgs::Odometry::ConstPtr& msg) {

    pose_p_msg.x = msg->pose.pose.position.x;
    pose_p_msg.y = msg->pose.pose.position.y;
    pose_p_msg.z = msg->pose.pose.position.z;
    pose_p_msg.qx = msg->pose.pose.orientation.x;
    pose_p_msg.qy = msg->pose.pose.orientation.y;
    pose_p_msg.qz = msg->pose.pose.orientation.z;
    pose_p_msg.qw = msg->pose.pose.orientation.w;
    pose_p_msg.vx = msg->twist.twist.linear.x;
    pose_p_msg.vy = msg->twist.twist.linear.y;
    pose_p_msg.vz= msg->twist.twist.linear.z;
    pose_p_msg.wx = msg->twist.twist.angular.x;
    pose_p_msg.wy = msg->twist.twist.angular.y;
    pose_p_msg.wz= msg->twist.twist.angular.z;    
}
void set_e_pose(const nav_msgs::Odometry::ConstPtr& msg) {

    pose_e_msg.x = msg->pose.pose.position.x;
    pose_e_msg.y = msg->pose.pose.position.y;
    pose_e_msg.z = msg->pose.pose.position.z;
    pose_e_msg.qx = msg->pose.pose.orientation.x;
    pose_e_msg.qy = msg->pose.pose.orientation.y;
    pose_e_msg.qz = msg->pose.pose.orientation.z;
    pose_e_msg.qw = msg->pose.pose.orientation.w;
    pose_e_msg.vx = msg->twist.twist.linear.x;
    pose_e_msg.vy = msg->twist.twist.linear.y;
    pose_e_msg.vz= msg->twist.twist.linear.z;
    pose_e_msg.wx = msg->twist.twist.angular.x;
    pose_e_msg.wy = msg->twist.twist.angular.y;
    pose_e_msg.wz= msg->twist.twist.angular.z;    
}


void set_armstate(const sensor_msgs::JointState::ConstPtr& msg) {

    arm_data.pos[0] = msg->position[0];
    arm_data.pos[1] = msg->position[1];
    arm_data.pos[2] = msg->position[2];
    arm_data.pos[3] = msg->position[3];
    arm_data.pos[4] = msg->position[4];
    arm_data.vel[0] = msg->velocity[0];
    arm_data.vel[1] = msg->velocity[1];
    arm_data.vel[2] = msg->velocity[2];
    arm_data.vel[3] = msg->velocity[3];
    arm_data.vel[4] = msg->velocity[4];

}


Eigen::Vector3d tmp_vector(double x, double y, double z)
{
	Eigen::Vector3d output;
	output << x, y, z;
	return output;
}

Eigen::Matrix3d compute_R(double x, double y, double z)
{
	Eigen::Matrix3d output_matrix;
	if(x!=0)
        output_matrix << 1,   0,    0,
						 0,cos(x),-sin(x),
						 0,sin(x), cos(x);
    else if(y!=0)
        output_matrix << cos(y),0,sin(y),
						 0,      1,    0,
						-sin(y),0,cos(y);
    else
        output_matrix << cos(z),-sin(z),0,
						 sin(z),cos(z),0,
						  0,   0,    1;
	return output_matrix;
}

Eigen::Matrix3d skew(Eigen::Vector3d input)
{
	Eigen::Matrix<double, 3, 3> output;
	output << 0, -input(2),input(1),
			input(2), 0, -input(0),
			-input(1), input(0), 0;
	return output;
}

Eigen::Matrix3d compute_Q(Eigen::Vector3d zyx)
{
	Eigen::Matrix3d output;
	output << 1, 0,-sin(zyx(1)),
			0, cos(zyx(0)), cos(zyx(1))*sin(zyx(0)),
			0, -sin(zyx(0)), cos(zyx(1))*cos(zyx(0));
	return output;
}


 
int main ( int argc, char** argv )
{
    //全局变量初始化
    tao<<0,0,0,0,0,0,0,0,0,0,0;
    taox<<0,0,0,0,0,0,0,0,0,0,0;
    Thrust_torque<<0,0,0,0,0,0;
    Eigen::MatrixXd tao_ext(11,1);
    tao_ext=Eigen::MatrixXd::Zero(11,1);
    taox_ext=Eigen::MatrixXd::Zero(11,1);
    bool set_state[20]={0};
    Vx=Eigen::MatrixXd::Zero(11,1);
    Vq=Eigen::MatrixXd::Zero(11,1);
    //ROS节点定义
    ros::init(argc, argv, "UAM4_dynamics_control");
    ros::NodeHandle n;
    //定义订阅者和发布者
    ros::Subscriber sub_armstate = n.subscribe("/firefly/arm/joint_states", 2, set_armstate);
    ros::Subscriber sub_pose = n.subscribe("/firefly/ground_truth/odometry", 2, set_pose);
    ros::Subscriber sub_imu = n.subscribe("/firefly/ground_truth/imu", 2, set_imu);

    ros::Subscriber sub_p_pose = n.subscribe("/firefly/odometry_sensor2/odometry", 2, set_p_pose);
    ros::Subscriber sub_e_pose = n.subscribe("/firefly/odometry_sensor3/odometry", 2, set_e_pose);


    ros::Publisher pub_J1_goalcur= n.advertise<std_msgs::Float64>("/firefly/arm/J1_effort_controller/command", 2);    
    ros::Publisher pub_J2_goalcur= n.advertise<std_msgs::Float64>("/firefly/arm/J2_effort_controller/command", 2); 
    ros::Publisher pub_J3_goalcur= n.advertise<std_msgs::Float64>("/firefly/arm/J3_effort_controller/command", 2); 
    ros::Publisher pub_J4_goalcur= n.advertise<std_msgs::Float64>("/firefly/arm/J4_effort_controller/command", 2); 
    ros::Publisher pub_J5_goalcur= n.advertise<std_msgs::Float64>("/firefly/arm/J5_effort_controller/command", 2); 
    ros::Publisher pub_vel = n.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 2);
    
    ros::Publisher pub_plot_estimate_fz = n.advertise<std_msgs::Float64>("/firefly/estimate_fz", 2);

    /*
    ros::Publisher pub_plot_actual_rx = n.advertise<std_msgs::Float64>("/firefly/plot_actual_rx", 2);
    ros::Publisher pub_plot_desire_rx = n.advertise<std_msgs::Float64>("/firefly/plot_desire_rx", 2);
    ros::Publisher pub_plot_actual_ry = n.advertise<std_msgs::Float64>("/firefly/plot_actual_ry", 2);
    ros::Publisher pub_plot_desire_ry = n.advertise<std_msgs::Float64>("/firefly/plot_desire_ry", 2);
    ros::Publisher pub_plot_actual_rz = n.advertise<std_msgs::Float64>("/firefly/plot_actual_rz", 2);
    ros::Publisher pub_plot_desire_rz = n.advertise<std_msgs::Float64>("/firefly/plot_desire_rz", 2);

    ros::Publisher pub_plot_actual_ex = n.advertise<std_msgs::Float64>("/firefly/plot_actual_ex", 2);
    ros::Publisher pub_plot_desire_ex = n.advertise<std_msgs::Float64>("/firefly/plot_desire_ex", 2);
    ros::Publisher pub_plot_actual_ey = n.advertise<std_msgs::Float64>("/firefly/plot_actual_ey", 2);
    ros::Publisher pub_plot_desire_ey = n.advertise<std_msgs::Float64>("/firefly/plot_desire_ey", 2);
    ros::Publisher pub_plot_actual_ez = n.advertise<std_msgs::Float64>("/firefly/plot_actual_ez", 2);
    ros::Publisher pub_plot_desire_ez = n.advertise<std_msgs::Float64>("/firefly/plot_desire_ez", 2); 

    ros::Publisher pub_plot_motor1 = n.advertise<std_msgs::Float64>("/firefly/plot_motor1", 2);
    ros::Publisher pub_plot_motor2 = n.advertise<std_msgs::Float64>("/firefly/plot_motor2", 2);
    ros::Publisher pub_plot_motor3 = n.advertise<std_msgs::Float64>("/firefly/plot_motor3", 2);
    ros::Publisher pub_plot_motor4 = n.advertise<std_msgs::Float64>("/firefly/plot_motor4", 2); 
    */
    //ros循环频率
    ros::Rate loop_rate(control_rate); 


    while(ros::ok()){
                
        //订阅b系位置
        Eigen::Vector3d P_IB,V_BB;
        Eigen::Matrix3d R_IB;
        P_IB << pose_msg.x, pose_msg.y, pose_msg.z;
        //订阅b系速度
        V_BB<<pose_msg.vx,pose_msg.vy,pose_msg.vz;
        //订阅I到b的旋转矩阵
        R_IB   <<1-2*pose_msg.qy*pose_msg.qy-2*pose_msg.qz*pose_msg.qz  , 2*pose_msg.qx*pose_msg.qy-2*pose_msg.qw*pose_msg.qz ,   2*pose_msg.qx*pose_msg.qz+2*pose_msg.qw*pose_msg.qy,
        2*pose_msg.qx*pose_msg.qy+2*pose_msg.qw*pose_msg.qz   , 1-2*pose_msg.qx*pose_msg.qx-2*pose_msg.qz*pose_msg.qz ,   2*pose_msg.qy*pose_msg.qz-2*pose_msg.qw*pose_msg.qx,
        2*pose_msg.qx*pose_msg.qz-2*pose_msg.qw*pose_msg.qy  ,  2*pose_msg.qy*pose_msg.qz+2*pose_msg.qw*pose_msg.qx  ,  1-2*pose_msg.qx*pose_msg.qx-2*pose_msg.qy*pose_msg.qy;               
        //订阅b系角速度(no use)
        Eigen::Vector3d W_BB;
        W_BB << imu_data.wx, imu_data.wy, imu_data.wz; 

        //订阅p系速度
        Eigen::Vector3d V_PPM,P_IPM;
        P_IPM << pose_p_msg.x, pose_p_msg.y, pose_p_msg.z;
        V_PPM <<pose_p_msg.vx,pose_p_msg.vy,pose_p_msg.vz;
        //订阅I到p的旋转矩阵
        Eigen::Matrix3d R_IPM;
        R_IPM   <<1-2*pose_p_msg.qy*pose_p_msg.qy-2*pose_p_msg.qz*pose_p_msg.qz  , 2*pose_p_msg.qx*pose_p_msg.qy-2*pose_p_msg.qw*pose_p_msg.qz ,   2*pose_p_msg.qx*pose_p_msg.qz+2*pose_p_msg.qw*pose_p_msg.qy,
        2*pose_p_msg.qx*pose_p_msg.qy+2*pose_p_msg.qw*pose_p_msg.qz   , 1-2*pose_p_msg.qx*pose_p_msg.qx-2*pose_p_msg.qz*pose_p_msg.qz ,   2*pose_p_msg.qy*pose_p_msg.qz-2*pose_p_msg.qw*pose_p_msg.qx,
        2*pose_p_msg.qx*pose_p_msg.qz-2*pose_p_msg.qw*pose_p_msg.qy  ,  2*pose_p_msg.qy*pose_p_msg.qz+2*pose_p_msg.qw*pose_p_msg.qx  ,  1-2*pose_p_msg.qx*pose_p_msg.qx-2*pose_p_msg.qy*pose_p_msg.qy;             
        //订阅p系角速度
        Eigen::Vector3d W_PPM;
        W_PPM << pose_p_msg.wx, pose_p_msg.wy, pose_p_msg.wz; 
        Eigen::Vector3d zyxm;
        zyxm<<atan2(R_IPM(2,1),R_IPM(2,2)),
             atan2(-R_IPM(2,0),sqrt(R_IPM(2,1)*R_IPM(2,1)+R_IPM(2,2)*R_IPM(2,2))),
             atan2(R_IPM(1,0),R_IPM(0,0));
        Eigen::Matrix3d QM=compute_Q(zyxm); 
        Eigen::Vector3d zyx_diffm=QM.inverse()*W_PPM; 
        
        //订阅e系速度
        Eigen::Vector3d V_EEM;
        V_EEM <<pose_e_msg.vx,pose_e_msg.vy,pose_e_msg.vz;
        Eigen::Vector3d P_IEM;
        P_IEM << pose_e_msg.x, pose_e_msg.y, pose_e_msg.z;

        
        //差分计算关节角速度
        Eigen::MatrixXd theta(5,1); 
        theta << arm_data.pos[0], arm_data.pos[1], arm_data.pos[2], arm_data.pos[3], arm_data.pos[4];
        Eigen::MatrixXd theta_diff(5,1); 
        if (step_control==1)  theta_cache=theta;
        theta_diff = (theta-theta_cache)/control_period;
        theta_cache=theta; 

        //动力学参数
        double R_IB1_1=R_IB(0,0), R_IB1_2=R_IB(0,1), R_IB1_3=R_IB(0,2), R_IB2_1=R_IB(1,0), R_IB2_2=R_IB(1,1), R_IB2_3=R_IB(1,2), R_IB3_1=R_IB(2,0), R_IB3_2=R_IB(2,1), R_IB3_3=R_IB(2,2);
        double theta1=theta(0,0), theta2=theta(1,0), theta3=theta(2,0), theta4=theta(3,0), theta5=theta(4,0);
        double theta1_diff=theta_diff(0,0),  theta2_diff=theta_diff(1,0),  theta3_diff=theta_diff(2,0),  theta4_diff=theta_diff(3,0),  theta5_diff=theta_diff(4,0);
        double vb1=V_BB(0,0), vb2=V_BB(1,0), vb3=V_BB(2,0);
        double wb1=W_BB(0,0), wb2=W_BB(1,0), wb3=W_BB(2,0);       
       
        Vq.block(0,0,3,1)=V_BB;
        Vq.block(3,0,3,1)=W_BB;
        Vq.block(6,0,5,1)=theta_diff;

        //动力学参数计算
        Eigen::MatrixXd M(11,11);
        Eigen::MatrixXd C(11,1);
        Eigen::MatrixXd G(11,1);         
        M<<              m1 + m2 + m3 + m4 + m5 + mb,                                                                                                                           0,                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                (L3*sin(theta3)*(2*m3 + 3*m4 + 3*m5))/3,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,               0,                                                 0,                                                                                                                                                                                                                                                                                          (L3*sin(theta3)*(2*m3 + 3*m4 + 3*m5))/3,                                                                                                                         0,                           0,
                                        0,                                                                                                 m1 + m2 + m3 + m4 + m5 + mb,                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     -(L3*sin(theta3)*(2*m3 + 3*m4 + 3*m5))/3,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              (Lb*m1)/2 + (Lb*m2)/2 - (Lb*m3)/2 - (Lb*m4)/2 - (Lb*m5)/2 - (2*L3*m3*cos(theta3))/3 - L3*m4*cos(theta3) - L3*m5*cos(theta3),               0,                                                 0,                                                                                                                                                                                                                                                                                                                                0,                                                                                                                         0,                           0,
                                        0,                                                                                                                           0,                                                                                                 m1 + m2 + m3 + m4 + m5 + mb,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            (Lb*m3)/2 - (Lb*m2)/2 - (Lb*m1)/2 + (Lb*m4)/2 + (Lb*m5)/2 + (2*L3*m3*cos(theta3))/3 + L3*m4*cos(theta3) + L3*m5*cos(theta3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,               0,                                                 0,                                                                                                                                                                                                                                                                                          (L3*cos(theta3)*(2*m3 + 3*m4 + 3*m5))/3,                                                                                                                         0,                           0,
                                        0,                                                                                    -(L3*sin(theta3)*(2*m3 + 3*m4 + 3*m5))/3,                                                                                                                           0,                                                                                                                                                                                                                                                                                  I13 + I21 + Ib1 + I11*cos(theta1)*cos(theta1) - I13*cos(theta1)*cos(theta1) + I31*cos(theta3)*cos(theta3) + I41*cos(theta3)*cos(theta3) + I33*sin(theta3)*sin(theta3) + I51*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) + I43*cos(theta4)*cos(theta4)*sin(theta3)*sin(theta3) + I53*cos(theta3)*cos(theta3)*sin(theta5)*sin(theta5) + I42*sin(theta3)*sin(theta3)*sin(theta4)*sin(theta4) + I52*sin(theta3)*sin(theta3)*sin(theta4)*sin(theta4) + (4*L3*L3*m3*sin(theta3)*sin(theta3))/9 + L3*L3*m4*sin(theta3)*sin(theta3)*sin(theta4)*sin(theta4) + L3*L3*m5*sin(theta3)*sin(theta3)*sin(theta4)*sin(theta4) + I53*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta3) + I51*cos(theta4)*cos(theta4)*sin(theta3)*sin(theta3)*sin(theta5)*sin(theta5) + L3*L3*m4*cos(theta4)*cos(theta4)*sin(theta3)*sin(theta3) + L3*L3*m5*cos(theta4)*cos(theta4)*sin(theta3)*sin(theta3) - 2*I51*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) + 2*I53*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5),                                                                                                                                                                                                                                                                                                  I11*cos(theta1)*sin(theta1)*sin(theta2) - I13*cos(theta1)*sin(theta1)*sin(theta2) + I42*cos(theta4)*sin(theta3)*sin(theta4) - I43*cos(theta4)*sin(theta3)*sin(theta4) - I51*cos(theta4)*sin(theta3)*sin(theta4) + I52*cos(theta4)*sin(theta3)*sin(theta4) + I51*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - I53*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) + I51*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - I53*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4),                             (I33*sin(2*theta3))/2 - (I31*sin(2*theta3))/2 - (I41*sin(2*theta3))/2 + (I42*sin(2*theta3))/2 + (I52*sin(2*theta3))/2 - (I53*sin(2*theta3))/2 + (2*L3*L3*m3*sin(2*theta3))/9 + (L3*L3*m4*sin(2*theta3))/2 + (L3*L3*m5*sin(2*theta3))/2 - I11*cos(theta1)*cos(theta2)*sin(theta1) + I13*cos(theta1)*cos(theta2)*sin(theta1) + I51*cos(theta4)*cos(theta5)*sin(theta5) - I53*cos(theta4)*cos(theta5)*sin(theta5) + (L3*Lb*m3*sin(theta3))/3 + (L3*Lb*m4*sin(theta3))/2 + (L3*Lb*m5*sin(theta3))/2 - I42*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I43*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I51*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I51*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) - I52*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I53*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I51*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) + 2*I53*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) - I51*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + I53*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3),               0, I11 + I21 - I11*sin(theta1)*sin(theta1) + I13*sin(theta1)*sin(theta1),                  sin(theta4)*(I42*cos(theta4)*sin(theta3) - I43*cos(theta4)*sin(theta3) - I51*cos(theta4)*sin(theta3) + I52*cos(theta4)*sin(theta3) + I51*cos(theta3)*cos(theta5)*sin(theta5) - I53*cos(theta3)*cos(theta5)*sin(theta5) + I51*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - I53*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)),   cos(theta3)*(I41 + I51 - I51*sin(theta5)*sin(theta5) + I53*sin(theta5)*sin(theta5)) - sin(2*theta5)*cos(theta4)*sin(theta3)*(I51/2 - I53/2), I52*sin(theta3)*sin(theta4),
                    (L3*sin(theta3)*(2*m3 + 3*m4 + 3*m5))/3,                                                                                                                           0, (Lb*m3)/2 - (Lb*m2)/2 - (Lb*m1)/2 + (Lb*m4)/2 + (Lb*m5)/2 + (2*L3*m3*cos(theta3))/3 + L3*m4*cos(theta3) + L3*m5*cos(theta3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  I11*cos(theta1)*sin(theta1)*sin(theta2) - I13*cos(theta1)*sin(theta1)*sin(theta2) + I42*cos(theta4)*sin(theta3)*sin(theta4) - I43*cos(theta4)*sin(theta3)*sin(theta4) + I52*cos(theta4)*sin(theta3)*sin(theta4) + I51*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - I53*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - I53*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - I51*cos(theta4)*sin(theta3)*sin(theta4)*sin(theta5)*sin(theta5), I11/4 + I12/2 + I13/4 + I22/2 + I23/2 + I32 + I42/2 + I43/2 + I51/4 + I52/2 + I53/4 + Ib2 + (4*L3*L3*m3)/9 + L3*L3*m4 + L3*L3*m5 + (Lb*Lb*m1)/4 + (Lb*Lb*m2)/4 + (Lb*Lb*m3)/4 + (Lb*Lb*m4)/4 + (Lb*Lb*m5)/4 - (I11*cos(2*theta1))/4 - (I11*cos(2*theta2))/4 + (I12*cos(2*theta2))/2 + (I13*cos(2*theta1))/4 - (I13*cos(2*theta2))/4 + (I22*cos(2*theta2))/2 - (I23*cos(2*theta2))/2 + (I42*cos(2*theta4))/2 - (I43*cos(2*theta4))/2 - (I51*cos(2*theta4))/4 - (I51*cos(2*theta5))/4 + (I52*cos(2*theta4))/2 - (I53*cos(2*theta4))/4 + (I53*cos(2*theta5))/4 + (I11*cos(2*theta1)*cos(2*theta2))/4 - (I13*cos(2*theta1)*cos(2*theta2))/4 + (I51*cos(2*theta4)*cos(2*theta5))/4 - (I53*cos(2*theta4)*cos(2*theta5))/4 + (2*L3*Lb*m3*cos(theta3))/3 + L3*Lb*m4*cos(theta3) + L3*Lb*m5*cos(theta3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          (I12*sin(2*theta2))/2 - (I11*sin(2*theta2))/2 + (I22*sin(2*theta2))/2 - (I23*sin(2*theta2))/2 + I42*cos(theta3)*cos(theta4)*sin(theta4) - I43*cos(theta3)*cos(theta4)*sin(theta4) + I52*cos(theta3)*cos(theta4)*sin(theta4) + I11*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) - I13*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) - I51*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + I53*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) - I53*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - I51*cos(theta3)*cos(theta4)*sin(theta4)*sin(theta5)*sin(theta5), I12*cos(theta2),         sin(2*theta1)*sin(theta2)*(I11/2 - I13/2), I32 + I43 + I51 + (4*L3*L3*m3)/9 + L3*L3*m4 + L3*L3*m5 + I42*cos(theta4)*cos(theta4) - I43*cos(theta4)*cos(theta4) - I51*cos(theta4)*cos(theta4) - I51*cos(theta5)*cos(theta5) + I52*cos(theta4)*cos(theta4) + I53*cos(theta5)*cos(theta5) + I51*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) - I53*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + (L3*Lb*m3*cos(theta3))/3 + (L3*Lb*m4*cos(theta3))/2 + (L3*Lb*m5*cos(theta3))/2,                                                                                 sin(2*theta5)*sin(theta4)*(I51/2 - I53/2),             I52*cos(theta4),
                                        0, (Lb*m1)/2 + (Lb*m2)/2 - (Lb*m3)/2 - (Lb*m4)/2 - (Lb*m5)/2 - (2*L3*m3*cos(theta3))/3 - L3*m4*cos(theta3) - L3*m5*cos(theta3),                                                                                                                           0, (I33*sin(2*theta3))/2 - (I31*sin(2*theta3))/2 - (I41*sin(2*theta3))/2 + (I42*sin(2*theta3))/2 + (I52*sin(2*theta3))/2 - (I53*sin(2*theta3))/2 + (2*L3*L3*m3*sin(2*theta3))/9 + (L3*L3*m4*sin(2*theta3))/2 + (L3*L3*m5*sin(2*theta3))/2 - I11*cos(theta1)*cos(theta2)*sin(theta1) + I13*cos(theta1)*cos(theta2)*sin(theta1) + I51*cos(theta4)*cos(theta5)*sin(theta5) - I53*cos(theta4)*cos(theta5)*sin(theta5) + (L3*Lb*m3*sin(theta3))/3 + (L3*Lb*m4*sin(theta3))/2 + (L3*Lb*m5*sin(theta3))/2 - I42*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I43*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I51*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I51*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) - I52*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I53*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I51*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) + 2*I53*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) - I51*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + I53*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3),                                                                                                                                                                                              (I12*sin(2*theta2))/2 - (I11*sin(2*theta2))/2 + (I22*sin(2*theta2))/2 - (I23*sin(2*theta2))/2 + I42*cos(theta3)*cos(theta4)*sin(theta4) - I43*cos(theta3)*cos(theta4)*sin(theta4) - I51*cos(theta3)*cos(theta4)*sin(theta4) + I52*cos(theta3)*cos(theta4)*sin(theta4) + I11*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) - I13*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) - I51*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + I53*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + I51*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - I53*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4), I12 + I22 + I31 + I41 + I53 + Ib3 + (Lb*Lb*m1)/4 + (Lb*Lb*m2)/4 + (Lb*Lb*m3)/4 + (Lb*Lb*m4)/4 + (Lb*Lb*m5)/4 + I11*cos(theta2)*cos(theta2) - I12*cos(theta2)*cos(theta2) - I22*cos(theta2)*cos(theta2) + I23*cos(theta2)*cos(theta2) - I31*cos(theta3)*cos(theta3) + I33*cos(theta3)*cos(theta3) - I41*cos(theta3)*cos(theta3) + I42*cos(theta3)*cos(theta3) + I52*cos(theta3)*cos(theta3) + I51*cos(theta5)*cos(theta5) - I53*cos(theta3)*cos(theta3) - I53*cos(theta5)*cos(theta5) - I11*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) + I13*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) - I42*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + I43*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + I51*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) - I51*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) - I52*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + I53*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) + (4*L3*L3*m3*cos(theta3)*cos(theta3))/9 + L3*L3*m4*cos(theta3)*cos(theta3) + L3*L3*m5*cos(theta3)*cos(theta3) + (2*L3*Lb*m3*cos(theta3))/3 + L3*Lb*m4*cos(theta3) + L3*Lb*m5*cos(theta3) - I51*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I53*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + 2*I51*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 2*I53*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5), I12*sin(theta2),        -(sin(2*theta1)*cos(theta2)*(I11 - I13))/2,                  sin(theta4)*(I42*cos(theta3)*cos(theta4) - I43*cos(theta3)*cos(theta4) - I51*cos(theta3)*cos(theta4) + I52*cos(theta3)*cos(theta4) - I51*cos(theta5)*sin(theta3)*sin(theta5) + I53*cos(theta5)*sin(theta3)*sin(theta5) + I51*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5) - I53*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)), - sin(theta3)*(I41 + I51 - I51*sin(theta5)*sin(theta5) + I53*sin(theta5)*sin(theta5)) - sin(2*theta5)*cos(theta3)*cos(theta4)*(I51/2 - I53/2), I52*cos(theta3)*sin(theta4),
                                        0,                                                                                                                           0,                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        I12*cos(theta2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          I12*sin(theta2),             I12,                                                 0,                                                                                                                                                                                                                                                                                                                                0,                                                                                                                         0,                           0,
                                        0,                                                                                                                           0,                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            I11 + I21 - I11*sin(theta1)*sin(theta1) + I13*sin(theta1)*sin(theta1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        cos(theta1)*sin(theta1)*sin(theta2)*(I11 - I13),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -cos(theta1)*cos(theta2)*sin(theta1)*(I11 - I13),               0, I11 + I21 - I11*sin(theta1)*sin(theta1) + I13*sin(theta1)*sin(theta1),                                                                                                                                                                                                                                                                                                                                0,                                                                                                                         0,                           0,
                    (L3*sin(theta3)*(2*m3 + 3*m4 + 3*m5))/3,                                                                                                                           0,                                                                                     (L3*cos(theta3)*(2*m3 + 3*m4 + 3*m5))/3,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           -sin(theta4)*(I43*cos(theta4)*sin(theta3) - I42*cos(theta4)*sin(theta3) - I52*cos(theta4)*sin(theta3) - I51*cos(theta3)*cos(theta5)*sin(theta5) + I53*cos(theta3)*cos(theta5)*sin(theta5) + I53*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + I51*cos(theta4)*sin(theta3)*sin(theta5)*sin(theta5)),                                                                                                                                                                                                                                                                                                                                                                   I32 + I42/2 + I43/2 + I51/4 + I52/2 + I53/4 + (4*L3*L3*m3)/9 + L3*L3*m4 + L3*L3*m5 + (I42*cos(2*theta4))/2 - (I43*cos(2*theta4))/2 - (I51*cos(2*theta4))/4 - (I51*cos(2*theta5))/4 + (I52*cos(2*theta4))/2 - (I53*cos(2*theta4))/4 + (I53*cos(2*theta5))/4 + (I51*cos(2*theta4)*cos(2*theta5))/4 - (I53*cos(2*theta4)*cos(2*theta5))/4 + (L3*Lb*m3*cos(theta3))/3 + (L3*Lb*m4*cos(theta3))/2 + (L3*Lb*m5*cos(theta3))/2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       -sin(theta4)*(I43*cos(theta3)*cos(theta4) - I42*cos(theta3)*cos(theta4) - I52*cos(theta3)*cos(theta4) + I51*cos(theta5)*sin(theta3)*sin(theta5) - I53*cos(theta5)*sin(theta3)*sin(theta5) + I53*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5) + I51*cos(theta3)*cos(theta4)*sin(theta5)*sin(theta5)),               0,                                                 0,                                                                                                                          I32 + I42 + I52 + (4*L3*L3*m3)/9 + L3*L3*m4 + L3*L3*m5 - I42*sin(theta4)*sin(theta4) + I43*sin(theta4)*sin(theta4) - I52*sin(theta4)*sin(theta4) + I53*sin(theta4)*sin(theta4) + I51*sin(theta4)*sin(theta4)*sin(theta5)*sin(theta5) - I53*sin(theta4)*sin(theta4)*sin(theta5)*sin(theta5),                                                                                 sin(2*theta5)*sin(theta4)*(I51/2 - I53/2),             I52*cos(theta4),
                                        0,                                                                                                                           0,                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          I41*cos(theta3) + I53*sin(theta5)*(cos(theta3)*sin(theta5) + cos(theta4)*cos(theta5)*sin(theta3)) + I51*cos(theta5)*(cos(theta3)*cos(theta5) - cos(theta4)*sin(theta3)*sin(theta5)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        cos(theta5)*sin(theta4)*sin(theta5)*(I51 - I53),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    - I41*sin(theta3) - I53*sin(theta5)*(sin(theta3)*sin(theta5) - cos(theta3)*cos(theta4)*cos(theta5)) - I51*cos(theta5)*(cos(theta5)*sin(theta3) + cos(theta3)*cos(theta4)*sin(theta5)),               0,                                                 0,                                                                                                                                                                                                                                                                                  cos(theta5)*sin(theta4)*sin(theta5)*(I51 - I53),                                                                         I41 + I51 - I51*sin(theta5)*sin(theta5) + I53*sin(theta5)*sin(theta5),                           0,
                                        0,                                                                                                                           0,                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  I52*sin(theta3)*sin(theta4),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        I52*cos(theta4),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              I52*cos(theta3)*sin(theta4),               0,                                                 0,                                                                                                                                                                                                                                                                                                                  I52*cos(theta4),                                                                                                                         0,                         I52;

        C<<-m4*theta3_diff*vb3 - m5*theta3_diff*vb3 - m3*theta3_diff*vb3 + L3*m5*theta4_diff*wb1 - (Lb*m3*theta3_diff*wb2)/2 - (Lb*m4*theta3_diff*wb2)/2 - (Lb*m5*theta3_diff*wb2)/2 - m1*theta1_diff*vb3*cos(theta2) - m5*theta5_diff*vb3*cos(theta4) + m1*theta1_diff*vb2*sin(theta2) - m4*theta4_diff*vb2*sin(theta3) - m5*theta4_diff*vb2*sin(theta3) - L3*m4*theta4_diff*wb1*cos(theta3)*cos(theta3) - L3*m5*theta4_diff*wb1*cos(theta3)*cos(theta3) + (L3*m4*theta4_diff*wb3*sin(2*theta3))/2 + (L3*m5*theta4_diff*wb3*sin(2*theta3))/2 + m5*theta5_diff*vb2*cos(theta3)*sin(theta4) + (Lb*m1*theta1_diff*wb2*cos(theta2))/2 - (Lb*m5*theta5_diff*wb2*cos(theta4))/2 + (Lb*m1*theta1_diff*wb3*sin(theta2))/2 + (Lb*m4*theta4_diff*wb3*sin(theta3))/2 + (Lb*m5*theta4_diff*wb3*sin(theta3))/2 - L3*m5*theta5_diff*wb3*cos(theta3)*cos(theta3)*sin(theta4) - L3*m5*theta3_diff*theta5_diff*cos(theta3)*cos(theta4) - L3*m5*theta5_diff*wb2*cos(theta3)*cos(theta4) - (Lb*m5*theta5_diff*wb3*cos(theta3)*sin(theta4))/2 - L3*m5*theta5_diff*wb1*cos(theta3)*sin(theta3)*sin(theta4),
            m1*theta2_diff*vb3 + m2*theta2_diff*vb3 + L3*m4*theta3_diff*theta4_diff + L3*m5*theta3_diff*theta4_diff + L3*m4*theta4_diff*wb2 + L3*m5*theta4_diff*wb2 - (Lb*m1*theta2_diff*wb2)/2 - (Lb*m2*theta2_diff*wb2)/2 + m4*theta4_diff*vb3*cos(theta3) + m5*theta4_diff*vb3*cos(theta3) - m1*theta1_diff*vb1*sin(theta2) + m4*theta4_diff*vb1*sin(theta3) + m5*theta4_diff*vb1*sin(theta3) - m5*theta5_diff*vb1*cos(theta3)*sin(theta4) + m5*theta5_diff*vb3*sin(theta3)*sin(theta4) - (2*L3*m3*theta3_diff*wb1*cos(theta3))/3 - L3*m4*theta3_diff*wb1*cos(theta3) - L3*m5*theta3_diff*wb1*cos(theta3) + (Lb*m4*theta4_diff*wb2*cos(theta3))/2 + (Lb*m5*theta4_diff*wb2*cos(theta3))/2 + (2*L3*m3*theta3_diff*wb3*sin(theta3))/3 + L3*m4*theta3_diff*wb3*sin(theta3) + L3*m5*theta3_diff*wb3*sin(theta3) + (Lb*m5*theta5_diff*wb2*sin(theta3)*sin(theta4))/2,
            m3*theta3_diff*vb1 - m2*theta2_diff*vb2 - m1*theta2_diff*vb2 + m4*theta3_diff*vb1 + m5*theta3_diff*vb1 - (Lb*m1*theta2_diff*wb3)/2 - (Lb*m2*theta2_diff*wb3)/2 + m1*theta1_diff*vb1*cos(theta2) - m4*theta4_diff*vb2*cos(theta3) - m5*theta4_diff*vb2*cos(theta3) + m5*theta5_diff*vb1*cos(theta4) + L3*m4*theta4_diff*wb3*cos(theta3)*cos(theta3) + L3*m5*theta4_diff*wb3*cos(theta3)*cos(theta3) + (L3*m4*theta4_diff*wb1*sin(2*theta3))/2 + (L3*m5*theta4_diff*wb1*sin(2*theta3))/2 - m5*theta5_diff*vb2*sin(theta3)*sin(theta4) + (Lb*m4*theta4_diff*wb3*cos(theta3))/2 + (Lb*m5*theta4_diff*wb3*cos(theta3))/2 + L3*m5*theta5_diff*wb1*sin(theta4) - L3*m5*theta5_diff*wb1*cos(theta3)*cos(theta3)*sin(theta4) + L3*m5*theta3_diff*theta5_diff*cos(theta4)*sin(theta3) + L3*m5*theta5_diff*wb2*cos(theta4)*sin(theta3) + (Lb*m5*theta5_diff*wb3*sin(theta3)*sin(theta4))/2 + L3*m5*theta5_diff*wb3*cos(theta3)*sin(theta3)*sin(theta4),
            I31*theta3_diff*wb3 - I32*theta3_diff*wb3 - I33*theta3_diff*wb3 + I41*theta3_diff*wb3 - I42*theta3_diff*wb3 - I43*theta3_diff*wb3 - I51*theta3_diff*wb3 - I52*theta3_diff*wb3 + I53*theta3_diff*wb3 - I11*wb2*wb3 + I12*wb2*wb3 + I22*wb2*wb3 - I23*wb2*wb3 + I31*wb2*wb3 - I32*wb2*wb3 + I41*wb2*wb3 - I43*wb2*wb3 - I51*wb2*wb3 + I53*wb2*wb3 - Ib2*wb2*wb3 + Ib3*wb2*wb3 - (I11*wb2*wb2*sin(2*theta2))/2 + (I11*wb3*wb3*sin(2*theta2))/2 + (I12*wb2*wb2*sin(2*theta2))/2 - (I12*wb3*wb3*sin(2*theta2))/2 + (I22*wb2*wb2*sin(2*theta2))/2 - (I22*wb3*wb3*sin(2*theta2))/2 - (I23*wb2*wb2*sin(2*theta2))/2 + (I23*wb3*wb3*sin(2*theta2))/2 - L3*m4*theta4_diff*vb1 - L3*m5*theta4_diff*vb1 - (4*L3*L3*m3*theta3_diff*wb3)/9 - L3*L3*m4*theta3_diff*wb3 - L3*L3*m5*theta3_diff*wb3 + I11*theta1_diff*wb3*cos(theta2) - I12*theta1_diff*wb3*cos(theta2) - I13*theta1_diff*wb3*cos(theta2) - I51*theta5_diff*wb3*cos(theta4) - I52*theta5_diff*wb3*cos(theta4) + I53*theta5_diff*wb3*cos(theta4) - I41*theta3_diff*theta4_diff*sin(theta3) - I42*theta3_diff*theta4_diff*sin(theta3) + I43*theta3_diff*theta4_diff*sin(theta3) + I51*theta3_diff*theta4_diff*sin(theta3) - I52*theta3_diff*theta4_diff*sin(theta3) - I53*theta3_diff*theta4_diff*sin(theta3) - I11*theta1_diff*wb2*sin(theta2) + I12*theta1_diff*wb2*sin(theta2) + I13*theta1_diff*wb2*sin(theta2) - I41*theta4_diff*wb2*sin(theta3) - I42*theta4_diff*wb2*sin(theta3) + I43*theta4_diff*wb2*sin(theta3) + I51*theta4_diff*wb2*sin(theta3) - I52*theta4_diff*wb2*sin(theta3) - I53*theta4_diff*wb2*sin(theta3) - 2*I31*theta3_diff*wb3*cos(theta3)*cos(theta3) + 2*I33*theta3_diff*wb3*cos(theta3)*cos(theta3) - 2*I41*theta3_diff*wb3*cos(theta3)*cos(theta3) + 2*I42*theta3_diff*wb3*cos(theta3)*cos(theta3) + 2*I52*theta3_diff*wb3*cos(theta3)*cos(theta3) + 2*I51*theta3_diff*wb3*cos(theta5)*cos(theta5) - 2*I53*theta3_diff*wb3*cos(theta3)*cos(theta3) - 2*I53*theta3_diff*wb3*cos(theta5)*cos(theta5) + I11*wb2*wb3*cos(theta1)*cos(theta1) + 2*I11*wb2*wb3*cos(theta2)*cos(theta2) - 2*I12*wb2*wb3*cos(theta2)*cos(theta2) - I13*wb2*wb3*cos(theta1)*cos(theta1) - 2*I22*wb2*wb3*cos(theta2)*cos(theta2) + 2*I23*wb2*wb3*cos(theta2)*cos(theta2) - I31*wb2*wb3*cos(theta3)*cos(theta3) + I33*wb2*wb3*cos(theta3)*cos(theta3) - I41*wb2*wb3*cos(theta3)*cos(theta3) + I42*wb2*wb3*cos(theta3)*cos(theta3) - I42*wb2*wb3*cos(theta4)*cos(theta4) + I43*wb2*wb3*cos(theta4)*cos(theta4) + I51*wb2*wb3*cos(theta4)*cos(theta4) + I52*wb2*wb3*cos(theta3)*cos(theta3) + 2*I51*wb2*wb3*cos(theta5)*cos(theta5) - I52*wb2*wb3*cos(theta4)*cos(theta4) - I53*wb2*wb3*cos(theta3)*cos(theta3) - 2*I53*wb2*wb3*cos(theta5)*cos(theta5) - I11*theta1_diff*theta2_diff*sin(2*theta1) + I13*theta1_diff*theta2_diff*sin(2*theta1) - I11*theta1_diff*wb1*sin(2*theta1) + I13*theta1_diff*wb1*sin(2*theta1) - I31*theta3_diff*wb1*sin(2*theta3) + I33*theta3_diff*wb1*sin(2*theta3) - I41*theta3_diff*wb1*sin(2*theta3) + I42*theta3_diff*wb1*sin(2*theta3) + I42*theta4_diff*wb1*sin(2*theta4) - I43*theta4_diff*wb1*sin(2*theta4) + I52*theta3_diff*wb1*sin(2*theta3) - I51*theta4_diff*wb1*sin(2*theta4) - I53*theta3_diff*wb1*sin(2*theta3) + I52*theta4_diff*wb1*sin(2*theta4) - (I31*wb1*wb2*sin(2*theta3))/2 + (I33*wb1*wb2*sin(2*theta3))/2 - (I41*wb1*wb2*sin(2*theta3))/2 + (I42*wb1*wb2*sin(2*theta3))/2 + (I52*wb1*wb2*sin(2*theta3))/2 - (I53*wb1*wb2*sin(2*theta3))/2 - 2*I42*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + 2*I43*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + 2*I51*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) - 2*I51*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) - 2*I52*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + 2*I53*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) - 2*I11*wb2*wb3*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) + 2*I13*wb2*wb3*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) - I42*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + I43*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + I51*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) - I51*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) - I52*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) - I51*wb2*wb3*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I53*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) + I53*wb2*wb3*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I42*theta3_diff*theta3_diff*cos(theta3)*cos(theta4)*sin(theta4) - I43*theta3_diff*theta3_diff*cos(theta3)*cos(theta4)*sin(theta4) - I51*theta3_diff*theta3_diff*cos(theta3)*cos(theta4)*sin(theta4) + I52*theta3_diff*theta3_diff*cos(theta3)*cos(theta4)*sin(theta4) + I42*wb2*wb2*cos(theta3)*cos(theta4)*sin(theta4) - I42*wb3*wb3*cos(theta3)*cos(theta4)*sin(theta4) - I43*wb2*wb2*cos(theta3)*cos(theta4)*sin(theta4) + I43*wb3*wb3*cos(theta3)*cos(theta4)*sin(theta4) - I51*wb2*wb2*cos(theta3)*cos(theta4)*sin(theta4) + I51*wb3*wb3*cos(theta3)*cos(theta4)*sin(theta4) + I52*wb2*wb2*cos(theta3)*cos(theta4)*sin(theta4) - I52*wb3*wb3*cos(theta3)*cos(theta4)*sin(theta4) + L3*m4*theta4_diff*vb1*cos(theta3)*cos(theta3) + L3*m5*theta4_diff*vb1*cos(theta3)*cos(theta3) - L3*L3*m4*theta3_diff*theta4_diff*sin(theta3) - L3*L3*m5*theta3_diff*theta4_diff*sin(theta3) - (L3*m4*theta4_diff*vb3*sin(2*theta3))/2 - (L3*m5*theta4_diff*vb3*sin(2*theta3))/2 - L3*L3*m4*theta4_diff*wb2*sin(theta3) - L3*L3*m5*theta4_diff*wb2*sin(theta3) - I51*theta3_diff*theta5_diff*cos(theta3)*sin(theta4) + I51*theta4_diff*theta5_diff*cos(theta4)*sin(theta3) + I52*theta3_diff*theta5_diff*cos(theta3)*sin(theta4) + I52*theta4_diff*theta5_diff*cos(theta4)*sin(theta3) + I53*theta3_diff*theta5_diff*cos(theta3)*sin(theta4) - I53*theta4_diff*theta5_diff*cos(theta4)*sin(theta3) - I51*theta5_diff*wb2*cos(theta3)*sin(theta4) + I52*theta5_diff*wb2*cos(theta3)*sin(theta4) + I53*theta5_diff*wb2*cos(theta3)*sin(theta4) + I11*wb2*wb2*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) - I11*wb3*wb3*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) - I13*wb2*wb2*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) + I13*wb3*wb3*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) + (4*L3*L3*m3*theta3_diff*wb3*cos(theta3)*cos(theta3))/9 + L3*L3*m4*theta3_diff*wb3*cos(theta3)*cos(theta3) + L3*L3*m5*theta3_diff*wb3*cos(theta3)*cos(theta3) - 2*I11*theta1_diff*wb3*cos(theta1)*cos(theta1)*cos(theta2) + 2*I13*theta1_diff*wb3*cos(theta1)*cos(theta1)*cos(theta2) + 2*I51*theta5_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4) + 2*I51*theta5_diff*wb3*cos(theta4)*cos(theta5)*cos(theta5) - 2*I53*theta5_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4) - 2*I53*theta5_diff*wb3*cos(theta4)*cos(theta5)*cos(theta5) + (2*L3*L3*m3*theta3_diff*wb1*sin(2*theta3))/9 + (L3*L3*m4*theta3_diff*wb1*sin(2*theta3))/2 + (L3*L3*m5*theta3_diff*wb1*sin(2*theta3))/2 + 2*I42*theta3_diff*theta4_diff*cos(theta4)*cos(theta4)*sin(theta3) - 2*I43*theta3_diff*theta4_diff*cos(theta4)*cos(theta4)*sin(theta3) - 2*I51*theta3_diff*theta4_diff*cos(theta4)*cos(theta4)*sin(theta3) - 2*I51*theta3_diff*theta4_diff*cos(theta5)*cos(theta5)*sin(theta3) + 2*I52*theta3_diff*theta4_diff*cos(theta4)*cos(theta4)*sin(theta3) + 2*I53*theta3_diff*theta4_diff*cos(theta5)*cos(theta5)*sin(theta3) + 2*I11*theta1_diff*wb2*cos(theta1)*cos(theta1)*sin(theta2) - 2*I13*theta1_diff*wb2*cos(theta1)*cos(theta1)*sin(theta2) + 2*I42*theta4_diff*wb2*cos(theta4)*cos(theta4)*sin(theta3) - 2*I43*theta4_diff*wb2*cos(theta4)*cos(theta4)*sin(theta3) - 2*I51*theta4_diff*wb2*cos(theta4)*cos(theta4)*sin(theta3) - 2*I51*theta4_diff*wb2*cos(theta5)*cos(theta5)*sin(theta3) + 2*I52*theta4_diff*wb2*cos(theta4)*cos(theta4)*sin(theta3) + 2*I53*theta4_diff*wb2*cos(theta5)*cos(theta5)*sin(theta3) - L3*m5*theta5_diff*vb3*sin(theta4) - I51*theta3_diff*theta3_diff*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + I51*theta4_diff*theta4_diff*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + I53*theta3_diff*theta3_diff*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) - I53*theta4_diff*theta4_diff*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) - I51*wb2*wb2*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + I51*wb3*wb3*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + I53*wb2*wb2*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) - I53*wb3*wb3*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + L3*m5*theta5_diff*vb3*cos(theta3)*cos(theta3)*sin(theta4) - 2*I51*theta4_diff*theta5_diff*cos(theta3)*cos(theta5)*sin(theta5) + 2*I53*theta4_diff*theta5_diff*cos(theta3)*cos(theta5)*sin(theta5) + 2*I42*theta3_diff*wb2*cos(theta3)*cos(theta4)*sin(theta4) - 2*I43*theta3_diff*wb2*cos(theta3)*cos(theta4)*sin(theta4) - 2*I51*theta3_diff*wb2*cos(theta3)*cos(theta4)*sin(theta4) + 2*I51*theta5_diff*wb1*cos(theta3)*cos(theta4)*sin(theta3) + 2*I52*theta3_diff*wb2*cos(theta3)*cos(theta4)*sin(theta4) + 2*I51*theta3_diff*wb1*cos(theta4)*cos(theta5)*sin(theta5) - 2*I53*theta5_diff*wb1*cos(theta3)*cos(theta4)*sin(theta3) - 2*I53*theta3_diff*wb1*cos(theta4)*cos(theta5)*sin(theta5) - I11*wb1*wb2*cos(theta1)*cos(theta2)*sin(theta1) + I13*wb1*wb2*cos(theta1)*cos(theta2)*sin(theta1) + I51*wb1*wb2*cos(theta4)*cos(theta5)*sin(theta5) - I53*wb1*wb2*cos(theta4)*cos(theta5)*sin(theta5) - (L3*Lb*m5*theta5_diff*wb2*sin(theta4))/2 - 2*I51*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + 2*I53*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) - 2*I51*theta4_diff*wb3*cos(theta5)*sin(theta4)*sin(theta5) + 2*I53*theta4_diff*wb3*cos(theta5)*sin(theta4)*sin(theta5) - I51*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I53*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) - I11*wb1*wb3*cos(theta1)*sin(theta1)*sin(theta2) + I13*wb1*wb3*cos(theta1)*sin(theta1)*sin(theta2) - I42*wb1*wb3*cos(theta4)*sin(theta3)*sin(theta4) + I43*wb1*wb3*cos(theta4)*sin(theta3)*sin(theta4) + I51*wb1*wb3*cos(theta4)*sin(theta3)*sin(theta4) - I52*wb1*wb3*cos(theta4)*sin(theta3)*sin(theta4) + I51*theta3_diff*theta3_diff*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - I53*theta3_diff*theta3_diff*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + I51*wb2*wb2*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - I51*wb3*wb3*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - I53*wb2*wb2*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + I53*wb3*wb3*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + 2*I51*theta3_diff*theta5_diff*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I51*theta4_diff*theta5_diff*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I53*theta3_diff*theta5_diff*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta4) + 2*I53*theta4_diff*theta5_diff*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I42*theta3_diff*wb1*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + 2*I43*theta3_diff*wb1*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - 2*I42*theta4_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*sin(theta4) + 2*I43*theta4_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*sin(theta4) + 2*I51*theta3_diff*wb1*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - 2*I51*theta3_diff*wb1*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I52*theta3_diff*wb1*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + 2*I51*theta4_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*sin(theta4) - 2*I52*theta4_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*sin(theta4) + 2*I53*theta3_diff*wb1*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I51*theta4_diff*wb1*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I51*theta5_diff*wb1*cos(theta3)*cos(theta3)*cos(theta5)*sin(theta5) + 2*I51*theta5_diff*wb2*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta4) + 2*I51*theta5_diff*wb1*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) - 2*I53*theta4_diff*wb1*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + 2*I53*theta5_diff*wb1*cos(theta3)*cos(theta3)*cos(theta5)*sin(theta5) - 2*I53*theta5_diff*wb2*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I53*theta5_diff*wb1*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) - I42*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I43*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I51*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I51*wb1*wb2*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) - I52*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I53*wb1*wb2*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) - (L3*Lb*m4*theta4_diff*wb2*sin(2*theta3))/4 - (L3*Lb*m5*theta4_diff*wb2*sin(2*theta3))/4 - 4*I51*theta5_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5) + 4*I53*theta5_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5) + 2*I51*theta3_diff*theta4_diff*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I53*theta3_diff*theta4_diff*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I51*theta4_diff*wb2*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I53*theta4_diff*wb2*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I42*theta4_diff*wb3*cos(theta3)*cos(theta4)*sin(theta3)*sin(theta4) - 2*I43*theta4_diff*wb3*cos(theta3)*cos(theta4)*sin(theta3)*sin(theta4) - 2*I51*theta4_diff*wb3*cos(theta3)*cos(theta4)*sin(theta3)*sin(theta4) + 2*I52*theta4_diff*wb3*cos(theta3)*cos(theta4)*sin(theta3)*sin(theta4) + 2*I51*theta5_diff*wb3*cos(theta3)*cos(theta5)*sin(theta3)*sin(theta5) - 2*I53*theta5_diff*wb3*cos(theta3)*cos(theta5)*sin(theta3)*sin(theta5) - I51*wb1*wb3*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) + I53*wb1*wb3*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - 2*I51*theta3_diff*wb2*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + 2*I53*theta3_diff*wb2*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) - 4*I51*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) + 2*I51*theta3_diff*wb2*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - 4*I51*theta5_diff*wb1*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + 4*I53*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) - 2*I53*theta3_diff*wb2*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + 4*I53*theta5_diff*wb1*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I51*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) + 2*I53*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) + (L3*Lb*m5*theta5_diff*wb2*cos(theta3)*cos(theta3)*sin(theta4))/2 + 2*I51*theta4_diff*wb3*cos(theta3)*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - 2*I53*theta4_diff*wb3*cos(theta3)*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - I51*wb1*wb3*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + I53*wb1*wb3*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + L3*m5*theta5_diff*vb1*cos(theta3)*sin(theta3)*sin(theta4) - 2*I51*theta3_diff*wb1*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I51*theta4_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + 2*I53*theta3_diff*wb1*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I51*theta5_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) + 2*I53*theta4_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + 2*I53*theta5_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) - I51*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + I53*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I51*theta4_diff*wb3*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + 2*I51*theta5_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 2*I53*theta4_diff*wb3*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - 2*I53*theta5_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) + 4*I51*theta3_diff*wb3*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 4*I53*theta3_diff*wb3*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) + 2*I51*wb2*wb3*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 2*I53*wb2*wb3*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 2*I51*theta3_diff*theta5_diff*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + 2*I53*theta3_diff*theta5_diff*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + 2*I51*theta4_diff*wb1*cos(theta3)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) - 2*I53*theta4_diff*wb1*cos(theta3)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) - 2*I51*theta5_diff*wb2*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + 2*I53*theta5_diff*wb2*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5),
            I11*theta2_diff*wb3 - I12*theta2_diff*wb3 + I13*theta2_diff*wb3 + I21*theta2_diff*wb3 - I22*theta2_diff*wb3 + I23*theta2_diff*wb3 - I12*wb1*wb3 + I13*wb1*wb3 + I21*wb1*wb3 - I22*wb1*wb3 - I31*wb1*wb3 + I33*wb1*wb3 - I41*wb1*wb3 + I42*wb1*wb3 + I52*wb1*wb3 - I53*wb1*wb3 + Ib1*wb1*wb3 - Ib3*wb1*wb3 + (I31*wb1*wb1*sin(2*theta3))/2 - (I31*wb3*wb3*sin(2*theta3))/2 - (I33*wb1*wb1*sin(2*theta3))/2 + (I33*wb3*wb3*sin(2*theta3))/2 + (I41*wb1*wb1*sin(2*theta3))/2 - (I42*wb1*wb1*sin(2*theta3))/2 - (I41*wb3*wb3*sin(2*theta3))/2 + (I42*wb3*wb3*sin(2*theta3))/2 - (I52*wb1*wb1*sin(2*theta3))/2 + (I53*wb1*wb1*sin(2*theta3))/2 + (I52*wb3*wb3*sin(2*theta3))/2 - (I53*wb3*wb3*sin(2*theta3))/2 - L3*m4*theta4_diff*vb2 - L3*m5*theta4_diff*vb2 + (Lb*m1*theta2_diff*vb2)/2 + (Lb*m2*theta2_diff*vb2)/2 + (Lb*m3*theta3_diff*vb1)/2 + (Lb*m4*theta3_diff*vb1)/2 + (Lb*m5*theta3_diff*vb1)/2 + (Lb*Lb*m1*theta2_diff*wb3)/4 + (Lb*Lb*m2*theta2_diff*wb3)/4 + I41*theta4_diff*wb3*cos(theta3) - I42*theta4_diff*wb3*cos(theta3) + I43*theta4_diff*wb3*cos(theta3) + I51*theta4_diff*wb3*cos(theta3) - I52*theta4_diff*wb3*cos(theta3) + I53*theta4_diff*wb3*cos(theta3) - I11*theta1_diff*theta2_diff*sin(theta2) - I12*theta1_diff*theta2_diff*sin(theta2) + I13*theta1_diff*theta2_diff*sin(theta2) - I51*theta4_diff*theta5_diff*sin(theta4) - I52*theta4_diff*theta5_diff*sin(theta4) + I53*theta4_diff*theta5_diff*sin(theta4) - I11*theta1_diff*wb1*sin(theta2) - I12*theta1_diff*wb1*sin(theta2) + I13*theta1_diff*wb1*sin(theta2) + I41*theta4_diff*wb1*sin(theta3) - I42*theta4_diff*wb1*sin(theta3) + I43*theta4_diff*wb1*sin(theta3) + I51*theta4_diff*wb1*sin(theta3) - I52*theta4_diff*wb1*sin(theta3) + I53*theta4_diff*wb1*sin(theta3) - 2*I11*theta2_diff*wb3*cos(theta2)*cos(theta2) + 2*I12*theta2_diff*wb3*cos(theta2)*cos(theta2) + 2*I22*theta2_diff*wb3*cos(theta2)*cos(theta2) - 2*I23*theta2_diff*wb3*cos(theta2)*cos(theta2) + I11*wb1*wb3*cos(theta1)*cos(theta1) - I11*wb1*wb3*cos(theta2)*cos(theta2) + I12*wb1*wb3*cos(theta2)*cos(theta2) - I13*wb1*wb3*cos(theta1)*cos(theta1) + I22*wb1*wb3*cos(theta2)*cos(theta2) - I23*wb1*wb3*cos(theta2)*cos(theta2) + 2*I31*wb1*wb3*cos(theta3)*cos(theta3) - 2*I33*wb1*wb3*cos(theta3)*cos(theta3) + 2*I41*wb1*wb3*cos(theta3)*cos(theta3) - 2*I42*wb1*wb3*cos(theta3)*cos(theta3) - I42*wb1*wb3*cos(theta4)*cos(theta4) + I43*wb1*wb3*cos(theta4)*cos(theta4) + I51*wb1*wb3*cos(theta4)*cos(theta4) - 2*I52*wb1*wb3*cos(theta3)*cos(theta3) - I51*wb1*wb3*cos(theta5)*cos(theta5) - I52*wb1*wb3*cos(theta4)*cos(theta4) + 2*I53*wb1*wb3*cos(theta3)*cos(theta3) + I53*wb1*wb3*cos(theta5)*cos(theta5) - I42*theta3_diff*theta4_diff*sin(2*theta4) + I43*theta3_diff*theta4_diff*sin(2*theta4) + I51*theta3_diff*theta4_diff*sin(2*theta4) - I52*theta3_diff*theta4_diff*sin(2*theta4) + I51*theta3_diff*theta5_diff*sin(2*theta5) - I53*theta3_diff*theta5_diff*sin(2*theta5) + I11*theta1_diff*wb2*sin(2*theta1) + I11*theta2_diff*wb2*sin(2*theta2) - I13*theta1_diff*wb2*sin(2*theta1) - I12*theta2_diff*wb2*sin(2*theta2) - I22*theta2_diff*wb2*sin(2*theta2) + I23*theta2_diff*wb2*sin(2*theta2) - I42*theta4_diff*wb2*sin(2*theta4) + I43*theta4_diff*wb2*sin(2*theta4) + I51*theta4_diff*wb2*sin(2*theta4) - I52*theta4_diff*wb2*sin(2*theta4) + I51*theta5_diff*wb2*sin(2*theta5) - I53*theta5_diff*wb2*sin(2*theta5) + (I11*wb1*wb2*sin(2*theta2))/2 - (I12*wb1*wb2*sin(2*theta2))/2 - (I22*wb1*wb2*sin(2*theta2))/2 + (I23*wb1*wb2*sin(2*theta2))/2 + 2*I11*theta2_diff*wb3*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) - 2*I13*theta2_diff*wb3*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) + I11*wb1*wb3*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) - I13*wb1*wb3*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) + 2*I42*wb1*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) - 2*I43*wb1*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) - 2*I51*wb1*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + 2*I51*wb1*wb3*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) + 2*I52*wb1*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) - I51*wb1*wb3*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) - 2*I53*wb1*wb3*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) + I53*wb1*wb3*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I11*theta2_diff*theta2_diff*cos(theta1)*cos(theta2)*sin(theta1) - I13*theta2_diff*theta2_diff*cos(theta1)*cos(theta2)*sin(theta1) + I51*theta4_diff*theta4_diff*cos(theta4)*cos(theta5)*sin(theta5) - I53*theta4_diff*theta4_diff*cos(theta4)*cos(theta5)*sin(theta5) + I11*wb1*wb1*cos(theta1)*cos(theta2)*sin(theta1) - I11*wb3*wb3*cos(theta1)*cos(theta2)*sin(theta1) - I13*wb1*wb1*cos(theta1)*cos(theta2)*sin(theta1) + I13*wb3*wb3*cos(theta1)*cos(theta2)*sin(theta1) - I51*wb1*wb1*cos(theta4)*cos(theta5)*sin(theta5) + I51*wb3*wb3*cos(theta4)*cos(theta5)*sin(theta5) + I53*wb1*wb1*cos(theta4)*cos(theta5)*sin(theta5) - I53*wb3*wb3*cos(theta4)*cos(theta5)*sin(theta5) + L3*L3*m4*theta4_diff*wb3*cos(theta3) + L3*L3*m5*theta4_diff*wb3*cos(theta3) + (Lb*Lb*m4*theta4_diff*wb3*cos(theta3))/4 + (Lb*Lb*m5*theta4_diff*wb3*cos(theta3))/4 + L3*L3*m4*theta4_diff*wb1*sin(theta3) + L3*L3*m5*theta4_diff*wb1*sin(theta3) + (L3*Lb*m4*theta4_diff*wb3)/2 + (L3*Lb*m5*theta4_diff*wb3)/2 - I51*theta5_diff*wb1*cos(theta3)*sin(theta4) - I52*theta5_diff*wb1*cos(theta3)*sin(theta4) + I53*theta5_diff*wb1*cos(theta3)*sin(theta4) + I51*theta5_diff*wb3*sin(theta3)*sin(theta4) + I52*theta5_diff*wb3*sin(theta3)*sin(theta4) - I53*theta5_diff*wb3*sin(theta3)*sin(theta4) + I42*wb1*wb1*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I43*wb1*wb1*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I42*wb3*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I43*wb3*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I51*wb1*wb1*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I51*wb1*wb1*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) + I52*wb1*wb1*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I51*wb3*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I51*wb3*wb3*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) - I52*wb3*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I53*wb1*wb1*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) + I53*wb3*wb3*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I42*theta4_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4) - 2*I43*theta4_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4) - 2*I51*theta4_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4) + 2*I52*theta4_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4) + 2*I11*theta1_diff*theta2_diff*cos(theta1)*cos(theta1)*sin(theta2) - 2*I13*theta1_diff*theta2_diff*cos(theta1)*cos(theta1)*sin(theta2) + 2*I51*theta4_diff*theta5_diff*cos(theta5)*cos(theta5)*sin(theta4) - 2*I53*theta4_diff*theta5_diff*cos(theta5)*cos(theta5)*sin(theta4) + 2*I11*theta1_diff*wb1*cos(theta1)*cos(theta1)*sin(theta2) - 2*I13*theta1_diff*wb1*cos(theta1)*cos(theta1)*sin(theta2) + 2*I42*theta4_diff*wb1*cos(theta4)*cos(theta4)*sin(theta3) - 2*I43*theta4_diff*wb1*cos(theta4)*cos(theta4)*sin(theta3) - 2*I51*theta4_diff*wb1*cos(theta4)*cos(theta4)*sin(theta3) + 2*I52*theta4_diff*wb1*cos(theta4)*cos(theta4)*sin(theta3) + (2*L3*m3*theta3_diff*vb1*cos(theta3))/3 + L3*m4*theta3_diff*vb1*cos(theta3) + L3*m5*theta3_diff*vb1*cos(theta3) - (Lb*m1*theta1_diff*vb1*cos(theta2))/2 - (Lb*m4*theta4_diff*vb2*cos(theta3))/2 - (Lb*m5*theta4_diff*vb2*cos(theta3))/2 + (Lb*m5*theta5_diff*vb1*cos(theta4))/2 - (2*L3*m3*theta3_diff*vb3*sin(theta3))/3 - L3*m4*theta3_diff*vb3*sin(theta3) - L3*m5*theta3_diff*vb3*sin(theta3) + 2*I11*theta2_diff*wb1*cos(theta1)*cos(theta2)*sin(theta1) - 2*I13*theta2_diff*wb1*cos(theta1)*cos(theta2)*sin(theta1) - I42*wb1*wb2*cos(theta3)*cos(theta4)*sin(theta4) + I43*wb1*wb2*cos(theta3)*cos(theta4)*sin(theta4) + I51*wb1*wb2*cos(theta3)*cos(theta4)*sin(theta4) - I52*wb1*wb2*cos(theta3)*cos(theta4)*sin(theta4) + (Lb*Lb*m5*theta5_diff*wb3*sin(theta3)*sin(theta4))/4 - (L3*Lb*m3*theta3_diff*wb2*sin(theta3))/3 - (L3*Lb*m4*theta3_diff*wb2*sin(theta3))/2 - (L3*Lb*m5*theta3_diff*wb2*sin(theta3))/2 + (L3*Lb*m5*theta5_diff*wb1*sin(theta4))/2 + 2*I51*wb1*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) - 2*I53*wb1*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I11*wb2*wb3*cos(theta1)*sin(theta1)*sin(theta2) - I13*wb2*wb3*cos(theta1)*sin(theta1)*sin(theta2) + I42*wb2*wb3*cos(theta4)*sin(theta3)*sin(theta4) - I43*wb2*wb3*cos(theta4)*sin(theta3)*sin(theta4) - I51*wb2*wb3*cos(theta4)*sin(theta3)*sin(theta4) + I52*wb2*wb3*cos(theta4)*sin(theta3)*sin(theta4) + 2*I51*wb1*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) - 2*I51*wb3*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) - 2*I53*wb1*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) + 2*I53*wb3*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) + (L3*Lb*m4*theta4_diff*wb3*cos(theta3)*cos(theta3))/2 + (L3*Lb*m5*theta4_diff*wb3*cos(theta3)*cos(theta3))/2 - 2*I51*theta3_diff*theta4_diff*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I51*theta3_diff*theta5_diff*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) + 2*I53*theta3_diff*theta4_diff*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + 2*I53*theta3_diff*theta5_diff*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) - 2*I11*theta1_diff*wb2*cos(theta1)*cos(theta2)*cos(theta2)*sin(theta1) - 2*I11*theta2_diff*wb2*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) + 2*I13*theta1_diff*wb2*cos(theta1)*cos(theta2)*cos(theta2)*sin(theta1) + 2*I13*theta2_diff*wb2*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) + 2*I51*theta5_diff*wb1*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I51*theta4_diff*wb2*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I53*theta5_diff*wb1*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I51*theta5_diff*wb2*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) + 2*I53*theta4_diff*wb2*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + 2*I53*theta5_diff*wb2*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) - I11*wb1*wb2*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) + I13*wb1*wb2*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) + (L3*Lb*m4*theta4_diff*wb1*sin(2*theta3))/4 + (L3*Lb*m5*theta4_diff*wb1*sin(2*theta3))/4 - 2*I51*theta5_diff*wb3*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + 2*I53*theta5_diff*wb3*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + I51*wb1*wb1*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - I51*wb3*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - I53*wb1*wb1*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + I53*wb3*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + L3*m5*theta5_diff*vb1*cos(theta3)*cos(theta4) - L3*m5*theta5_diff*vb3*cos(theta4)*sin(theta3) - (Lb*m5*theta5_diff*vb2*sin(theta3)*sin(theta4))/2 + 2*I51*theta4_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) - 2*I53*theta4_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + 2*I51*theta4_diff*wb1*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I53*theta4_diff*wb1*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I51*theta4_diff*wb1*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) - 2*I53*theta4_diff*wb1*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) + (L3*Lb*m5*theta3_diff*theta5_diff*cos(theta4)*sin(theta3))/2 - 2*I11*theta1_diff*wb3*cos(theta1)*cos(theta2)*sin(theta1)*sin(theta2) + 2*I13*theta1_diff*wb3*cos(theta1)*cos(theta2)*sin(theta1)*sin(theta2) - 2*I51*theta4_diff*wb3*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) + 2*I53*theta4_diff*wb3*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) + I51*wb2*wb3*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - I53*wb2*wb3*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) + I51*wb1*wb2*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) - I53*wb1*wb2*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) - I51*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + I53*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - (L3*Lb*m5*theta5_diff*wb1*cos(theta3)*cos(theta3)*sin(theta4))/2 + I51*wb2*wb3*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - I53*wb2*wb3*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - 2*I51*theta5_diff*wb3*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta4)*sin(theta5) + 2*I53*theta5_diff*wb3*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta4)*sin(theta5) - 4*I51*wb1*wb3*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) + 4*I53*wb1*wb3*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) + (L3*Lb*m5*theta5_diff*wb3*cos(theta3)*sin(theta3)*sin(theta4))/2 - 2*I51*theta5_diff*wb1*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + 2*I53*theta5_diff*wb1*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5),
            I11*theta2_diff*wb2 - I12*theta2_diff*wb2 - I13*theta2_diff*wb2 - I21*theta2_diff*wb2 - I22*theta2_diff*wb2 + I23*theta2_diff*wb2 + I31*theta3_diff*wb1 + I32*theta3_diff*wb1 - I33*theta3_diff*wb1 + I41*theta3_diff*wb1 - I42*theta3_diff*wb1 + I43*theta3_diff*wb1 + I51*theta3_diff*wb1 - I52*theta3_diff*wb1 + I53*theta3_diff*wb1 + I11*wb1*wb2 - I13*wb1*wb2 - I21*wb1*wb2 + I23*wb1*wb2 + I32*wb1*wb2 - I33*wb1*wb2 - I42*wb1*wb2 + I43*wb1*wb2 + I51*wb1*wb2 - I52*wb1*wb2 - Ib1*wb1*wb2 + Ib2*wb1*wb2 + (Lb*m1*theta2_diff*vb3)/2 + (Lb*m2*theta2_diff*vb3)/2 - (Lb*Lb*m1*theta2_diff*wb2)/4 - (Lb*Lb*m2*theta2_diff*wb2)/4 + I11*theta1_diff*theta2_diff*cos(theta2) + I12*theta1_diff*theta2_diff*cos(theta2) - I13*theta1_diff*theta2_diff*cos(theta2) - I41*theta3_diff*theta4_diff*cos(theta3) - I42*theta3_diff*theta4_diff*cos(theta3) + I43*theta3_diff*theta4_diff*cos(theta3) + I51*theta3_diff*theta4_diff*cos(theta3) - I52*theta3_diff*theta4_diff*cos(theta3) - I53*theta3_diff*theta4_diff*cos(theta3) + I11*theta1_diff*wb1*cos(theta2) + I12*theta1_diff*wb1*cos(theta2) - I13*theta1_diff*wb1*cos(theta2) - I41*theta4_diff*wb2*cos(theta3) - I42*theta4_diff*wb2*cos(theta3) + I43*theta4_diff*wb2*cos(theta3) + I51*theta4_diff*wb2*cos(theta3) - I51*theta5_diff*wb1*cos(theta4) - I52*theta4_diff*wb2*cos(theta3) + I52*theta5_diff*wb1*cos(theta4) - I53*theta4_diff*wb2*cos(theta3) + I53*theta5_diff*wb1*cos(theta4) - 2*I11*theta2_diff*wb2*cos(theta1)*cos(theta1) - 2*I11*theta2_diff*wb2*cos(theta2)*cos(theta2) + 2*I12*theta2_diff*wb2*cos(theta2)*cos(theta2) + 2*I13*theta2_diff*wb2*cos(theta1)*cos(theta1) + 2*I22*theta2_diff*wb2*cos(theta2)*cos(theta2) - 2*I23*theta2_diff*wb2*cos(theta2)*cos(theta2) - 2*I31*theta3_diff*wb1*cos(theta3)*cos(theta3) + 2*I33*theta3_diff*wb1*cos(theta3)*cos(theta3) - 2*I41*theta3_diff*wb1*cos(theta3)*cos(theta3) + 2*I42*theta3_diff*wb1*cos(theta3)*cos(theta3) + 2*I42*theta3_diff*wb1*cos(theta4)*cos(theta4) - 2*I43*theta3_diff*wb1*cos(theta4)*cos(theta4) - 2*I51*theta3_diff*wb1*cos(theta4)*cos(theta4) + 2*I52*theta3_diff*wb1*cos(theta3)*cos(theta3) + 2*I52*theta3_diff*wb1*cos(theta4)*cos(theta4) - 2*I53*theta3_diff*wb1*cos(theta3)*cos(theta3) - 2*I11*wb1*wb2*cos(theta1)*cos(theta1) - I11*wb1*wb2*cos(theta2)*cos(theta2) + I12*wb1*wb2*cos(theta2)*cos(theta2) + 2*I13*wb1*wb2*cos(theta1)*cos(theta1) + I22*wb1*wb2*cos(theta2)*cos(theta2) - I23*wb1*wb2*cos(theta2)*cos(theta2) - I31*wb1*wb2*cos(theta3)*cos(theta3) + I33*wb1*wb2*cos(theta3)*cos(theta3) - I41*wb1*wb2*cos(theta3)*cos(theta3) + I42*wb1*wb2*cos(theta3)*cos(theta3) + 2*I42*wb1*wb2*cos(theta4)*cos(theta4) - 2*I43*wb1*wb2*cos(theta4)*cos(theta4) - 2*I51*wb1*wb2*cos(theta4)*cos(theta4) + I52*wb1*wb2*cos(theta3)*cos(theta3) - I51*wb1*wb2*cos(theta5)*cos(theta5) + 2*I52*wb1*wb2*cos(theta4)*cos(theta4) - I53*wb1*wb2*cos(theta3)*cos(theta3) + I53*wb1*wb2*cos(theta5)*cos(theta5) - I11*theta2_diff*wb3*sin(2*theta2) + I12*theta2_diff*wb3*sin(2*theta2) + I22*theta2_diff*wb3*sin(2*theta2) - I23*theta2_diff*wb3*sin(2*theta2) + I31*theta3_diff*wb3*sin(2*theta3) - I33*theta3_diff*wb3*sin(2*theta3) + I41*theta3_diff*wb3*sin(2*theta3) - I42*theta3_diff*wb3*sin(2*theta3) - I52*theta3_diff*wb3*sin(2*theta3) + I53*theta3_diff*wb3*sin(2*theta3) - I51*theta5_diff*wb3*sin(2*theta5) + I53*theta5_diff*wb3*sin(2*theta5) - (I11*wb1*wb3*sin(2*theta2))/2 + (I12*wb1*wb3*sin(2*theta2))/2 + (I22*wb1*wb3*sin(2*theta2))/2 - (I23*wb1*wb3*sin(2*theta2))/2 + (I31*wb2*wb3*sin(2*theta3))/2 - (I33*wb2*wb3*sin(2*theta3))/2 + (I41*wb2*wb3*sin(2*theta3))/2 - (I42*wb2*wb3*sin(2*theta3))/2 - (I52*wb2*wb3*sin(2*theta3))/2 + (I53*wb2*wb3*sin(2*theta3))/2 + 2*I11*theta2_diff*wb2*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) - 2*I13*theta2_diff*wb2*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) - 2*I42*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + 2*I43*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + 2*I51*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) - 2*I51*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) - 2*I52*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + 2*I51*theta3_diff*wb1*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + 2*I53*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) - 2*I53*theta3_diff*wb1*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I11*wb1*wb2*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) - I13*wb1*wb2*cos(theta1)*cos(theta1)*cos(theta2)*cos(theta2) - I42*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + I43*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + I51*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) - I51*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) - I52*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4) + 2*I51*wb1*wb2*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I53*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta5)*cos(theta5) - 2*I53*wb1*wb2*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I11*theta2_diff*theta2_diff*cos(theta1)*sin(theta1)*sin(theta2) - I13*theta2_diff*theta2_diff*cos(theta1)*sin(theta1)*sin(theta2) - I42*theta3_diff*theta3_diff*cos(theta4)*sin(theta3)*sin(theta4) + I43*theta3_diff*theta3_diff*cos(theta4)*sin(theta3)*sin(theta4) + I51*theta3_diff*theta3_diff*cos(theta4)*sin(theta3)*sin(theta4) - I52*theta3_diff*theta3_diff*cos(theta4)*sin(theta3)*sin(theta4) + I11*wb1*wb1*cos(theta1)*sin(theta1)*sin(theta2) - I11*wb2*wb2*cos(theta1)*sin(theta1)*sin(theta2) - I13*wb1*wb1*cos(theta1)*sin(theta1)*sin(theta2) + I13*wb2*wb2*cos(theta1)*sin(theta1)*sin(theta2) + I42*wb1*wb1*cos(theta4)*sin(theta3)*sin(theta4) - I42*wb2*wb2*cos(theta4)*sin(theta3)*sin(theta4) - I43*wb1*wb1*cos(theta4)*sin(theta3)*sin(theta4) + I43*wb2*wb2*cos(theta4)*sin(theta3)*sin(theta4) - I51*wb1*wb1*cos(theta4)*sin(theta3)*sin(theta4) + I51*wb2*wb2*cos(theta4)*sin(theta3)*sin(theta4) + I52*wb1*wb1*cos(theta4)*sin(theta3)*sin(theta4) - I52*wb2*wb2*cos(theta4)*sin(theta3)*sin(theta4) - L3*L3*m4*theta3_diff*theta4_diff*cos(theta3) - L3*L3*m5*theta3_diff*theta4_diff*cos(theta3) - L3*m4*theta4_diff*vb3*cos(theta3)*cos(theta3) - L3*m5*theta4_diff*vb3*cos(theta3)*cos(theta3) - L3*L3*m4*theta4_diff*wb2*cos(theta3) - L3*L3*m5*theta4_diff*wb2*cos(theta3) - (Lb*Lb*m4*theta4_diff*wb2*cos(theta3))/4 - (Lb*Lb*m5*theta4_diff*wb2*cos(theta3))/4 + I51*theta4_diff*theta5_diff*cos(theta3)*cos(theta4) + I52*theta4_diff*theta5_diff*cos(theta3)*cos(theta4) - I53*theta4_diff*theta5_diff*cos(theta3)*cos(theta4) - (L3*m4*theta4_diff*vb1*sin(2*theta3))/2 - (L3*m5*theta4_diff*vb1*sin(2*theta3))/2 - (L3*Lb*m4*theta3_diff*theta4_diff)/2 - (L3*Lb*m5*theta3_diff*theta4_diff)/2 - (L3*Lb*m4*theta4_diff*wb2)/2 - (L3*Lb*m5*theta4_diff*wb2)/2 + I51*theta3_diff*theta5_diff*sin(theta3)*sin(theta4) - I52*theta3_diff*theta5_diff*sin(theta3)*sin(theta4) - I53*theta3_diff*theta5_diff*sin(theta3)*sin(theta4) + I51*theta5_diff*wb2*sin(theta3)*sin(theta4) - I52*theta5_diff*wb2*sin(theta3)*sin(theta4) - I53*theta5_diff*wb2*sin(theta3)*sin(theta4) + (4*L3*L3*m3*theta3_diff*wb1*cos(theta3)*cos(theta3))/9 + L3*L3*m4*theta3_diff*wb1*cos(theta3)*cos(theta3) + L3*L3*m5*theta3_diff*wb1*cos(theta3)*cos(theta3) - 2*I11*theta1_diff*theta2_diff*cos(theta1)*cos(theta1)*cos(theta2) + 2*I13*theta1_diff*theta2_diff*cos(theta1)*cos(theta1)*cos(theta2) + 2*I42*theta3_diff*theta4_diff*cos(theta3)*cos(theta4)*cos(theta4) - 2*I43*theta3_diff*theta4_diff*cos(theta3)*cos(theta4)*cos(theta4) - 2*I51*theta3_diff*theta4_diff*cos(theta3)*cos(theta4)*cos(theta4) - 2*I51*theta3_diff*theta4_diff*cos(theta3)*cos(theta5)*cos(theta5) + 2*I52*theta3_diff*theta4_diff*cos(theta3)*cos(theta4)*cos(theta4) + 2*I53*theta3_diff*theta4_diff*cos(theta3)*cos(theta5)*cos(theta5) - 2*I11*theta1_diff*wb1*cos(theta1)*cos(theta1)*cos(theta2) + 2*I13*theta1_diff*wb1*cos(theta1)*cos(theta1)*cos(theta2) + 2*I42*theta4_diff*wb2*cos(theta3)*cos(theta4)*cos(theta4) - 2*I43*theta4_diff*wb2*cos(theta3)*cos(theta4)*cos(theta4) - 2*I51*theta4_diff*wb2*cos(theta3)*cos(theta4)*cos(theta4) + 2*I51*theta5_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4) - 2*I51*theta4_diff*wb2*cos(theta3)*cos(theta5)*cos(theta5) + 2*I52*theta4_diff*wb2*cos(theta3)*cos(theta4)*cos(theta4) + 2*I51*theta5_diff*wb1*cos(theta4)*cos(theta5)*cos(theta5) - 2*I53*theta5_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4) + 2*I53*theta4_diff*wb2*cos(theta3)*cos(theta5)*cos(theta5) - 2*I53*theta5_diff*wb1*cos(theta4)*cos(theta5)*cos(theta5) - (2*L3*L3*m3*theta3_diff*wb3*sin(2*theta3))/9 - (L3*L3*m4*theta3_diff*wb3*sin(2*theta3))/2 - (L3*L3*m5*theta3_diff*wb3*sin(2*theta3))/2 - (Lb*m4*theta4_diff*vb3*cos(theta3))/2 - (Lb*m5*theta4_diff*vb3*cos(theta3))/2 - (Lb*m1*theta1_diff*vb1*sin(theta2))/2 - (Lb*m4*theta4_diff*vb1*sin(theta3))/2 - (Lb*m5*theta4_diff*vb1*sin(theta3))/2 + L3*m5*theta5_diff*vb1*cos(theta3)*cos(theta3)*sin(theta4) + (L3*Lb*m3*theta3_diff*wb1*cos(theta3))/3 + (L3*Lb*m4*theta3_diff*wb1*cos(theta3))/2 + (L3*Lb*m5*theta3_diff*wb1*cos(theta3))/2 - 2*I51*theta5_diff*wb3*cos(theta3)*cos(theta4)*sin(theta3) - 2*I51*theta3_diff*wb3*cos(theta4)*cos(theta5)*sin(theta5) + 2*I53*theta5_diff*wb3*cos(theta3)*cos(theta4)*sin(theta3) + 2*I53*theta3_diff*wb3*cos(theta4)*cos(theta5)*sin(theta5) + I11*wb2*wb3*cos(theta1)*cos(theta2)*sin(theta1) - I13*wb2*wb3*cos(theta1)*cos(theta2)*sin(theta1) + I42*wb1*wb3*cos(theta3)*cos(theta4)*sin(theta4) - I43*wb1*wb3*cos(theta3)*cos(theta4)*sin(theta4) - I51*wb1*wb3*cos(theta3)*cos(theta4)*sin(theta4) + I52*wb1*wb3*cos(theta3)*cos(theta4)*sin(theta4) - I51*wb2*wb3*cos(theta4)*cos(theta5)*sin(theta5) + I53*wb2*wb3*cos(theta4)*cos(theta5)*sin(theta5) - (Lb*Lb*m5*theta5_diff*wb2*sin(theta3)*sin(theta4))/4 - (L3*Lb*m3*theta3_diff*wb3*sin(theta3))/3 - (L3*Lb*m4*theta3_diff*wb3*sin(theta3))/2 - (L3*Lb*m5*theta3_diff*wb3*sin(theta3))/2 + 2*I51*theta4_diff*theta5_diff*cos(theta5)*sin(theta3)*sin(theta5) - 2*I53*theta4_diff*theta5_diff*cos(theta5)*sin(theta3)*sin(theta5) - 2*I51*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + 2*I53*theta3_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + 2*I11*theta2_diff*wb1*cos(theta1)*sin(theta1)*sin(theta2) - 2*I13*theta2_diff*wb1*cos(theta1)*sin(theta1)*sin(theta2) - 2*I42*theta3_diff*wb2*cos(theta4)*sin(theta3)*sin(theta4) + 2*I43*theta3_diff*wb2*cos(theta4)*sin(theta3)*sin(theta4) + 2*I51*theta3_diff*wb2*cos(theta4)*sin(theta3)*sin(theta4) - 2*I52*theta3_diff*wb2*cos(theta4)*sin(theta3)*sin(theta4) - I51*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + I53*wb1*wb2*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) - I51*theta3_diff*theta3_diff*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + I53*theta3_diff*theta3_diff*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + I51*wb1*wb1*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - I51*wb2*wb2*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - I53*wb1*wb1*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + I53*wb2*wb2*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - 2*I51*theta4_diff*theta5_diff*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5) + 2*I53*theta4_diff*theta5_diff*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5) - (L3*Lb*m4*theta4_diff*wb2*cos(theta3)*cos(theta3))/2 - (L3*Lb*m5*theta4_diff*wb2*cos(theta3)*cos(theta3))/2 + 2*I11*theta1_diff*wb3*cos(theta1)*cos(theta2)*cos(theta2)*sin(theta1) + 2*I11*theta2_diff*wb3*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) - 2*I13*theta1_diff*wb3*cos(theta1)*cos(theta2)*cos(theta2)*sin(theta1) - 2*I13*theta2_diff*wb3*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) + 2*I42*theta3_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - 2*I43*theta3_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + 2*I42*theta4_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*sin(theta4) - 2*I43*theta4_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*sin(theta4) - 2*I51*theta3_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + 2*I51*theta3_diff*wb3*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I52*theta3_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - 2*I51*theta4_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*sin(theta4) + 2*I52*theta4_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*sin(theta4) - 2*I53*theta3_diff*wb3*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I51*theta5_diff*wb3*cos(theta3)*cos(theta3)*cos(theta5)*sin(theta5) - 2*I53*theta5_diff*wb3*cos(theta3)*cos(theta3)*cos(theta5)*sin(theta5) + I11*wb1*wb3*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) - I13*wb1*wb3*cos(theta1)*cos(theta1)*cos(theta2)*sin(theta2) + I42*wb2*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I43*wb2*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I51*wb2*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) + I51*wb2*wb3*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) + I52*wb2*wb3*cos(theta3)*cos(theta4)*cos(theta4)*sin(theta3) - I53*wb2*wb3*cos(theta3)*cos(theta5)*cos(theta5)*sin(theta3) - 2*I51*theta3_diff*theta5_diff*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + 2*I53*theta3_diff*theta5_diff*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - 2*I51*theta5_diff*wb2*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + 2*I53*theta5_diff*wb2*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + (Lb*m5*theta5_diff*vb1*cos(theta3)*sin(theta4))/2 - (Lb*m5*theta5_diff*vb3*sin(theta3)*sin(theta4))/2 + 2*I51*theta3_diff*theta4_diff*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) - 2*I53*theta3_diff*theta4_diff*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + 2*I51*theta4_diff*wb2*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) - 4*I51*theta5_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5) - 2*I53*theta4_diff*wb2*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5) + 4*I53*theta5_diff*wb1*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5) - I51*theta3_diff*theta3_diff*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) + I51*theta4_diff*theta4_diff*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) + I53*theta3_diff*theta3_diff*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - I53*theta4_diff*theta4_diff*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) + I51*wb1*wb1*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - I51*wb2*wb2*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - I53*wb1*wb1*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) + I53*wb2*wb2*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - 2*I11*theta1_diff*wb2*cos(theta1)*cos(theta2)*sin(theta1)*sin(theta2) + 2*I13*theta1_diff*wb2*cos(theta1)*cos(theta2)*sin(theta1)*sin(theta2) + 2*I42*theta4_diff*wb1*cos(theta3)*cos(theta4)*sin(theta3)*sin(theta4) - 2*I43*theta4_diff*wb1*cos(theta3)*cos(theta4)*sin(theta3)*sin(theta4) - 2*I51*theta4_diff*wb1*cos(theta3)*cos(theta4)*sin(theta3)*sin(theta4) + 2*I52*theta4_diff*wb1*cos(theta3)*cos(theta4)*sin(theta3)*sin(theta4) - 2*I51*theta3_diff*wb2*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) + 2*I51*theta5_diff*wb1*cos(theta3)*cos(theta5)*sin(theta3)*sin(theta5) + 2*I53*theta3_diff*wb2*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - 2*I53*theta5_diff*wb1*cos(theta3)*cos(theta5)*sin(theta3)*sin(theta5) - I51*wb1*wb3*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + I53*wb1*wb3*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + 4*I51*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) + 4*I51*theta5_diff*wb3*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - 4*I53*theta3_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) - 4*I53*theta5_diff*wb3*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + I51*wb1*wb3*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) + 2*I51*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) - I53*wb1*wb3*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I53*wb2*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta5) - 2*I51*theta3_diff*wb2*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + 2*I51*theta4_diff*wb1*cos(theta3)*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) + 2*I53*theta3_diff*wb2*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - 2*I53*theta4_diff*wb1*cos(theta3)*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta5) - L3*m5*theta5_diff*vb3*cos(theta3)*sin(theta3)*sin(theta4) + 2*I51*theta3_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I51*theta4_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I53*theta3_diff*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I51*theta5_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) - 2*I53*theta4_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta4) - 2*I53*theta5_diff*wb3*cos(theta3)*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta5) + I51*wb2*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) - I53*wb2*wb3*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3) + 2*I51*theta4_diff*wb1*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) + 2*I51*theta5_diff*wb1*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 2*I53*theta4_diff*wb1*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta5)*sin(theta3)*sin(theta4) - 2*I53*theta5_diff*wb1*cos(theta3)*cos(theta4)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 2*I51*theta3_diff*theta5_diff*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta4)*sin(theta5) + 2*I53*theta3_diff*theta5_diff*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta4)*sin(theta5) + 4*I51*theta3_diff*wb1*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 4*I53*theta3_diff*wb1*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 2*I51*theta5_diff*wb2*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta4)*sin(theta5) + 2*I53*theta5_diff*wb2*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta4)*sin(theta5) + 2*I51*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - 2*I53*wb1*wb2*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta5) - (L3*Lb*m5*theta5_diff*wb2*cos(theta3)*sin(theta3)*sin(theta4))/2 - 2*I51*theta4_diff*wb3*cos(theta3)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5) + 2*I53*theta4_diff*wb3*cos(theta3)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta5),
            I12*(theta2_diff*wb3*cos(theta2) - theta2_diff*wb2*sin(theta2)) - I11*(cos(theta1)*(wb3*cos(theta2) - wb2*sin(theta2)) + sin(theta1)*(theta2_diff + wb1))*(sin(theta1)*(wb3*cos(theta2) - wb2*sin(theta2)) - cos(theta1)*(theta2_diff + wb1)) + I13*(cos(theta1)*(wb3*cos(theta2) - wb2*sin(theta2)) + sin(theta1)*(theta2_diff + wb1))*(sin(theta1)*(wb3*cos(theta2) - wb2*sin(theta2)) - cos(theta1)*(theta2_diff + wb1)),
            I23*(wb2*cos(theta2) + wb3*sin(theta2))*(wb3*cos(theta2) - wb2*sin(theta2)) - sin(theta1)*(I13*(cos(theta1)*(theta2_diff*wb2*cos(theta2) + theta2_diff*wb3*sin(theta2)) + theta1_diff*sin(theta1)*(wb3*cos(theta2) - wb2*sin(theta2)) - theta1_diff*cos(theta1)*(theta2_diff + wb1)) - I11*(sin(theta1)*(wb3*cos(theta2) - wb2*sin(theta2)) - cos(theta1)*(theta2_diff + wb1))*(theta1_diff + wb2*cos(theta2) + wb3*sin(theta2)) + I12*(sin(theta1)*(wb3*cos(theta2) - wb2*sin(theta2)) - cos(theta1)*(theta2_diff + wb1))*(theta1_diff + wb2*cos(theta2) + wb3*sin(theta2))) - I22*(wb2*cos(theta2) + wb3*sin(theta2))*(wb3*cos(theta2) - wb2*sin(theta2)) - cos(theta1)*(I11*(theta1_diff*sin(theta1)*(theta2_diff + wb1) - sin(theta1)*(theta2_diff*wb2*cos(theta2) + theta2_diff*wb3*sin(theta2)) + theta1_diff*cos(theta1)*(wb3*cos(theta2) - wb2*sin(theta2))) + I12*(cos(theta1)*(wb3*cos(theta2) - wb2*sin(theta2)) + sin(theta1)*(theta2_diff + wb1))*(theta1_diff + wb2*cos(theta2) + wb3*sin(theta2)) - I13*(cos(theta1)*(wb3*cos(theta2) - wb2*sin(theta2)) + sin(theta1)*(theta2_diff + wb1))*(theta1_diff + wb2*cos(theta2) + wb3*sin(theta2))),
            cos(theta4)*(I42*(theta4_diff*cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - theta4_diff*sin(theta4)*(theta3_diff + wb2) + theta3_diff*sin(theta4)*(wb1*cos(theta3) - wb3*sin(theta3))) + I52*(theta4_diff*cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - theta4_diff*sin(theta4)*(theta3_diff + wb2) + theta3_diff*sin(theta4)*(wb1*cos(theta3) - wb3*sin(theta3))) - I51*(sin(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) + cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(cos(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) - sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))) + I53*(sin(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) + cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(cos(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) - sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))) - I41*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) + I43*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3))) + sin(theta4)*(cos(theta5)*(I53*(cos(theta5)*(theta4_diff*sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - theta3_diff*cos(theta4)*(wb1*cos(theta3) - wb3*sin(theta3)) + theta4_diff*cos(theta4)*(theta3_diff + wb2)) + theta3_diff*sin(theta5)*(wb3*cos(theta3) + wb1*sin(theta3)) - theta5_diff*sin(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) - theta5_diff*cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))) + I51*(sin(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) + cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(theta5_diff + theta3_diff*cos(theta4) + wb2*cos(theta4) + wb3*cos(theta3)*sin(theta4) + wb1*sin(theta3)*sin(theta4)) - I52*(sin(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) + cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(theta5_diff + theta3_diff*cos(theta4) + wb2*cos(theta4) + wb3*cos(theta3)*sin(theta4) + wb1*sin(theta3)*sin(theta4))) + sin(theta5)*(I51*(sin(theta5)*(theta4_diff*sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - theta3_diff*cos(theta4)*(wb1*cos(theta3) - wb3*sin(theta3)) + theta4_diff*cos(theta4)*(theta3_diff + wb2)) - theta3_diff*cos(theta5)*(wb3*cos(theta3) + wb1*sin(theta3)) + theta5_diff*cos(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) - theta5_diff*sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))) + I52*(cos(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) - sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(theta5_diff + theta3_diff*cos(theta4) + wb2*cos(theta4) + wb3*cos(theta3)*sin(theta4) + wb1*sin(theta3)*sin(theta4)) - I53*(cos(theta5)*(theta3_diff*sin(theta4) + wb2*sin(theta4) - wb3*cos(theta3)*cos(theta4) - wb1*cos(theta4)*sin(theta3)) - sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(theta5_diff + theta3_diff*cos(theta4) + wb2*cos(theta4) + wb3*cos(theta3)*sin(theta4) + wb1*sin(theta3)*sin(theta4))) + I43*(theta4_diff*sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - theta3_diff*cos(theta4)*(wb1*cos(theta3) - wb3*sin(theta3)) + theta4_diff*cos(theta4)*(theta3_diff + wb2)) + I41*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))*(theta3_diff*cos(theta4) + wb2*cos(theta4) + wb3*cos(theta3)*sin(theta4) + wb1*sin(theta3)*sin(theta4)) - I42*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))*(theta3_diff*cos(theta4) + wb2*cos(theta4) + wb3*cos(theta3)*sin(theta4) + wb1*sin(theta3)*sin(theta4))) + I31*(wb3*cos(theta3) + wb1*sin(theta3))*(wb1*cos(theta3) - wb3*sin(theta3)) - I33*(wb3*cos(theta3) + wb1*sin(theta3))*(wb1*cos(theta3) - wb3*sin(theta3)) + (L3*m3*(981*R_IB3_3*cos(theta3) + 981*R_IB3_1*sin(theta3) + 100*theta3_diff*vb1*cos(theta3) - 100*theta3_diff*vb3*sin(theta3) - 50*Lb*theta3_diff*wb2*sin(theta3)))/150 - L3*cos(theta4)*((981*R_IB3_2*m4*sin(theta4))/100 + (981*R_IB3_2*m5*sin(theta4))/100 + m4*theta4_diff*vb2*cos(theta4) - m5*theta5_diff*vb1*cos(theta3) + m5*theta4_diff*vb2*cos(theta4) + m5*theta5_diff*vb3*sin(theta3) - (981*R_IB3_3*m4*cos(theta3)*cos(theta4))/100 - (981*R_IB3_3*m5*cos(theta3)*cos(theta4))/100 - (981*R_IB3_1*m4*cos(theta4)*sin(theta3))/100 - (981*R_IB3_1*m5*cos(theta4)*sin(theta3))/100 - m4*theta3_diff*vb1*cos(theta3)*cos(theta4) - m5*theta3_diff*vb1*cos(theta3)*cos(theta4) + m4*theta3_diff*vb3*cos(theta4)*sin(theta3) + m4*theta4_diff*vb3*cos(theta3)*sin(theta4) + m5*theta3_diff*vb3*cos(theta4)*sin(theta3) + m5*theta4_diff*vb3*cos(theta3)*sin(theta4) + m4*theta4_diff*vb1*sin(theta3)*sin(theta4) + m5*theta4_diff*vb1*sin(theta3)*sin(theta4) - (Lb*m4*theta4_diff*wb3*cos(theta4))/2 - (Lb*m5*theta4_diff*wb3*cos(theta4))/2 + L3*m4*theta3_diff*theta4_diff*sin(theta4) + L3*m5*theta3_diff*theta4_diff*sin(theta4) + L3*m4*theta4_diff*wb2*sin(theta4) + L3*m5*theta4_diff*wb2*sin(theta4) + (Lb*m5*theta5_diff*wb2*sin(theta3))/2 - L3*m4*theta4_diff*wb3*cos(theta3)*cos(theta4) - L3*m5*theta4_diff*wb3*cos(theta3)*cos(theta4) - L3*m4*theta3_diff*wb1*cos(theta3)*sin(theta4) - L3*m4*theta4_diff*wb1*cos(theta4)*sin(theta3) - L3*m5*theta3_diff*wb1*cos(theta3)*sin(theta4) - L3*m5*theta4_diff*wb1*cos(theta4)*sin(theta3) + (Lb*m4*theta3_diff*wb2*cos(theta4)*sin(theta3))/2 + (Lb*m4*theta4_diff*wb2*cos(theta3)*sin(theta4))/2 + (Lb*m5*theta3_diff*wb2*cos(theta4)*sin(theta3))/2 + (Lb*m5*theta4_diff*wb2*cos(theta3)*sin(theta4))/2 + L3*m4*theta3_diff*wb3*sin(theta3)*sin(theta4) + L3*m5*theta3_diff*wb3*sin(theta3)*sin(theta4)) - (327*L3*(R_IB3_3*cos(theta3) + R_IB3_1*sin(theta3))*(2*m3 + 3*m4 + 3*m5))/100 + L3*sin(theta4)*(m4/100 + m5/100)*(981*R_IB3_2*cos(theta4) - 100*theta4_diff*vb2*sin(theta4) + 981*R_IB3_3*cos(theta3)*sin(theta4) + 981*R_IB3_1*sin(theta3)*sin(theta4) + 100*theta3_diff*vb1*cos(theta3)*sin(theta4) + 100*theta4_diff*vb1*cos(theta4)*sin(theta3) - 100*theta3_diff*vb3*sin(theta3)*sin(theta4) + 100*L3*theta3_diff*theta4_diff*cos(theta4) + 100*L3*theta4_diff*wb2*cos(theta4) + 50*Lb*theta4_diff*wb3*sin(theta4) + 100*theta4_diff*vb3*cos(theta3)*cos(theta4) - 100*L3*theta3_diff*wb1*cos(theta3)*cos(theta4) + 50*Lb*theta4_diff*wb2*cos(theta3)*cos(theta4) + 100*L3*theta3_diff*wb3*cos(theta4)*sin(theta3) + 100*L3*theta4_diff*wb3*cos(theta3)*sin(theta4) + 100*L3*theta4_diff*wb1*sin(theta3)*sin(theta4) - 50*Lb*theta3_diff*wb2*sin(theta3)*sin(theta4)),
            I43*(sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) + cos(theta4)*(theta3_diff + wb2))*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) - cos(theta5)*(I51*(cos(theta5)*(theta3_diff*wb3*cos(theta3) + theta3_diff*wb1*sin(theta3)) - sin(theta5)*(theta4_diff*sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - cos(theta4)*(theta3_diff*wb1*cos(theta3) - theta3_diff*wb3*sin(theta3)) + theta4_diff*cos(theta4)*(theta3_diff + wb2)) + theta5_diff*cos(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) + theta5_diff*sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))) + I52*(cos(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) + sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(theta5_diff + sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) + cos(theta4)*(theta3_diff + wb2)) - I53*(cos(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) + sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(theta5_diff + sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) + cos(theta4)*(theta3_diff + wb2))) - sin(theta5)*(I53*(cos(theta5)*(theta4_diff*sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - cos(theta4)*(theta3_diff*wb1*cos(theta3) - theta3_diff*wb3*sin(theta3)) + theta4_diff*cos(theta4)*(theta3_diff + wb2)) + sin(theta5)*(theta3_diff*wb3*cos(theta3) + theta3_diff*wb1*sin(theta3)) + theta5_diff*sin(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) - theta5_diff*cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))) - I51*(sin(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) - cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(theta5_diff + sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) + cos(theta4)*(theta3_diff + wb2)) + I52*(sin(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) - cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(theta5_diff + sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) + cos(theta4)*(theta3_diff + wb2))) - I42*(sin(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) + cos(theta4)*(theta3_diff + wb2))*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) - I41*(theta3_diff*wb3*cos(theta3) + theta3_diff*wb1*sin(theta3)),
            I52*(sin(theta4)*(theta3_diff*wb1*cos(theta3) - theta3_diff*wb3*sin(theta3)) - theta4_diff*sin(theta4)*(theta3_diff + wb2) + theta4_diff*cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3))) - I51*(cos(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) + sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(sin(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) - cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3))) + I53*(cos(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) + sin(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)))*(sin(theta5)*(cos(theta4)*(wb3*cos(theta3) + wb1*sin(theta3)) - sin(theta4)*(theta3_diff + wb2)) - cos(theta5)*(theta4_diff + wb1*cos(theta3) - wb3*sin(theta3)));

        G<< (981*R_IB3_1*(m1 + m2 + m3 + m4 + m5 + mb))/100,
            (981*R_IB3_2*(m1 + m2 + m3 + m4 + m5 + mb))/100,
            (981*R_IB3_3*(m1 + m2 + m3 + m4 + m5 + mb))/100,
            -(327*L3*R_IB3_2*sin(theta3)*(2*m3 + 3*m4 + 3*m5))/100,
            (981*Lb*R_IB3_3*m3)/200 - (981*Lb*R_IB3_3*m2)/200 - (981*Lb*R_IB3_3*m1)/200 + (981*Lb*R_IB3_3*m4)/200 + (981*Lb*R_IB3_3*m5)/200 + (327*L3*R_IB3_3*m3*cos(theta3))/50 + (981*L3*R_IB3_3*m4*cos(theta3))/100 + (981*L3*R_IB3_3*m5*cos(theta3))/100 + (327*L3*R_IB3_1*m3*sin(theta3))/50 + (981*L3*R_IB3_1*m4*sin(theta3))/100 + (981*L3*R_IB3_1*m5*sin(theta3))/100,
            -(327*R_IB3_2*(3*Lb*m3 - 3*Lb*m2 - 3*Lb*m1 + 3*Lb*m4 + 3*Lb*m5 + 4*L3*m3*cos(theta3) + 6*L3*m4*cos(theta3) + 6*L3*m5*cos(theta3)))/200,
            0,
            0,
            (327*L3*(R_IB3_3*cos(theta3) + R_IB3_1*sin(theta3))*(2*m3 + 3*m4 + 3*m5))/100,
            0,
            0;        
        //A是（6×8） 8×1驱动力向量 到 b系下力/力矩向量 的映射矩阵
        Eigen::MatrixXd A(6,6);
        A<<    R_IB3_1,              1,                       0,                     0,                                                             0,                                                                                                                                                 0,
                R_IB3_2,              0,                       1,                     0,                                                             0,                                                                                                                                                 0,
                R_IB3_3,              0,                       0,                     1,                                                             0,                                                                                                                                                 0,
                0,                                                     0,            -L3*sin(arm_data.pos[2]),                                           0,                                                                   sin(arm_data.pos[0]) - Ct_Cm/Lr*cos(arm_data.pos[0]), cos(arm_data.pos[2])*sin(arm_data.pos[4]) + Ct_Cm/Lr*cos(arm_data.pos[2])*cos(arm_data.pos[4]) + cos(arm_data.pos[3])*cos(arm_data.pos[4])*sin(arm_data.pos[2]) - Ct_Cm/Lr*cos(arm_data.pos[3])*sin(arm_data.pos[2])*sin(arm_data.pos[4]),
                -(Lb*R_IB3_3)*0.5, L3*sin(arm_data.pos[2]),            0,           Lb*0.5 + L3*cos(arm_data.pos[2]),   - cos(arm_data.pos[0])*sin(arm_data.pos[1]) - Ct_Cm/Lr*sin(arm_data.pos[0])*sin(arm_data.pos[1]),                                                                                       Ct_Cm/Lr*sin(arm_data.pos[3])*sin(arm_data.pos[4]) - cos(arm_data.pos[4])*sin(arm_data.pos[3]),
                (Lb*R_IB3_2)*0.5,                                      0,         - Lb*0.5 - L3*cos(arm_data.pos[2]),                                   0,                                                                     cos(arm_data.pos[0])*cos(arm_data.pos[1]) + Ct_Cm/Lr*cos(arm_data.pos[1])*sin(arm_data.pos[0]), cos(arm_data.pos[2])*cos(arm_data.pos[3])*cos(arm_data.pos[4]) - Ct_Cm/Lr*cos(arm_data.pos[4])*sin(arm_data.pos[2]) - sin(arm_data.pos[2])*sin(arm_data.pos[4]) - Ct_Cm/Lr*cos(arm_data.pos[2])*cos(arm_data.pos[3])*sin(arm_data.pos[4]);        
        
        //calculate P_IP,P_IE,zyx
        Eigen::Vector3d P_BE=-Lb/2*tmp_vector(1,0,0)-compute_R(0,theta3,0)*L3*tmp_vector(1,0,0);
        Eigen::Vector3d P_BP=Lb/2*tmp_vector(1,0,0)+compute_R(theta2,0,0)*compute_R(0,theta1,0)*Lp*tmp_vector(1,0,0);        
        Eigen::Vector3d P_IE=P_IB+R_IB*P_BE;
        Eigen::Vector3d P_IP=P_IB+R_IB*P_BP;
        Eigen::Matrix3d R_BP=compute_R(theta2,0,0)*compute_R(0,theta1,0);//*compute_R(0,-pi/2,0);
        Eigen::Matrix3d R_IP=R_IB*R_BP;
        Eigen::Vector3d zyx;
        zyx<<atan2(R_IP(2,1),R_IP(2,2)),
            atan2(-R_IP(2,0),sqrt(R_IP(2,1)*R_IP(2,1)+R_IP(2,2)*R_IP(2,2))),
            atan2(R_IP(1,0),R_IP(0,0));
        Eigen::Matrix3d Q=compute_Q(zyx); 

            
        //J is transform matrix from joint to workspace
        Eigen::MatrixXd J=Eigen::MatrixXd::Zero(11,11);              
        J.block(0,0,3,3)=R_IB;
        J.block(6,0,3,3)=R_IB;

        J.block(3,3,3,3)=Q.inverse()*R_BP.transpose();
        J.block(0,3,3,3)=-skew(R_IB*P_BP)*R_IB;
        J.block(6,3,3,3)=-skew(R_IB*P_BE)*R_IB;

        J.block(0,6,3,1)=R_IB*skew(compute_R(theta2,0,0)*tmp_vector(0,1,0))*compute_R(theta2,0,0)*compute_R(0,theta1,0)*Lp*tmp_vector(1,0,0);
        J.block(0,7,3,1)=R_IB*skew(tmp_vector(1,0,0))*compute_R(theta2,0,0)*compute_R(0,theta1,0)*Lp*tmp_vector(1,0,0);
        J.block(3,6,3,1)=Q.inverse()*R_BP.transpose()*compute_R(theta2,0,0)*tmp_vector(0,1,0);
        J.block(3,7,3,1)=Q.inverse()*R_BP.transpose()*tmp_vector(1,0,0);
        J.block(6,8,3,1)=skew(tmp_vector(0,1,0))*compute_R(0,theta3,0)*(-L3)*tmp_vector(1,0,0);

        J(9,9)=1;
        J(10,10)=1;

        //广义关节期望值设置
        if (step_control<pretime/control_period)
        {
            if(!set_state[0]){
                cout<<"准备状态"<<endl;
                set_state[0]=1;
            }

            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-0.5,0);
            //P_IP_d<<0,0,1;
            //zyx_d<<0,0,0;  
        }

        else if (step_control<(pretime+5)/control_period)
        {           
            if(!set_state[1]){
                cout<<"初始悬停"<<endl;
                set_state[1]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-0.5,0); 
            //P_IP_d<<0,0,1;
            //zyx_d<<0,0,0;         
        }
        /*
        else if (step_control<(pretime+10)/control_period)
        {
            if(!set_state[2]){
                cout<<"旋转1"<<endl;
                set_state[2]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,0,0);
        }   
        else if (step_control<(pretime+15)/control_period)
        {
            if(!set_state[3]){
                cout<<"初始悬停"<<endl;
                set_state[3]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-0.5,0);
        }   
        else if (step_control<(pretime+20)/control_period)
        {
            if(!set_state[4]){
                cout<<"旋转2"<<endl;
                set_state[4]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-1.2,0);
        }   
        else if (step_control<(pretime+25)/control_period)
        {
            if(!set_state[5]){
                cout<<"初始悬停"<<endl;
                set_state[5]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-0.5,0);
        }      
        else if (step_control<(pretime+30)/control_period)
        {
            if(!set_state[6]){
                cout<<"旋转3"<<endl;
                set_state[6]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-0.5,0)*compute_R(0,0,1);
        }  
        else if (step_control<(pretime+35)/control_period)
        {
            if(!set_state[7]){
                cout<<"初始悬停"<<endl;
                set_state[7]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-0.5,0);
        }          
        else if (step_control<(pretime+40)/control_period)
        {
            if(!set_state[8]){
                cout<<"平移1"<<endl;
                set_state[8]=1;
            }
            
            theta3_goal=1;
            P_IB_d << 0.5,0,0.5;
            R_IB_d= compute_R(0,-0.5,0);
        }          
        else if (step_control<(pretime+45)/control_period)
        {
            if(!set_state[9]){
                cout<<"初始悬停"<<endl;
                set_state[9]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-0.5,0);
        }     
        else if (step_control<(pretime+50)/control_period)
        {
            if(!set_state[10]){
                 cout<<"平移2"<<endl;
                set_state[10]=1;
            }                      
            theta3_goal=1;
            P_IB_d << 0,0.2,0.5;
            R_IB_d= compute_R(0,-0.5,0);
        }         
        else if (step_control<(pretime+55)/control_period)
        {
               if(!set_state[11]){
                cout<<"初始悬停"<<endl;
                set_state[11]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-0.5,0);
        }     
        else if (step_control<(pretime+60)/control_period)
        {
            if(!set_state[12]){
                cout<<"平移3"<<endl;
                set_state[12]=1;
            }           
            theta3_goal=1;
            P_IB_d << 0,0,0.8;
            R_IB_d= compute_R(0,-0.5,0);
        }  
        else if (step_control<(pretime+65)/control_period)
        {
            if(!set_state[13]){
                cout<<"初始悬停"<<endl;
                set_state[13]=1;
            }
            theta3_goal=1;
            P_IB_d << 0,0,0.5;
            R_IB_d= compute_R(0,-0.5,0);
        } 
        */
        else if (step_control<(pretime+10)/control_period)
        {
            if(!set_state[14]){
                cout<<"栖息姿态"<<endl;
                set_state[14]=1;
            }            
            theta3_goal=1;
            R_IB_d= compute_R(0,-1.2,0);
            P_IB_d=tmp_vector(0,0,1.05)-R_IB_d*tmp_vector(Lb/2,0,0);
            //P_IB_d<<-0.4,0,0.4;
            if (abs(P_IB(0)-P_IP_d(0))<0.05 && abs(P_IB(1)-P_IP_d(1))<0.05 && abs(P_IB(2)-P_IP_d(2))<0.05 && !set_state[15]){
                cout<<"perch success"<<endl;
                cout<<step_control*control_period<<endl;
            }
        }  
        else if (step_control<(pretime+15)/control_period)
        {
            if(!set_state[15]){
                cout<<"状态1"<<endl;
                set_state[15]=1;
            }           
            theta3_goal=1+0.4 * (step_control-(pretime+10)/control_period)   /  (5/control_period) ; 
            P_IP_d << 0,0,1.05;         
            R_IB_d= compute_R(0,-1.2,0);
            P_IB_d=P_IP_d-R_IB_d*tmp_vector(Lb/2,0,0);
        }  
        else if (step_control<(pretime+25)/control_period)
        {
            if(!set_state[16]){
                cout<<"状态2"<<endl;
                set_state[16]=1;
            }
            theta3_goal=1.4;
            P_IP_d << 0,0,1.05;            
            R_IB_d= compute_R(0,-1.2,0)*compute_R(0,0,0.5);
            P_IB_d=P_IP_d-R_IB_d*tmp_vector(Lb/2,0,0);
        }
        else if (set_state[15]==1 && step_control<(pretime+35)/control_period)
        {
            theta3_goal=1.4-0.8 * (step_control-(pretime+25)/control_period)   /  (10/control_period);
            P_IP_d << 0,0,1.05;            
            R_IB_d= compute_R(0,-0.8,0);
            P_IB_d=P_IP_d-R_IB_d*tmp_vector(Lb/2,0,0);
            if(!set_state[18]){
                cout<<"状态3"<<endl;
                set_state[18]=1;
                //cout<<P_IE_d<<endl;
            }
        }


       /*
        else if (step_control<(pretime+10)/control_period)
        {
            theta3_goal=1;
            R_IB_d= compute_R(0,-0.5,0);
            P_IB_d=tmp_vector(0,0,1.05)-R_IB_d*tmp_vector(Lb/2,0,0);
            if(!set_state[14]){
                cout<<"栖息姿态"<<endl;
                set_state[14]=1;
                //ifperchsuccess=1;
            }
            P_IP_d<<0,0,1;
            zyx_d<<0,0,0; 
            P_IE_d<<0,0,0; 
            //P_IB_d<<-0.4,0,0.4;
            if (abs(P_IPM(0)-P_IP_d(0))<0.1 && abs(P_IPM(1)-P_IP_d(1))<0.1 && abs(P_IPM(2)-P_IP_d(2))<0.1 &&
                abs(zyxm(0)-zyx_d(0))<0.05 && abs(zyxm(0)-zyx_d(0))<0.05 && abs(zyxm(0)-zyx_d(0))<0.05 && !set_state[15]){
                cout<<"perch success"<<endl;
                cout<<step_control*control_period<<endl;
                
            }
            set_state[15]=1;
        }

        else if (set_state[15]==1 && step_control<(pretime+15)/control_period)
        {            
            theta3_goal=1+0.4 * (step_control-(pretime+10)/control_period)   /  (5/control_period) ; 
            P_IP_d << 0,0,1.05;         
            R_IB_d= compute_R(0,-1.2,0);
            P_IB_d=P_IP_d-R_IB_d*tmp_vector(Lb/2,0,0);
            Eigen::Vector3d P_BE_d=-tmp_vector(Lb/2,0,0)-compute_R(0,1.4,0)*tmp_vector(L3,0,0);
            P_IE_d=P_IB_d+R_IB_d*P_BE_d;
            P_IP_d << 0,0,1;
            zyx_d<<0,0,0;
            P_IB_d<<-0.4,0,0.83;
            if(!set_state[16]){
                cout<<"状态1"<<endl;
                set_state[16]=1;
                //cout<<P_IE_d<<endl;   
            }
        }  
        else if (set_state[15]==1 && step_control<(pretime+25)/control_period)
        {
            //theta3_goal=1.57-1.07 * (step_control-(pretime+15)/control_period)   /  (5/control_period);
            theta3_goal=1.4;
            P_IP_d << 0,0,1.05;            
            R_IB_d= compute_R(0,-1.2,0)*compute_R(0,0,0.5);
            P_IB_d=P_IP_d-R_IB_d*tmp_vector(Lb/2,0,0);
            Eigen::Vector3d P_BE_d=-tmp_vector(Lb/2,0,0)-compute_R(0,theta3_goal,0)*tmp_vector(L3,0,0);
            P_IE_d=P_IB_d+R_IB_d*P_BE_d;
            P_IP_d << 0,0,1;
            zyx_d<<0,0,0;
            P_IB_d<<-0.38,-0.16,0.87;
            if(!set_state[17]){
                cout<<"状态2"<<endl;
                set_state[17]=1;
                //cout<<P_IE_d<<endl;
            }
        } 
        else if (set_state[15]==1 && step_control<(pretime+35)/control_period)
        {
            theta3_goal=1.4-0.8 * (step_control-(pretime+25)/control_period)   /  (10/control_period);
            P_IP_d << 0,0,1.05;            
            R_IB_d= compute_R(0,-0.8,0);
            P_IB_d=P_IP_d-R_IB_d*tmp_vector(Lb/2,0,0);
            Eigen::Vector3d P_BE_d=-tmp_vector(Lb/2,0,0)-compute_R(0,1,0)*tmp_vector(L3,0,0);
            P_IE_d=P_IB_d+R_IB_d*P_BE_d;
            P_IP_d << 0,0,1;
            zyx_d<<0,0,0;
            P_IB_d<<-0.5,0,0.9;
            if(!set_state[18]){
                cout<<"状态3"<<endl;
                set_state[18]=1;
                //cout<<P_IE_d<<endl;
            }
        }
        */


        if(ifperchsuccess && !set_state[10])
            set_state[10]=1;
        if(ifperchsuccess && set_state[10])  
            tao=J.transpose()*taox;


        Thrust_torque = A.inverse()  * tao.block(0,0,6,1);
        //计算关节期望角度
        double theta1_goal=0, theta2_goal=0, theta4_goal=0, theta5_goal=0;
        //TO DO：万向锁问题尚未解决
        //r1动力组由于用于栖息，暂假设栖息面法向量0,0,1,所以theta1和theta2的期望值设置为始终使r1推力方向指向栖息面。r1矢量推力因此只有1维可控
        Eigen::Vector3d vector_r1_IN_B, vector_r2_IN_L3;
        vector_r1_IN_B=R_IB.transpose()*tmp_vector(0,0,1);
        theta2_goal=atan2(-vector_r1_IN_B(1),vector_r1_IN_B(2));
        theta1_goal=atan2(vector_r1_IN_B(0),-vector_r1_IN_B(1)*sin(theta2_goal)+vector_r1_IN_B(2)*cos(theta2_goal));

        //r2动力组由于用于驱动飞行运动，其矢量推力3方向都可控
        vector_r2_IN_L3=compute_R(0,arm_data.pos[2],0).transpose() * tmp_vector(Thrust_torque(1,0),Thrust_torque(2,0),Thrust_torque(3,0));
        theta4_goal=atan2(-vector_r2_IN_L3(1),vector_r2_IN_L3(2));
        theta5_goal=atan2(vector_r2_IN_L3(0),-vector_r2_IN_L3(1)*sin(theta4_goal)+vector_r2_IN_L3(2)*cos(theta4_goal));

        
        //设置关节期望角
        Eigen::MatrixXd theta_goal(5,1);
        if (step_control<pretime/control_period)
            theta_goal<< theta1_goal, theta2_goal, theta3_goal, 0, -0.5;
        else
            theta_goal<< theta1_goal, theta2_goal, theta3_goal, theta4_goal, theta5_goal;
        //差分计算关节期望角速度
        Eigen::MatrixXd theta_diff_goal(5,1); 
        if (step_control==1)  theta_goal_cache=theta_goal;
        theta_diff_goal = (theta_goal-theta_goal_cache)/control_period;
        theta_goal_cache=theta_goal;
        
        if(ifperchsuccess){  
            //Eigen::MatrixXd J_diff(9,11)
            //Eigen::MatrixXd Vx(11,1) ///////
            Eigen::MatrixXd pre_Vx(11,1),x_error(11,1),Vxm(9,1);

            Vxm.block(0,0,3,1)=V_PPM;
            Vxm.block(3,0,3,1)=zyx_diffm;
            Vxm.block(6,0,3,1)=V_EEM;
 
            pre_Vx=Vx;        
            Vx=J*Vq;
            Eigen::MatrixXd Vx_error=Vx;
            Vx_error(9,0)=theta4_diff-theta_diff_goal(3);
            Vx_error(10,0)=theta5_diff-theta_diff_goal(4);        

            x_error.block(0,0,3,1)=P_IP-P_IP_d;
            x_error.block(3,0,3,1)=zyx-zyx_d;
            x_error.block(6,0,3,1)=P_IE-P_IE_d;
            x_error(9,0)=theta4-theta_goal(3);
            x_error(10,0)=theta5-theta_goal(4);
        
            /*
            Eigen::MatrixXd x(11,1);
            x.block(0,0,3,1)=P_IP;
            x.block(3,0,3,1)=zyx;
            x.block(6,0,3,1)=P_IE;
            x(9,0)=theta4;
            x(10,0)=theta5;
            */        
            //full rank
                    
            Eigen::MatrixXd Mx=J.inverse().transpose()*M*J.inverse();
            Eigen::MatrixXd Cx=J.inverse().transpose()*C;//-Mx*J_diff*Vx;       
            Eigen::MatrixXd Gx=J.inverse().transpose()*G;  
                
            //Eigen::MatrixXd taox_ext=J.inverse().transpose()*tao_ext;

            /*
            Eigen::MatrixXd Ki(11,11);
            Ki=Eigen::MatrixXd::Identity(11,11);        
            //external wrench estimator
            if (step_control<pretime/control_period)    
                taox_ext=Eigen::MatrixXd::Zero(11,1);
            else{
                taox_ext=taox_ext+Ki*((Gx+Cx-taox_ext-taox)+Mx*(Vx-pre_Vx));         
            }
            */
            Eigen::MatrixXd Kd(11,11),Dd(11,11),Md(11,11);  
            Kd<< 0.4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0.4, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.6, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0.12, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.6, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 4, 0, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 0, 0.01, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 0, 0, 0.005;
                
            Dd<< 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0.8, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 1.2, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 8, 0, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 0, 0.004, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 0, 0, 0.001;   
            Md=Mx;
            
            //Eigen::MatrixXd taox=Gx-Kd*x_error-Dd*Vx_error;
            taox=Gx;
            tao=J.transpose()*taox;
            //taox=Gx;
            //taox=Gx-Mx*Md.inverse()*(Dd*Vx_error+Kd*x_error)+(Mx*Md.inverse()-Eigen::MatrixXd::Identity(11,11))*taox_ext;
                            

            if(Vx_error(0,0)>=-100 && Vx_error(0,0)<=100)  {           
                /*
                cout<<"taox"<<endl;
                cout<<taox<<endl;
                cout<<"Gx"<<endl;
                cout<<Gx<<endl;
                cout<<"G"<<endl;
                cout<<G<<endl;                          
                cout<<"Vx_error"<<endl;
                cout<<Vx_error<<endl;
                */
                cout<<"J"<<endl; 
                cout<<J<<endl;
                cout<<"J-1"<<endl;
                cout<<J.inverse()<<endl; 
                /*
                cout<<"tao"<<endl;
                cout<<tao<<endl; 
                cout<<endl;  
                */ 
                //cout<<"x_error"<<endl;
                //cout<<x_error<<endl;                     
            } 
        }

        else{

            ////////////////////////////////////////////////////逆动力学控制
            Eigen::MatrixXd temp(3,3), q_error(11,1),Vq_error(11,1),pre_Vq(11,1);
            q_error.block(0,0,3,1)=R_IB.transpose()*(P_IB-P_IB_d);
            temp=0.5*(R_IB_d.transpose()*R_IB-R_IB.transpose()*R_IB_d);
            q_error.block(3,0,3,1)=tmp_vector(-temp(1,2),temp(0,2),-temp(0,1));
            q_error.block(6,0,5,1)=theta-theta_goal;
            pre_Vq=Vq;

            Vq_error=Vq;
            Vq_error.block(6,0,5,1)=theta_diff-theta_diff_goal;
            
            

            Eigen::MatrixXd Ki(11,11);
            Ki=Eigen::MatrixXd::Identity(11,11);        
            //external wrench estimator
            if (step_control<pretime/control_period)    
                tao_ext=Eigen::MatrixXd::Zero(11,1);
            else{
                tao_ext=tao_ext+Ki*((G+C-tao_ext-tao)+M*(Vq-pre_Vq));         
                //cout<<"tao_ext"<<endl;
                //cout<<tao_ext<<endl;  
            }
            taox_ext=J.inverse().transpose()*tao_ext;            

            Eigen::MatrixXd Kd(11,11);
            Eigen::MatrixXd Dd(11,11);              
            Kd<< 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 1.2, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.6, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 10, 0, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 0, 1, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 0, 0, 0.5;
            ///8->4    
            Dd<< 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0.8, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 1.2, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0.004, 0, 0, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 0.2, 0, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 0, 0.004, 0,
                0 ,0 ,0, 0, 0, 0, 0, 0, 0, 0, 0.001;   


            Eigen::MatrixXd Md(11,11);
            Md=M;
            //tao=G-M*Md.inverse()*(Dd*Vq_error+Kd*q_error)+(M*Md.inverse()-Eigen::MatrixXd::Identity(11,11))*tao_ext; 
            //保留自然惯性的逆动力学控制（忽略期望加速度项，C应该加的，但加上好像不鲁帮，所以这里不加）
            tao=  -Kd*q_error-Dd*Vq_error + G;
            //tao=G;            

        }
        ////////////////////////////////////////////////计算转子油门
        //这个扭矩抑制非常重要，如果阈值设太大的话，会出现共振
        Eigen::Vector4d motor_command_output;
        double threshold=0.01,threshold2=3.2,threshold3=4.4;
        if (Thrust_torque(4,0)>threshold)
            Thrust_torque(4,0)=threshold;
        if (Thrust_torque(4,0)<-threshold)
            Thrust_torque(4,0)=-threshold;            
        if (Thrust_torque(5,0)>threshold)
            Thrust_torque(5,0)=threshold;
        if (Thrust_torque(5,0)<-threshold)
            Thrust_torque(5,0)=-threshold; 
        /*
        Eigen::MatrixXd Thrust_torque_copy(6,1);
        Thrust_torque_copy=Thrust_torque;
        if (Thrust_torque_copy(0,0)>threshold2)
            Thrust_torque_copy(0,0)=threshold2;
        if (Thrust_torque_copy(0,0)<-threshold2)
            Thrust_torque_copy(0,0)=-threshold2;            
        if (Thrust_torque_copy(1,0)>threshold3)
            Thrust_torque_copy(1,0)=threshold3;
        if (Thrust_torque_copy(1,0)<-threshold3)
            Thrust_torque_copy(1,0)=-threshold3;     
        if (Thrust_torque_copy(2,0)>threshold3)
            Thrust_torque_copy(2,0)=threshold3;
        if (Thrust_torque_copy(2,0)<-threshold3)
            Thrust_torque_copy(2,0)=-threshold3;  
        if (Thrust_torque_copy(3,0)>threshold3)
            Thrust_torque_copy(3,0)=threshold3;
        if (Thrust_torque_copy(3,0)<-threshold3)
            Thrust_torque_copy(3,0)=-threshold3;         
        motor_command_output << (Thrust_torque_copy(0,0)  -  Thrust_torque_copy(4,0)/(Lr/2))/2,
                         (Thrust_torque_copy(0,0)  +  Thrust_torque_copy(4,0)/(Lr/2))/2,
                         (sqrt(Thrust_torque_copy(1,0)*Thrust_torque_copy(1,0)+Thrust_torque_copy(2,0)*Thrust_torque_copy(2,0)+Thrust_torque_copy(3,0)*Thrust_torque_copy(3,0))  +  Thrust_torque_copy(5,0)/(Lr/2))/2,
                         (sqrt(Thrust_torque_copy(1,0)*Thrust_torque_copy(1,0)+Thrust_torque_copy(2,0)*Thrust_torque_copy(2,0)+Thrust_torque_copy(3,0)*Thrust_torque_copy(3,0))  -  Thrust_torque_copy(5,0)/(Lr/2))/2;        
        */

  
        //发布电机速度 
        Eigen::MatrixXd motor_command(4,1);
        
        motor_command << (Thrust_torque(0,0)  -  Thrust_torque(4,0)/(Lr/2))/2,
                         (Thrust_torque(0,0)  +  Thrust_torque(4,0)/(Lr/2))/2,
                         (sqrt(Thrust_torque(1,0)*Thrust_torque(1,0)+Thrust_torque(2,0)*Thrust_torque(2,0)+Thrust_torque(3,0)*Thrust_torque(3,0))  +  Thrust_torque(5,0)/(Lr/2))/2,
                         (sqrt(Thrust_torque(1,0)*Thrust_torque(1,0)+Thrust_torque(2,0)*Thrust_torque(2,0)+Thrust_torque(3,0)*Thrust_torque(3,0))  -  Thrust_torque(5,0)/(Lr/2))/2;
        mav_msgs::Actuators vel_ref;

        if (step_control<pretime/control_period)    
            {vel_ref.angular_velocities.push_back(0);
            vel_ref.angular_velocities.push_back(0);
            vel_ref.angular_velocities.push_back(0);
            vel_ref.angular_velocities.push_back(0);}
        else
            {
                for (int i=0; i<4; i++)
                {
                    vel_ref.angular_velocities.push_back( sqrt(  (motor_command(i,0) /Cm)>0 ? motor_command(i,0) /Cm : 0 ) );
                    if ((motor_command(i,0) /Cm)<0)
                    cout<<"below zero"<<endl;
                }
            }
        pub_vel.publish(vel_ref);




        ////////////////////////////////////////////////计算关节扭矩
        Eigen::MatrixXd A_tao(5,4);  //A的后5列，描述4个转子油门（N）对五个5个关节扭矩（Nm）产生的广义力影响，注意这种影响只有在完全动力学控制中才能被考虑
        A_tao<<
                                                          0,                                                  0,                                                                                                                                       0,                                                                                                                                     0,
         (Lr*cos(theta1)*cos(theta2))/2 - Ct_Cm*sin(theta1), Ct_Cm*sin(theta1) - (Lr*cos(theta1)*cos(theta2))/2,                                                                                                                                       0,                                                                                                                                     0,
                                                          0,                                                  0, - (sin(theta3)*sin(theta5) - cos(theta3)*cos(theta4)*cos(theta5))*(L3*cos(theta3) + (Lr*sin(theta3))/2) - Ct_Cm*cos(theta5)*sin(theta4), Ct_Cm*cos(theta5)*sin(theta4) - (sin(theta3)*sin(theta5) - cos(theta3)*cos(theta4)*cos(theta5))*(L3*cos(theta3) - (Lr*sin(theta3))/2),
                                                          0,                                                  0,                                                                                                  (Lr*cos(theta5))/2 + Ct_Cm*sin(theta5),                                                                                              - (Lr*cos(theta5))/2 - Ct_Cm*sin(theta5),
                                                          0,                                                  0,                                                                                                                                       0,                                                                                                                                     0;
        Eigen::MatrixXd tao_compensation(5,1);
        if (step_control<pretime/control_period) 
            tao_compensation<<0,0,0,0,0;
        else
            tao_compensation=A_tao*motor_command;
        
     
        std_msgs::Float64 J1_goalcur_msg;
        J1_goalcur_msg.data=tao(6,0)-tao_compensation(0,0);
        pub_J1_goalcur.publish(J1_goalcur_msg);

        std_msgs::Float64 J2_goalcur_msg;
        J2_goalcur_msg.data=tao(7,0)-tao_compensation(1,0);
        pub_J2_goalcur.publish(J2_goalcur_msg);

        std_msgs::Float64 J3_goalcur_msg;
        J3_goalcur_msg.data=tao(8,0)-tao_compensation(2,0);
        pub_J3_goalcur.publish(J3_goalcur_msg);

        std_msgs::Float64 J4_goalcur_msg;
        J4_goalcur_msg.data=tao(9,0)-tao_compensation(3,0);
        pub_J4_goalcur.publish(J4_goalcur_msg);

        std_msgs::Float64 J5_goalcur_msg;
        J5_goalcur_msg.data=tao(10,0)-tao_compensation(4,0);
        pub_J5_goalcur.publish(J5_goalcur_msg);
        
        float a=0.05*(2.0*rand()/RAND_MAX-1.0);
        cout<<a<<endl;
        std_msgs::Float64 estimate_fz;
        estimate_fz.data=taox_ext(2,0);
        pub_plot_estimate_fz.publish(estimate_fz);       
        /*
        std_msgs::Float64 actual_px;
        actual_px.data=taox_ext(0,0);
        pub_plot_actual_px.publish(actual_px);

        std_msgs::Float64 actual_py;
        actual_py.data=taox_ext(1,0);
        pub_plot_actual_py.publish(actual_py);

        std_msgs::Float64 actual_pz;
        actual_pz.data=taox_ext(2,0);
        pub_plot_actual_pz.publish(actual_pz);

        std_msgs::Float64 desire_px;
        desire_px.data=taox_ext(3,0);
        pub_plot_desire_px.publish(desire_px);

        std_msgs::Float64 desire_py;
        desire_py.data=taox_ext(4,0);
        pub_plot_desire_py.publish(desire_py);

        std_msgs::Float64 desire_pz;
        desire_pz.data=taox_ext(5,0);
        pub_plot_desire_pz.publish(desire_pz);       
        

        std_msgs::Float64 actual_rx;
        actual_rx.data=zyxm(0);
        pub_plot_actual_rx.publish(actual_rx);

        std_msgs::Float64 actual_ry;
        actual_ry.data=zyxm(1);
        pub_plot_actual_ry.publish(actual_ry);

        std_msgs::Float64 actual_rz;
        actual_rz.data=zyxm(2);
        pub_plot_actual_rz.publish(actual_rz);

        std_msgs::Float64 desire_rx;
        desire_rx.data=zyx_d(0);
        pub_plot_desire_rx.publish(desire_rx);

        std_msgs::Float64 desire_ry;
        desire_ry.data=zyx_d(1);
        pub_plot_desire_ry.publish(desire_ry);

        std_msgs::Float64 desire_rz;
        desire_rz.data=zyx_d(2);
        pub_plot_desire_rz.publish(desire_rz); 
        
        std_msgs::Float64 actual_ex;
        actual_ex.data=P_IB(0);
        pub_plot_actual_ex.publish(actual_ex);

        std_msgs::Float64 actual_ey;
        actual_ey.data=P_IB(1);
        pub_plot_actual_ey.publish(actual_ey);

        std_msgs::Float64 actual_ez;
        actual_ez.data=P_IB(2)+0.06;
        pub_plot_actual_pz.publish(actual_ez);


        std_msgs::Float64 desire_ex;
        desire_ex.data=P_IB_d(0);
        pub_plot_desire_ex.publish(desire_ex);

        std_msgs::Float64 desire_ey;
        desire_ey.data=P_IB_d(1);
        pub_plot_desire_ey.publish(desire_ey);

        std_msgs::Float64 desire_ez;
        desire_ez.data=P_IB_d(2);
        pub_plot_desire_pz.publish(desire_ez); 

        std_msgs::Float64 motor1;
        motor1.data=motor_command_output(0);
        pub_plot_motor1.publish(motor1);

        std_msgs::Float64 motor2;
        motor2.data=motor_command_output(1);
        pub_plot_motor2.publish(motor2);

        std_msgs::Float64 motor3;
        motor3.data=motor_command_output(2);
        pub_plot_motor3.publish(motor3);

        std_msgs::Float64 motor4;
        motor4.data=motor_command_output(3);
        pub_plot_motor4.publish(motor4); 
        */
        step_control+=1;        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

