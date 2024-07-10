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



#define pi 3.141592653589793

using namespace std;

double control_rate=200;
double control_period=1/control_rate;
unsigned long step_control=1;
double pretime=3;      //准备时间，在这个时间之内先让关节控制到初始角度，在这之后才控制螺旋浆


////////广义关节期望值 (5个关节中只有theta3是人为指定)
double theta3_goal;
Eigen::Vector3d P_IB_d;
Eigen::Matrix3d R_IB_d;


//运动学与动力学参数
double Lb=0.3;
double L3=0.3;
double Lr=0.1; 
double Cm=8.54858e-06;
double Ct_Cm=0.02;          //拉力系数与反扭矩系数的比例
double m1=0.15;
double m2=0.015;
double m5=0.15;
double m4=0.015;
double m3=0.2;
double mb=0.35;



//其他全局变量
Eigen::MatrixXd tao(6,1);   
Eigen::MatrixXd theta_cache(5,1);
Eigen::MatrixXd theta_goal_cache(5,1);
Eigen::MatrixXd Thrust_torque(6,1);      //1x1 r1动力组驱动力，3×1 r2动力组3维驱动力，1×1 r1动力组驱动扭力，1×1 r2动力组驱动扭力 ，更多解释见后面




//////////////////////////////////回调函数
struct pose_msg_t{
    // 位置
    double x,y,z;
    // 四元数
    double qx,qy,qz,qw;
	double vx,vy,vz;
}pose_msg;

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
}
void set_imu(const sensor_msgs::Imu::ConstPtr& msg) {

    imu_data.wx = msg->angular_velocity.x;
    imu_data.wy = msg->angular_velocity.y;
    imu_data.wz = msg->angular_velocity.z;
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

Eigen::Matrix<double, 3, 3> compute_R(double x, double y, double z)
{
	Eigen::Matrix<double, 3, 3> output_matrix;
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

Eigen::Matrix<double, 3, 3> skew(Eigen::Vector3d input)
{
	Eigen::Matrix<double, 3, 3> output;
	output << 0, -input(2),input(1),
			input(2), 0, -input(0),
			-input(1), input(0), 0;
	return output;
}


 





int main ( int argc, char** argv )
{
    //全局变量初始化
    tao<<0,0,0,0,0,0;
    Thrust_torque<<0,0,0,0,0,0;
    bool set_state[20]={0};
    
    //ROS节点定义
    ros::init(argc, argv, "UAM4_dynamics_control");
    ros::NodeHandle n;
    //定义订阅者和发布者
    ros::Subscriber sub_armstate = n.subscribe("/firefly/arm/joint_states", 2, set_armstate);
    ros::Subscriber sub_pose = n.subscribe("/firefly/ground_truth/odometry", 2, set_pose);
    ros::Subscriber sub_imu = n.subscribe("/firefly/ground_truth/imu", 2, set_imu);
    ros::Publisher pub_J1_goalpos= n.advertise<std_msgs::Float64>("/firefly/arm/J1_position_controller/command", 2);    
    ros::Publisher pub_J2_goalpos= n.advertise<std_msgs::Float64>("/firefly/arm/J2_position_controller/command", 2); 
    ros::Publisher pub_J3_goalpos= n.advertise<std_msgs::Float64>("/firefly/arm/J3_position_controller/command", 2); 
    ros::Publisher pub_J4_goalpos= n.advertise<std_msgs::Float64>("/firefly/arm/J4_position_controller/command", 2); 
    ros::Publisher pub_J5_goalpos= n.advertise<std_msgs::Float64>("/firefly/arm/J5_position_controller/command", 2); 
    ros::Publisher pub_vel = n.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 2);

    ros::Publisher pub_plot_actual_px = n.advertise<std_msgs::Float64>("/firefly/plot_actual_px", 2);
    ros::Publisher pub_plot_desire_px = n.advertise<std_msgs::Float64>("/firefly/plot_desire_px", 2);
    ros::Publisher pub_plot_actual_py = n.advertise<std_msgs::Float64>("/firefly/plot_actual_py", 2);
    ros::Publisher pub_plot_desire_py = n.advertise<std_msgs::Float64>("/firefly/plot_desire_py", 2);
    ros::Publisher pub_plot_actual_pz = n.advertise<std_msgs::Float64>("/firefly/plot_actual_pz", 2);
    ros::Publisher pub_plot_desire_pz = n.advertise<std_msgs::Float64>("/firefly/plot_desire_pz", 2);

    ros::Publisher pub_plot_actual_rx = n.advertise<std_msgs::Float64>("/firefly/plot_actual_rx", 2);
    ros::Publisher pub_plot_desire_rx = n.advertise<std_msgs::Float64>("/firefly/plot_desire_rx", 2);
    ros::Publisher pub_plot_actual_ry = n.advertise<std_msgs::Float64>("/firefly/plot_actual_ry", 2);
    ros::Publisher pub_plot_desire_ry = n.advertise<std_msgs::Float64>("/firefly/plot_desire_ry", 2);
    ros::Publisher pub_plot_actual_rz = n.advertise<std_msgs::Float64>("/firefly/plot_actual_rz", 2);
    ros::Publisher pub_plot_desire_rz = n.advertise<std_msgs::Float64>("/firefly/plot_desire_rz", 2);
    //ros循环频率
    ros::Rate loop_rate(control_rate); 



    while(ros::ok()){

        //广义关节期望值设置
        theta3_goal=1;
        P_IB_d << 0,0,0.5;
        R_IB_d= compute_R(0,-0.5,0);


        if (step_control<pretime/control_period)
        {
            if(!set_state[0]){
                cout<<"准备状态"<<endl;
                set_state[0]=1;
            }
        }
        else if (step_control<(pretime+5)/control_period)
        {           
            if(!set_state[1]){
                cout<<"初始悬停"<<endl;
                set_state[1]=1;
            }
        }
        /*
        else if (step_control<(pretime+10)/control_period)
        {            
            if(!set_state[2]){
                cout<<"旋转1"<<endl;
                set_state[2]=1;
            }
            R_IB_d= compute_R(0,0,0);
        }   
        else if (step_control<(pretime+15)/control_period)
        {
            if(!set_state[3]){
                cout<<"初始悬停"<<endl;
                set_state[3]=1;
            }
        }   
        else if (step_control<(pretime+20)/control_period)
        {
            if(!set_state[4]){
                cout<<"旋转2"<<endl;
                set_state[4]=1;
            }
            R_IB_d= compute_R(0,-1.2,0);
        }   
        else if (step_control<(pretime+25)/control_period)
        {
            if(!set_state[5]){
                cout<<"初始悬停"<<endl;
                set_state[5]=1;
            }
        }      
        else if (step_control<(pretime+30)/control_period)
        {
            if(!set_state[6]){
                cout<<"旋转3"<<endl;
                set_state[6]=1;
            }
            R_IB_d= compute_R(0,0,1)*compute_R(0,-0.5,0);
        }  
        else if (step_control<(pretime+35)/control_period)
        {
            if(!set_state[7]){
                cout<<"初始悬停"<<endl;
                set_state[7]=1;
            }
        }          
        else if (step_control<(pretime+40)/control_period)
        {           
            if(!set_state[8]){
                cout<<"平移1"<<endl;
                set_state[8]=1;
            }
            P_IB_d << 0.5,0,0.5;
        }          
        else if (step_control<(pretime+45)/control_period)
        {
            if(!set_state[9]){
                cout<<"初始悬停"<<endl;
                set_state[9]=1;
            }
        }     
        else if (step_control<(pretime+50)/control_period)
        {            
            if(!set_state[10]){
                cout<<"平移2"<<endl;
                set_state[10]=1;
            }
            P_IB_d << 0,0.2,0.5;
        }         
        else if (step_control<(pretime+55)/control_period)
        {
             if(!set_state[11]){
                cout<<"初始悬停"<<endl;
                set_state[11]=1;
            }
        }     
        else if (step_control<(pretime+60)/control_period)
        {
            if(!set_state[12]){
                cout<<"平移3"<<endl;
                set_state[12]=1;
            }
            P_IB_d << 0,0,0.8;
        }  
        else if (step_control<(pretime+65)/control_period)
        {
            if(!set_state[13]){
                cout<<"初始悬停"<<endl;
                set_state[13]=1;
            }
        }
        */   
        else if (step_control<(pretime+10)/control_period)
        {
            if(!set_state[14]){
                cout<<"栖息姿态"<<endl;
                set_state[14]=1;
            }
            R_IB_d= compute_R(0,-1.2,0);
            P_IB_d=tmp_vector(0,0,1.05)-R_IB_d*tmp_vector(Lb/2,0,0);
            
        }  
        else if (step_control<(pretime+18)/control_period)
        {
            if(!set_state[15]){
                cout<<"关节3状态1"<<endl;
                set_state[15]=1;
            }
            theta3_goal=1+0.57 * (step_control-(pretime+10)/control_period)   /  (8/control_period) ;
            R_IB_d= compute_R(0,-1.2,0);
            P_IB_d=tmp_vector(0,0,1.05)-R_IB_d*tmp_vector(Lb/2,0,0);
        }  
        else if (step_control<(pretime+32)/control_period)
        {            
            if(!set_state[15]){
                cout<<"关节3状态2"<<endl;
                set_state[15]=1;
            }
            theta3_goal=1.57-1.07 * (step_control-(pretime+18)/control_period)   /  (8/control_period);
            R_IB_d= compute_R(0,-1.2,0);
            P_IB_d=tmp_vector(0,0,1.05)-R_IB_d*tmp_vector(Lb/2,0,0);
        }
        

    

        //订阅b系位置
        Eigen::MatrixXd P_IB(3,1),R_IB(3,3);
        P_IB << pose_msg.x, pose_msg.y, pose_msg.z;
        //订阅b系速度
        Eigen::MatrixXd V_BB(3,1);
        V_BB<<pose_msg.vx,pose_msg.vy,pose_msg.vz;
        Eigen::MatrixXd V_IB(3,1);
        V_IB=R_IB*V_BB;
        //订阅I到b的旋转矩阵
        R_IB   <<1-2*pose_msg.qy*pose_msg.qy-2*pose_msg.qz*pose_msg.qz  , 2*pose_msg.qx*pose_msg.qy-2*pose_msg.qw*pose_msg.qz ,   2*pose_msg.qx*pose_msg.qz+2*pose_msg.qw*pose_msg.qy,
        2*pose_msg.qx*pose_msg.qy+2*pose_msg.qw*pose_msg.qz   , 1-2*pose_msg.qx*pose_msg.qx-2*pose_msg.qz*pose_msg.qz ,   2*pose_msg.qy*pose_msg.qz-2*pose_msg.qw*pose_msg.qx,
        2*pose_msg.qx*pose_msg.qz-2*pose_msg.qw*pose_msg.qy  ,  2*pose_msg.qy*pose_msg.qz+2*pose_msg.qw*pose_msg.qx  ,  1-2*pose_msg.qx*pose_msg.qx-2*pose_msg.qy*pose_msg.qy;               
        //订阅b系角速度
        Eigen::Vector3d W_BB;
        W_BB << imu_data.wx, imu_data.wy, imu_data.wz; 

        

        //差分计算关节角速度
        Eigen::MatrixXd theta(5,1); 
        theta << arm_data.pos[0], arm_data.pos[1], arm_data.pos[2], arm_data.pos[3], arm_data.pos[4];
        Eigen::MatrixXd theta_diff(5,1); 
        if (step_control==1)  theta_cache=theta;
        theta_diff = (theta-theta_cache)/control_period;
        theta_cache=theta;



        ////////////////动力学参数计算
        double R_IB1_1=R_IB(0,0), R_IB1_2=R_IB(0,1), R_IB1_3=R_IB(0,2), R_IB2_1=R_IB(1,0), R_IB2_2=R_IB(1,1), R_IB2_3=R_IB(1,2), R_IB3_1=R_IB(2,0), R_IB3_2=R_IB(2,1), R_IB3_3=R_IB(2,2);
        double theta1=theta(0,0), theta2=theta(1,0), theta3=theta(2,0), theta4=theta(3,0), theta5=theta(4,0);
        Eigen::MatrixXd G(6,1);


        G<< (981*R_IB3_1*(m1 + m2 + m3 + m4 + m5 + mb))/100,
            (981*R_IB3_2*(m1 + m2 + m3 + m4 + m5 + mb))/100,
            (981*R_IB3_3*(m1 + m2 + m3 + m4 + m5 + mb))/100,
                                                                                                                                                                                                                                                                                                           -(327*L3*R_IB3_2*sin(theta3)*(2*m3 + 3*m4 + 3*m5))/100,
            (981*Lb*R_IB3_3*m3)/200 - (981*Lb*R_IB3_3*m2)/200 - (981*Lb*R_IB3_3*m1)/200 + (981*Lb*R_IB3_3*m4)/200 + (981*Lb*R_IB3_3*m5)/200 + (327*L3*R_IB3_3*m3*cos(theta3))/50 + (981*L3*R_IB3_3*m4*cos(theta3))/100 + (981*L3*R_IB3_3*m5*cos(theta3))/100 + (327*L3*R_IB3_1*m3*sin(theta3))/50 + (981*L3*R_IB3_1*m4*sin(theta3))/100 + (981*L3*R_IB3_1*m5*sin(theta3))/100,
                                                                                                                                                                                                                           -(327*R_IB3_2*(3*Lb*m3 - 3*Lb*m2 - 3*Lb*m1 + 3*Lb*m4 + 3*Lb*m5 + 4*L3*m3*cos(theta3) + 6*L3*m4*cos(theta3) + 6*L3*m5*cos(theta3)))/200;

 

        //J是（6×8） 8×1驱动力向量 到 b系下力/力矩向量 的映射矩阵
        Eigen::MatrixXd J(6,6);

         J<<    R_IB3_1,              1,                       0,                     0,                                                             0,                                                                                                                                                 0,
                R_IB3_2,              0,                       1,                     0,                                                             0,                                                                                                                                                 0,
                R_IB3_3,              0,                       0,                     1,                                                             0,                                                                                                                                                 0,
                0,              0,         -L3*sin(arm_data.pos[2]),                     0,                           sin(arm_data.pos[0]) - Ct_Cm/Lr*cos(arm_data.pos[0]), cos(arm_data.pos[2])*sin(arm_data.pos[4]) + Ct_Cm/Lr*cos(arm_data.pos[2])*cos(arm_data.pos[4]) + cos(arm_data.pos[3])*cos(arm_data.pos[4])*sin(arm_data.pos[2]) - Ct_Cm/Lr*cos(arm_data.pos[3])*sin(arm_data.pos[2])*sin(arm_data.pos[4]),
                -(Lb*R_IB3_3)*0.5, L3*sin(arm_data.pos[2]),                       0, Lb*0.5 + L3*cos(arm_data.pos[2]), - cos(arm_data.pos[0])*sin(arm_data.pos[1]) - Ct_Cm/Lr*sin(arm_data.pos[0])*sin(arm_data.pos[1]),                                                                                       Ct_Cm/Lr*sin(arm_data.pos[3])*sin(arm_data.pos[4]) - cos(arm_data.pos[4])*sin(arm_data.pos[3]),
                (Lb*R_IB3_2)*0.5,              0, - Lb*0.5 - L3*cos(arm_data.pos[2]),                     0,   cos(arm_data.pos[0])*cos(arm_data.pos[1]) + Ct_Cm/Lr*cos(arm_data.pos[1])*sin(arm_data.pos[0]), cos(arm_data.pos[2])*cos(arm_data.pos[3])*cos(arm_data.pos[4]) - Ct_Cm/Lr*cos(arm_data.pos[4])*sin(arm_data.pos[2]) - sin(arm_data.pos[2])*sin(arm_data.pos[4]) - Ct_Cm/Lr*cos(arm_data.pos[2])*cos(arm_data.pos[3])*sin(arm_data.pos[4]);

        Thrust_torque = J.inverse()  * tao;

        
        

        //计算关节期望角度
        double theta1_goal=0, theta2_goal=0, theta4_goal=0, theta5_goal=0;
        //TO DO：万向锁问题尚未解决
        //r1动力组由于用于栖息，暂假设栖息面法向量0,0,1,所以theta1和theta2的期望值设置为始终使r1推力方向指向栖息面。r1矢量推力因此只有1维可控
        Eigen::MatrixXd vector_r1_IN_B(3,1), vector_r2_IN_L3(3,1);
        vector_r1_IN_B=R_IB.transpose()*tmp_vector(0,0,1);
        theta2_goal=atan2(-vector_r1_IN_B(1,0),vector_r1_IN_B(2,0));
        theta1_goal=atan2(vector_r1_IN_B(0,0),-vector_r1_IN_B(1,0)*sin(theta2_goal)+vector_r1_IN_B(2,0)*cos(theta2_goal));

        //r2动力组由于用于驱动飞行运动，其矢量推力3方向都可控
        vector_r2_IN_L3=compute_R(0,arm_data.pos[2],0).transpose() * tmp_vector(Thrust_torque(1,0),Thrust_torque(2,0),Thrust_torque(3,0));
        theta4_goal=atan2(-vector_r2_IN_L3(1,0),vector_r2_IN_L3(2,0));
        theta5_goal=atan2(vector_r2_IN_L3(0,0),-vector_r2_IN_L3(1,0)*sin(theta4_goal)+vector_r2_IN_L3(2,0)*cos(theta4_goal));


    
        //设置关节期望角
        Eigen::MatrixXd theta_goal(5,1);
        if (step_control<pretime/control_period)
            theta_goal<< theta1_goal, theta2_goal, theta3_goal, 0, -0.5;
        else
            theta_goal<< theta1_goal, theta2_goal, theta3_goal, theta4_goal, theta5_goal;



        //重力补偿PD控制
        Eigen::MatrixXd Kd(6,6);
        Eigen::MatrixXd Dd(6,6);   

        Kd<< 4, 0, 0, 0, 0, 0,
             0, 2, 0, 0, 0, 0,
             0, 0, 4, 0, 0, 0,
             0, 0, 0, 0.6, 0, 0,
             0, 0, 0, 0, 1.2, 0,
             0, 0, 0, 0, 0, 0.6;
        Dd<< 8, 0, 0, 0, 0, 0,
             0, 8, 0, 0, 0, 0,
             0, 0, 8, 0, 0, 0,
             0, 0, 0, 1.2, 0, 0,
             0, 0, 0, 0, 0.8, 0,
             0, 0, 0, 0, 0, 1.2;

        Eigen::MatrixXd temp(3,3), eq(6,1), eq_diff(6,1);
        eq.block(0,0,3,1)=R_IB.transpose()*(P_IB-P_IB_d);
        temp=0.5*(R_IB_d.transpose()*R_IB-R_IB.transpose()*R_IB_d);
        eq.block(3,0,3,1)=tmp_vector(-temp(1,2),temp(0,2),-temp(0,1));

        eq_diff.block(0,0,3,1)=V_BB;
        eq_diff.block(3,0,3,1)=W_BB;

        tao=  (-Kd*eq-Dd*eq_diff) + G;
        //tao= G;



        ////////////////////////////////////////////////计算转子油门
        //这个扭矩抑制非常重要，如果阈值设太大的话，会出现共振
        if (Thrust_torque(4,0)>0.1)
            Thrust_torque(4,0)=0.1;
        if (Thrust_torque(4,0)<-0.1)
            Thrust_torque(4,0)=-0.1;            
        if (Thrust_torque(5,0)>0.1)
            Thrust_torque(5,0)=0.1;
        if (Thrust_torque(5,0)<-0.1)
            Thrust_torque(5,0)=-0.1;        
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



        ////////////////////////////////////////////////发布关节位置
        std_msgs::Float64 J1_goalpos_msg;
        J1_goalpos_msg.data=theta_goal(0,0);
        pub_J1_goalpos.publish(J1_goalpos_msg);

        std_msgs::Float64 J2_goalpos_msg;
        J2_goalpos_msg.data=theta_goal(1,0);
        pub_J2_goalpos.publish(J2_goalpos_msg);

        std_msgs::Float64 J3_goalpos_msg;
        J3_goalpos_msg.data=theta_goal(2,0);
        pub_J3_goalpos.publish(J3_goalpos_msg);

        std_msgs::Float64 J4_goalpos_msg;
        J4_goalpos_msg.data=theta_goal(3,0);
        pub_J4_goalpos.publish(J4_goalpos_msg);

        std_msgs::Float64 J5_goalpos_msg;
        J5_goalpos_msg.data=theta_goal(4,0);
        pub_J5_goalpos.publish(J5_goalpos_msg);
        
        std_msgs::Float64 actual_px;
        actual_px.data=P_IB(0);
        pub_plot_actual_px.publish(actual_px);

        std_msgs::Float64 actual_py;
        actual_py.data=P_IB(1);
        pub_plot_actual_py.publish(actual_py);

        std_msgs::Float64 actual_pz;
        actual_pz.data=P_IB(2);
        pub_plot_actual_pz.publish(actual_pz);

        std_msgs::Float64 desire_px;
        desire_px.data=P_IB_d(0);
        pub_plot_desire_px.publish(desire_px);

        std_msgs::Float64 desire_py;
        desire_py.data=P_IB_d(1);
        pub_plot_desire_py.publish(desire_py);

        std_msgs::Float64 desire_pz;
        desire_pz.data=P_IB_d(2);
        pub_plot_desire_pz.publish(desire_pz);        

        Eigen::Vector3d zyxm;
        zyxm<<atan2(R_IB(2,1),R_IB(2,2)),
             atan2(-R_IB(2,0),sqrt(R_IB(2,1)*R_IB(2,1)+R_IB(2,2)*R_IB(2,2))),
             atan2(R_IB(1,0),R_IB(0,0));

        Eigen::Vector3d zyx_d;
        zyx_d<<atan2(R_IB_d(2,1),R_IB_d(2,2)),
             atan2(-R_IB_d(2,0),sqrt(R_IB_d(2,1)*R_IB_d(2,1)+R_IB_d(2,2)*R_IB_d(2,2))),
             atan2(R_IB_d(1,0),R_IB_d(0,0));

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

        step_control+=1;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

