syms theta1 theta2 theta3 theta4 theta5 Lb L3 Lr Ct_Cm Lr real
syms Thrust_torque1 Thrust_torque2 Thrust_torque3 Thrust_torque4 Thrust_torque5 Thrust_torque6 Thrust_torque7 Thrust_torque8 real
syms R_IB1_1 R_IB1_2 R_IB1_3 R_IB2_1 R_IB2_2 R_IB2_3 R_IB3_1 R_IB3_2 R_IB3_3 real
R_IB=[R_IB1_1,R_IB1_2,R_IB1_3;R_IB2_1,R_IB2_2,R_IB2_3;R_IB3_1,R_IB3_2,R_IB3_3];


theta1=0.5;
theta2=0;
theta3=1;
theta4=0;
theta5=-0.5;
Lr=0.1;
Lb=0.3;
L3=0.3;
Ct_Cm=0.02;
R_IB=compute_R(0,-0.5,0);


M1=compute_R(theta2,0,0)*compute_R(0,theta1,0)*[0;0;1] - Ct_Cm/Lr*compute_R(theta2,0,0)*compute_R(0,theta1,0)*[1;0;0];
M2=compute_R(0,theta3,0)*compute_R(theta4,0,0)*compute_R(0,theta5,0)*[0;0;1] + Ct_Cm/Lr*compute_R(0,theta3,0)*compute_R(theta4,0,0)*compute_R(0,theta5,0)*[1;0;0];



J=[(R_IB)'*[0;0;1],eye(3),zeros(3,2);skew([Lb/2;0;0])*(R_IB)'*[0;0;1],skew([-Lb/2;0;0]+compute_R(0,theta3,0)*[-L3;0;0]),M1,M2]
















