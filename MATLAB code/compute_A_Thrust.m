syms theta1 theta2 theta3 Lb L3 Lr Ct_Cm real
syms Thrust_theta45_1 Thrust_theta45_2 Thrust_theta45_3 Thrust_theta45_4 Thrust_theta45_5 Thrust_theta45_6 real


% theta1=1;
% theta2=0;
% theta3=1;
% Lb=0.3;
% L3=0.3;
% Lr=0.16;
% Cm=1;
% Ct=0.02;
% temp=[0.558166 0.640555 0.23937 0.148205 -0.765663 -0.880184];
% Thrust_theta45_1=temp(1);
% Thrust_theta45_2=temp(2);
% Thrust_theta45_3=temp(3);
% Thrust_theta45_4=temp(4);
% Thrust_theta45_5=temp(5);
% Thrust_theta45_6=temp(6);



A(:,1)=[   (compute_R(theta2,0,0)*compute_R(0,theta1,0))'*[eye(3),-skew(Lb/2*[1;0;0]+compute_R(theta2,0,0)*Lr/2*[0;1;0])]
           (compute_R(theta2,0,0)*compute_R(0,theta1,0))'*[zeros(3,3),eye(3)]     ]'          *        [0;0;1;0;0;-Ct_Cm];
A(:,2)=[   (compute_R(theta2,0,0)*compute_R(0,theta1,0))'*[eye(3),-skew(Lb/2*[1;0;0]+compute_R(theta2,0,0)*-Lr/2*[0;1;0])]
           (compute_R(theta2,0,0)*compute_R(0,theta1,0))'*[zeros(3,3),eye(3)]     ]'          *        [0;0;1;0;0;Ct_Cm];
A(:,3)=[   (compute_R(0,theta3,0)*compute_R(Thrust_theta45_5,0,0)*compute_R(0,Thrust_theta45_6,0))'*      [eye(3),        -skew(-Lb/2*[1;0;0]+compute_R(0,theta3,0)*-L3*[1;0;0]+compute_R(0,theta3,0)*compute_R(Thrust_theta45_5,0,0)*Lr/2*[0;1;0])]
           (compute_R(0,theta3,0)*compute_R(Thrust_theta45_5,0,0)*compute_R(0,Thrust_theta45_6,0))'*      [zeros(3,3),        eye(3)]     ]'      *        [0;0;1;0;0;Ct_Cm];
A(:,4)=[   (compute_R(0,theta3,0)*compute_R(Thrust_theta45_5,0,0)*compute_R(0,Thrust_theta45_6,0))'*      [eye(3),        -skew(-Lb/2*[1;0;0]+compute_R(0,theta3,0)*-L3*[1;0;0]+compute_R(0,theta3,0)*compute_R(Thrust_theta45_5,0,0)*-Lr/2*[0;1;0])]
           (compute_R(0,theta3,0)*compute_R(Thrust_theta45_5,0,0)*compute_R(0,Thrust_theta45_6,0))'*      [zeros(3,3),        eye(3)]     ]'      *        [0;0;1;0;0;-Ct_Cm];

  

A*[Thrust_theta45_1;Thrust_theta45_2;Thrust_theta45_3;Thrust_theta45_4]



       
% A(1,:)*[Thrust_theta45_1;Thrust_theta45_2;Thrust_theta45_3;Thrust_theta45_4]
% A(2,:)*[Thrust_theta45_1;Thrust_theta45_2;Thrust_theta45_3;Thrust_theta45_4]
% A(3,:)*[Thrust_theta45_1;Thrust_theta45_2;Thrust_theta45_3;Thrust_theta45_4]
% A(4,:)*[Thrust_theta45_1;Thrust_theta45_2;Thrust_theta45_3;Thrust_theta45_4]
% A(5,:)*[Thrust_theta45_1;Thrust_theta45_2;Thrust_theta45_3;Thrust_theta45_4]
% A(6,:)*[Thrust_theta45_1;Thrust_theta45_2;Thrust_theta45_3;Thrust_theta45_4]
