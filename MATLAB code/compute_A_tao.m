% syms theta1 theta2 theta3 theta4 theta5 Lb L3 Lr Ct_Cm real
% syms Thrust_theta45_1 Thrust_theta45_2 Thrust_theta45_3 Thrust_theta45_4 Thrust_theta45_5 Thrust_theta45_6 real


theta1=0.5;
theta2=0;
theta3=1;
theta4=0;
theta5=-0.5;
Lb=0.3;
L3=0.3;
Lr=0.16;
Ct_Cm=0.02;




A_tao(:,1)=[   (compute_R(theta2,0,0)*compute_R(0,theta1,0))'*[zeros(3,1),skew(compute_R(theta2,0,0)*[1;0;0])*compute_R(0,theta2,0)*Lr/2*[0;1;0],zeros(3,3)]
           (compute_R(theta2,0,0)*compute_R(0,theta1,0))'*[compute_R(theta2,0,0)*[0;1;0],[1;0;0],zeros(3,3)]     ]'          *        [0;0;1;0;0;-Ct_Cm];
A_tao(:,2)=[   (compute_R(theta2,0,0)*compute_R(0,theta1,0))'*[zeros(3,1),skew(compute_R(theta2,0,0)*[1;0;0])*compute_R(0,theta2,0)*-Lr/2*[0;1;0],zeros(3,3)]
           (compute_R(theta2,0,0)*compute_R(0,theta1,0))'*[compute_R(theta2,0,0)*[0;1;0],[1;0;0],zeros(3,3)]     ]'          *        [0;0;1;0;0;Ct_Cm];
A_tao(:,3)=[   (compute_R(0,theta3,0)*compute_R(theta4,0,0)*compute_R(0,theta5,0))'*      [zeros(3,2),  skew([0;1;0])*compute_R(0,0,theta3)*(-L3*[1;0;0]+Lr/2*[0;1;0]), skew(compute_R(0,theta3,0)*[1;0;0])*compute_R(0,theta3,0)*compute_R(theta4,0,0)*Lr/2*[0;1;0],zeros(3,1)]
           (compute_R(0,theta3,0)*compute_R(theta4,0,0)*compute_R(0,theta5,0))'*      [zeros(3,2),[0;1;0],compute_R(0,theta3,0)*[1;0;0],compute_R(0,theta3,0)*compute_R(theta4,0,0)*[0;1;0]]     ]'      *        [0;0;1;0;0;Ct_Cm];
A_tao(:,4)=[   (compute_R(0,theta3,0)*compute_R(theta4,0,0)*compute_R(0,theta5,0))'*      [zeros(3,2),  skew([0;1;0])*compute_R(0,0,theta3)*(-L3*[1;0;0]-Lr/2*[0;1;0]), skew(compute_R(0,theta3,0)*[1;0;0])*compute_R(0,theta3,0)*compute_R(theta4,0,0)*-Lr/2*[0;1;0],zeros(3,1)]
           (compute_R(0,theta3,0)*compute_R(theta4,0,0)*compute_R(0,theta5,0))'*      [zeros(3,2),[0;1;0],compute_R(0,theta3,0)*[1;0;0],compute_R(0,theta3,0)*compute_R(theta4,0,0)*[0;1;0]]     ]'      *        [0;0;1;0;0;-Ct_Cm];

% A_tao=simplify(A_tao)
A_tao
