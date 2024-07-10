%%b系建立在IMU的中心

%两连杆长度
L1=0.3;
L2=0.3;

%扭矩系数
Cm=0.02;
Cd=0.1;

%两连杆质量
m1=0.6;
m2=0.5;

%关节角度
t1=0;
t2=0;
t3=1;
t4=0;
t5=0;


%b系到中间关节的偏置
p_b_j3=[-0.1;0;-0.02];
%b系到连杆1质心的偏置
p_b_L1COM=[0.1;0;-0.02];
%中间关节到连杆2质心的偏置
p_j3_L2COM=[-0.22;0;0];

%计算质量中心位置
p_b_COM=(p_b_L1COM*m1+(p_b_j3+compute_R(0,t3,0)*p_j3_L2COM)*m2)/(m1+m2);

%计算b系到两动力中心的位置
p_br1=[L1;0;0];
p_br2=p_b_j3+compute_R(0,t3,0)*[-L2;0;0];

%扭矩相关映射矩阵
M1=Cm*compute_R(t2,0,0)*compute_R(0,t1,0)*[0;0;1] + Cd*compute_R(t2,0,0)*compute_R(0,t1,0)*[1;0;0];
M2=Cm*compute_R(0,t3,0)*compute_R(t4,0,0)*compute_R(0,t5,0)*[0;0;1] + Cd*compute_R(0,t3,0)*compute_R(t4,0,0)*compute_R(0,t5,0)*[1;0;0];



J=[eye(3),eye(3),zeros(3,2);skew(p_br1),skew(p_br2),M1,M2];



u = quadprog(diag([1,1,10,1,1,10,1,1]),[],[eye(8);-eye(8)],[[50;50;100;50;50;100;10;10];[50;50;100;50;50;100;10;10]],J,[50;0;50;0;0;0]);


%如果固定转子的那一端呢
J=[compute_R(t2,0,0)*compute_R(0,t1,0)*[0;0;1],eye(3),skew(M1)*(-p_br1),skew(M2)*(-p_br2);skew(p_br2)*compute_R(t2,0,0)*compute_R(0,t1,0)*[0;0;1],skew(p_br2),M1,M2];

cond(J)
%说明不行，变成欠驱动了

