syms vb vb_diff [3 1] real       %vb是表达在b系下的速度，vb_diff是表达在b系下的加速度
syms wb wb_diff [3 1] real       %vb是表达在b系下的角速度，wb_diff是表达在b系下的角加速度
syms theta1 theta1_diff theta1_diff2 real     %_diff是一阶导，_diff2是二阶导
syms theta2 theta2_diff theta2_diff2 real
syms theta3 theta3_diff theta3_diff2 real
syms theta4 theta4_diff theta4_diff2 real
syms theta5 theta5_diff theta5_diff2 real

syms Lb L3 real

syms R_IB [3 3] real
syms m1 m2 m5 m4 m3 mb real
syms I11 I12 I13 I21 I22 I23 I51 I52 I53 I41 I42 I43 I31 I32 I33 Ib1 Ib2 Ib3 real  %假设各连杆惯性张量为对角矩阵


%%%%%%%%%%%%%%前向计算vb,wb,vb_diff,wb_diff
%右臂(旋转)
w2=compute_R(theta2,0,0)'*wb ...  
    + theta2_diff*[1;0;0];
w2_d=compute_R(theta2,0,0)'*wb_diff + compute_R(theta2,0,0)'*-skew(theta2_diff*[1;0;0])*wb ... 
      + theta2_diff2*[1;0;0];
w1=compute_R(0,theta1,0)'*w2 ...
    + theta1_diff*[0;1;0];
w1_d=compute_R(0,theta1,0)'*w2_d + compute_R(0,theta1,0)'*-skew(theta1_diff*[0;1;0])*w2 ...
      + theta1_diff2*[0;1;0];
%右臂(平移)
v2=compute_R(theta2,0,0)'*vb ...
    + compute_R(theta2,0,0)'*skew(wb)*[Lb/2;0;0];
v2_d=compute_R(theta2,0,0)'*vb_diff + compute_R(theta2,0,0)'*-skew(theta2_diff*[1;0;0])*vb ... 
      + compute_R(theta2,0,0)'*skew(wb_diff)*[Lb/2;0;0] + compute_R(theta2,0,0)'*-skew(theta2_diff*[1;0;0])*skew(wb)*[Lb/2;0;0];
v1=compute_R(0,theta1,0)'*v2;
v1_d=compute_R(0,theta1,0)'*v2_d + compute_R(0,theta1,0)'*-skew(theta1_diff*[0;1;0])*v2;
%左臂(旋转)
w3=compute_R(0,theta3,0)'*wb ...
    + theta3_diff*[0;1;0];
w3_d=compute_R(0,theta3,0)'*wb_diff + compute_R(0,theta3,0)'*-skew(theta3_diff*[0;1;0])*wb ...
      + theta3_diff2*[0;1;0];
w4=compute_R(theta4,0,0)'*w3 ...
    + theta4_diff*[1;0;0];
w4_d=compute_R(theta4,0,0)'*w3_d + compute_R(theta4,0,0)'*-skew(theta4_diff*[1;0;0])*w3 ...
      + theta4_diff2*[1;0;0];
w5=compute_R(0,theta5,0)'*w4 ...
    + theta5_diff*[0;1;0];
w5_d=compute_R(0,theta5,0)'*w4_d + compute_R(0,theta5,0)'*-skew(theta5_diff*[0;1;0])*w4 ...
      + theta5_diff2*[0;1;0];
%左臂(平移)
v3=compute_R(0,theta3,0)'*vb ...
    + compute_R(0,theta3,0)'*skew(wb)*[-Lb/2;0;0] ...
    + skew(w3)*[-2*L3/3;0;0];
v3_d=compute_R(0,theta3,0)'*vb_diff + compute_R(0,theta3,0)'*-skew(theta3_diff*[0;1;0])*vb ...
      + compute_R(0,theta3,0)'*skew(wb_diff)*[-Lb/2;0;0] + compute_R(0,theta3,0)'*-skew(theta3_diff*[0;1;0])*skew(wb)*[-Lb/2;0;0] ...
      + skew(w3_d)*[-2*L3/3;0;0];
v4=compute_R(theta4,0,0)'*v3 ...
    + compute_R(theta4,0,0)'*skew(w3)*[-L3/3;0;0];
v4_d=compute_R(theta4,0,0)'*v3_d + compute_R(theta4,0,0)'*-skew(theta4_diff*[1;0;0])*v3 ...
      + compute_R(theta4,0,0)'*skew(w3_d)*[-L3/3;0;0] + compute_R(theta4,0,0)'*-skew(theta4_diff*[1;0;0])*skew(w3)*[-L3/3;0;0];
v5=compute_R(theta5,0,0)'*v4;
v5_d=compute_R(0,theta5,0)'*v4_d + compute_R(0,theta5,0)'*-skew(theta5_diff*[0;1;0])*v4;





%重力向量
g=[0;0;9.81];
%%%%%%%%%%%%%%反向计算力/力矩
%右臂
F1=m1*v1_d + m1*(R_IB*compute_R(theta2,0,0)*compute_R(0,theta1,0))'*g;
N1=diag([I11,I12,I13])*w1_d + skew(w1)*diag([I11,I12,I13])*w1;
f1=F1;
n1=N1;
tao1=n1'*[0;1;0];   
F2=m2*v2_d + m2*(R_IB*compute_R(theta2,0,0))'*g;
N2=diag([I21,I22,I23])*w2_d + skew(w2)*diag([I21,I22,I23])*w2;
f2=F2 + compute_R(0,theta1,0)*f1;
n2=N2 + compute_R(0,theta1,0)*n1;
tao2=n2'*[1;0;0];
%左臂
F5=m5*v5_d + m5*(R_IB*compute_R(0,theta3,0)*compute_R(theta4,0,0)*compute_R(0,theta5,0))'*g;
N5=diag([I51,I52,I53])*w5_d + skew(w5)*diag([I51,I52,I53])*w5;
f5=F5;
n5=N5;
tao5=n5'*[0;1;0];
F4=m4*v4_d + m4*(R_IB*compute_R(0,theta3,0)*compute_R(theta4,0,0))'*g;
N4=diag([I41,I42,I43])*w4_d + skew(w4)*diag([I41,I42,I43])*w4;
f4=F4 + compute_R(0,theta5,0)*f5;
n4=N4 + compute_R(0,theta5,0)*n5;
tao4=n4'*[1;0;0];
F3=m3*v3_d + m3*(R_IB*compute_R(0,theta3,0))'*g;
N3=diag([I31,I32,I33])*w3_d + skew(w3)*diag([I31,I32,I33])*w3;
f3=F3 + compute_R(theta4,0,0)*f4;
n3=N3 + compute_R(theta4,0,0)*n4 + skew([-2*L3/3;0;0])*F3 + skew([-L3;0;0])*compute_R(theta4,0,0)*f4;
tao3=n3'*[0;1;0];
%机身系
Fb=mb*vb_diff + mb*R_IB'*g;
Nb=diag([Ib1,Ib2,Ib3])*wb_diff + skew(wb)*diag([Ib1,Ib2,Ib3])*wb;
fb=Fb + compute_R(theta2,0,0)*f2 + compute_R(0,theta3,0)*f3;
nb=Nb + compute_R(theta2,0,0)*n2 + compute_R(0,theta3,0)*n3 + skew([Lb/2;0;0])*compute_R(theta2,0,0)*f2 + skew([-Lb/2;0;0])*compute_R(0,theta3,0)*f3;



tao_index=[fb',nb',tao1,tao2,tao3,tao4,tao5];
q_dd_index=[vb_diff1,vb_diff2,vb_diff3,wb_diff1,wb_diff2,wb_diff3,theta1_diff2,theta2_diff2,theta3_diff2,theta4_diff2,theta5_diff2];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%提取动力学参数
%第一步，提取二阶导的系数，得到惯性矩阵
syms B [11 11] real
for i=1:11
    for j=1:11
        if size(coeffs(tao_index(i),q_dd_index(j)),2)==1
            M(i,j)=0;
        else
            temp=coeffs(tao_index(i),q_dd_index(j));
            M(i,j)=temp(2);   %一次项的系数
        end
    end
end

M=simplify(M)  %simplify很关键,大大简化了表达式



%第二步，将二阶导数和一阶导数项置零，得到重力补偿项
syms G [11 1] real
for i=1:11
    G(i,1)=subs(tao_index(i),[vb',vb_diff',wb',wb_diff',theta1_diff2,theta2_diff2,theta3_diff2,theta4_diff2,theta5_diff2,theta1_diff,theta2_diff,theta3_diff,theta4_diff,theta5_diff],zeros(1,22));
end
G=simplify(G)


%第三步，将二阶导数置零，并减去重力补偿项，得到科氏力/离心力项
syms C [11 1] real
for i=1:11
    C(i,1)=subs(tao_index(i),[vb_diff',wb_diff',theta1_diff2,theta2_diff2,theta3_diff2,theta4_diff2,theta5_diff2],zeros(1,11))-G(i,1);
end
C=simplify(C)



























