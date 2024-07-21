# 异形空中机器人的设计与控制
本项目主要针对高空建筑清洁和桥梁检测等无人机应用场景，设计一款以多关节、分布式旋翼、可变形为特征的异形无人机，实现在桥梁等环境表面利用近面效应栖息并变形作业的任务。克服了传统无人机缺乏灵活性、能源损耗较大、作业精度不高的缺点。
本项目受到[Nishio 等提出的栖息作业的异形空中机器人](https://ieeexplore.ieee.org/document/9561923)的启发，克服了其无法自由控制栖息方向、未考虑飞行到栖息的过渡情况、使用力传感器增加重量的缺点。
## 无人机模型
"机器人模型"文件夹中，采用solidwork构建机器人仿真及实物模型。

<img width="300" src="https://github.com/WillianYe/aerial-manipulator/blob/main/img/img1.png"/></div>
<img width="600" src="https://github.com/WillianYe/aerial-manipulator/blob/main/img/img2.png"/></div>
## 无人机建模和控制器设计
"Matlab Code"文件夹中，进行运动学建模、准静态动力学和完整动力学两种建模方法，采用阻抗控制和重力补偿 PD 控制两种控制方法设计控制器，采用非线性优化和线性求解两种方法进行推力分配。
"aerial_manipulator"文件夹实现了控制器。

<img width="350" src="https://github.com/WillianYe/aerial-manipulator/blob/main/img/img3.png"/></div>
<img width="500" src="https://github.com/WillianYe/aerial-manipulator/blob/main/img/img4.png"/></div>
## 无人机仿真实验
无人机仿真使用了[rotors_simulator](https://github.com/ethz-asl/rotors_simulator)的gazebo模拟器框架。
采用完整动力学控制时：
```sh
roslaunch rotors_gazebo MY_UAM_dynamics.launch
rosrun aerial-manipulator UAM4_dynamics_control_sim_complete
```
采用准静态动力学控制时，将rotors_simulator/rotors_gazebo/launch/MY_UAM_dynamics.launch中的"effort"替换为"position":
```sh
roslaunch rotors_gazebo MY_UAM_dynamics.launch
rosrun aerial-manipulator UAM4_dynamics_control_sim_static
```
仿真实验执行两个控制任务：在自由空间保证稳定飞行，已知粗略的栖息面位置和方向，能自主过渡到栖息状态；保持栖息的稳定性，已知期望执行器位置，末端执行器能实现轨迹跟踪。

<img width="300" src="https://github.com/WillianYe/aerial-manipulator/blob/main/img/img5.png"/></div>
<img width="500" src="https://github.com/WillianYe/aerial-manipulator/blob/main/img/img6.png"/></div>

