
%% 机械臂建模
% 定义各个连杆以及关节类型，默认为转动关节
%           theta      d        a        alpha
L1=Link([     0        0      0      0 ], 'modified'); % [四个DH参数], options
L2=Link([     0        0       0     -pi/2], 'modified');
L3=Link([     0        0      180      0], 'modified');
L4=Link([     0      170.8     0     -pi/2], 'modified');
L5=Link([     0        0       0      pi/2], 'modified');
L6=Link([     0        0       0     -pi/2], 'modified');

robot=SerialLink([L1,L2,L3,L4,L5,L6]); % 将六个连杆组成机械臂
robot.name='6DOF Robotic Arm';
robot.display();
view(3); % 解决robot.teach()和plot的索引超出报错
robot.teach();
T0_6 = robot.fkine([-32.4*pi/180 -72*pi/180 -36*pi/180 0*pi/180 0*pi/180 -0*pi/180]);
disp(T0_6);
