% %% MOD-DH参数
% %定义连杆的D-H参数
% %连杆偏移
d1 = 400;
d2 = 0;
d3 = 0;
d4 = 365;
d5 = 0;
d6 = 80;
%连杆长度
a1 = 0;
a2 = 25;
a3 = 315;
a4 = 0;
a5 = 0;
a6 = 0;
%连杆扭角
alpha1 = -pi/2;
alpha2 = -pi/2;
alpha3 = 0;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = pi/2;
%建立机器人模型
% theta d a alpha 
L1=Link([0 d1 a1 alpha1 ],'modified');
L2=Link([0 d2 a2 alpha2 ],'modified');L2.offset=-pi/2; % theta2有偏移角-pi/2
L3=Link([pi*0.5 d3 a3 alpha3 ],'modified');L3.offset= pi; % theta3有偏移角pi/2
L4=Link([0 d4 a4 alpha4 ],'modified');
L5=Link([0 d5 a5 alpha5 ],'modified');
L6=Link([0 d6 a6 alpha6 ],'modified');

%限制机器人的关节空间
L1.qlim = [deg2rad(-170) deg2rad(170)];
L2.qlim = [deg2rad(-190) deg2rad(45)];
L3.qlim = [deg2rad(-120) deg2rad(156)];
L4.qlim = [deg2rad(-185) deg2rad(185)];
L5.qlim = [deg2rad(-120) deg2rad(120)];
L6.qlim = [deg2rad(-350) deg2rad(350)];

%连接连杆，机器人取名为myrobot
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','MyKUKArobot');
% q = [0 0 0 0 0 0];
% robot.plot(q,'tilesize',100);
% robot.plot([0 0 0 0 0 0]);%展示机器人模型
% robot.teach;

pos1 = [x(2),y(2)];
pos2 = [x(5),y(5)];
pos3 = [x(3),y(3)];
pos4 = [x(1),y(1)];
pos5 = [x(4),y(4)];
pos6 = [x(2),y(2)];

%齐次位姿：
% T1 = transl(0.4,0.2,0)*trotx(pi) ;
% T2 = transl(0.4,-0.2,0)*trotx(pi/2);
T1 = transl(400,pos1(1),pos1(2)) ;
T2 = transl(400,pos2(1),pos2(2)) ;
T3 = transl(400,pos3(1),pos3(2)) ;
T4 = transl(400,pos4(1),pos4(2)) ;
T5 = transl(400,pos5(1),pos5(2)) ;
T6 = transl(400,pos6(1),pos6(2)) ;

%对于许多应用，我们都需要在笛卡儿空间中的直线运动，这称为笛卡儿运动。
t=[0:0.05:2]';
% Ts=ctraj(T5,T6,[0:0.05:2]);
Ts1=trinterp(T1,T2,[0:0.01:2]); 
Ts2=trinterp(T2,T3,[0:0.01:2]); 
Ts3=trinterp(T3,T4,[0:0.01:2]); 
Ts4=trinterp(T4,T5,[0:0.01:2]); 
Ts5=trinterp(T5,T6,[0:0.01:2]); 
%其输入参数是初始和最后的位姿，以及时间步数，它返回的是一个三维矩阵形式的轨迹。
Ts = cat(3, Ts1(:,:,1:100),Ts2(:,:,1:100),Ts3(:,:,1:100),Ts4(:,:,1:100),Ts5(:,:,1:101));
% figure(1)
%位置分量：
%plot(t,transl(Ts));
%姿态分量：
%plot(t,tr2rpy(Ts))
% robot.plot([0 0 0 0 0 0],'tilesize',20)

%通过逆运动学求解，我们可以得到相应的关节空间轨迹：
qc = robot.ikcon(Ts);
figure(2)
robot.plot(qc,'trail','k','tilesize',20)
timelist = [0:0.01:5]';
out = [timelist qc];
% 导出矩阵数据到文本文件
filename = 'matrix_data.txt';
dlmwrite(filename, out, 'delimiter', '\t');
