function KUKA_R700()
    clear;
    clc;
L(1)=Link([0 0.15055 0 pi/2]);    
L(2)=Link([0 -0.115 0.246 0]);
L(3)=Link([0 0.1163 0.190 0]);
L(4)=Link([0 -0.1175 0 pi/2]);
L(5)=Link([0 0.1175 0 -pi/2]);
L(6)=Link([0 0.105 0 0]);

ZU3=SerialLink(L,'name','JAKA ZU3');     

%确定画图平面
p0=[0,0,0];       
p1=[0.4,0.4,0.4];
p2=[-0.5,0.5,0.5];
X=p1-p0;B=p2-p0;  
Z=cross(X,B);
Y=cross(Z,X);   
x1=X/norm(X);
y1=Y/norm(Y);
z1=Z/norm(Z); 
R=[x1.',y1.',z1.'];
Js=[R ,p0.'];
Js=[Js;0 0 0 1];
r=norm(p1-p0);  

%圆的轨迹
t=0:0.05:5;
[s,sd,~] = tpoly(0,2*pi,t);       
q1=[-12.7,98.9,-107,158,-114,11]; 
k = q1;
vs =[]; % qs  关节速度
pd=[];%ps  笛卡尔坐标系下的坐标位置
for i = 1:length(s)
    T=[1 0 0 r*cos(s(i));0 1 0 r*sin(s(i));0 0 1 0;0 0 0 1];
    T1=Js*T;       
    q=ZU3.ikunc(T1,k); %求解逆运动
    k=q;
    pd=[pd;T1(1:3,4).'];
    vs=[vs;q];
end
close all;
figure('NumberTitle', 'off', 'Name', '运动图像');
plot2(pd,'r','LineWidth',2);%圆的线宽
hold on;
ZU3.plot(vs,'fps',20);%输出运动图像
%绘制运动曲线
figure('NumberTitle', 'off', 'Name', '关节值随时间变化曲线');
    subplot(4,1,1);%图像布局
    plot(t,pd);  %输出空间位置曲线
    title('空间位置曲线')
    xlabel('秒(s)');
    ylabel('路程(m)')
    
    subplot(4,1,2);   
    plot(t,vs);%输出关节角度曲线
    title('关节角度曲线')  
    xlabel('秒(s)');  
    ylabel('角度(rad)');
    legend('θ1','θ2','θ3','θ4','θ5','θ6');
    
    %计算空间速度并输出空间速度曲线
    vx = -r*sin(s).*sd;
    vy = r*cos(s).*sd;
    vv = r*sd;
    subplot(4,1,3);   
    plot(t,[vx,vy,vv]);
    title('空间速度曲线')
    xlabel('秒(s)');   
    ylabel('速度(m/s)')
    
    %计算关节速度
    omiga=[];
    for i =1:length(s)
        J0=ZU3.jacob0(vs(i,:)');
        dq=J0\[vx(i);vy(i);0;0;0;0];
        omiga=[omiga;dq'];
    end
    subplot(4,1,4);  
    plot(t,rad2deg(omiga));
    title('关节速度曲线')%输出关节速度曲线
    xlabel('秒(s)');  
    ylabel('角速度(rad/s)')
