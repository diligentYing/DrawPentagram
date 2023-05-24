% 定义五角星的半径和中心点坐标
radius = 200; % 半径
center = [500, 200]; % 中心点坐标

% 计算五角星的顶点坐标
theta = linspace(0.5*pi, 2.5*pi, 6); % 分割角度为6个点，多一个点是为了闭合五角星
x = radius * cos(theta) + center(1);
y = radius * sin(theta) + center(2);

% 绘制五边形
figure;
plot(x, y, 'r-', 'LineWidth', 2);
axis equal; % 设置坐标轴比例相等
xlabel('X');
ylabel('Y');
title('Five-pointed Star');
