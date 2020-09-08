%% 数据处理
% motorTheta = [radian0 radian1 radian2 radian3 radian4]
clc
clear
close all
filename='data/stateRecord06.txt';
[time,radian1,velocity1,current1,radian4,velocity4,current4,radian2,velocity2,current2,radian3,velocity3,current3,radian0,velocity0,current0]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d','delimiter',',');

filename2='data/d_traRecord06.txt';
[time,dp1,dp4,dp2,dp3,dp0]=textread(filename2,'%f %*d %*d %*d %*d %*d %d %d %d %d %d','delimiter',',');


t=1:1:length(radian1);
t=0.1*t;
t1=1:1:length(dp3);
t1=0.1*t1;
% 角度
figure(100),
radian0=(radian0-radian0(1))*0.088;
radian1=(radian1-radian1(1))*0.088;
radian2=(radian2-radian2(1))*0.088;
radian3=(radian3-radian3(1))*0.088;
radian4=(radian4-radian4(1))*0.088;

hold on
plot(t,radian0', 'linewidth', 1.1,'color','#EDB120');
plot(t,radian1', 'linewidth', 1.1,'color','#0072BD');
plot(t,radian2', 'linewidth', 1.1,'color','#D95319');
plot(t,radian3', 'linewidth', 1.1,'color','#7E2F8E');
plot(t,radian4', 'linewidth', 1.1,'color','#77AC30');

dp1=(dp1)*0.088;
dp4=(dp4)*0.088;
dp2=(dp2)*0.088;
dp3=(dp3)*0.088;
dp0=(dp0)*0.088;

plot(t1,dp0', 'linewidth', 1.1,'linestyle','--','color','#EDB120');
plot(t1,dp1', 'linewidth', 1.1,'linestyle','--','color','#0072BD');
plot(t1,dp2', 'linewidth', 1.1,'linestyle','--','color','#D95319');
plot(t1,dp3', 'linewidth', 1.1,'linestyle','--','color','#7E2F8E');
plot(t1,dp4', 'linewidth', 1.1,'linestyle','--','color','#77AC30');


set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time'); 
ylabel('Radian ') ;


figure(200),
hold on
plot(t,velocity0', 'linewidth', 1.1,'color','#EDB120');
plot(t,velocity1', 'linewidth', 1.1,'color','#0072BD');
plot(t,velocity2', 'linewidth', 1.1,'color','#D95319');
plot(t,velocity3', 'linewidth', 1.1,'color','#7E2F8E');
plot(t,velocity4', 'linewidth', 1.1,'color','#77AC30');
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') ;
xlabel('Time'); 
ylabel('Velocity ') ;



figure(300),
hold on
plot(t,current0', 'linewidth', 1.1,'color','#EDB120');
plot(t,current1', 'linewidth', 1.1,'color','#0072BD');
plot(t,current2', 'linewidth', 1.1,'color','#D95319');
plot(t,current3', 'linewidth', 1.1,'color','#7E2F8E');
plot(t,current4', 'linewidth', 1.1,'color','#77AC30');
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time'); 
ylabel('Current ') ;

%% optitrack 数据处理
clc
clear
close all
fileName = 'data/test02.xlsx';
sheet = 2;
xlRange = 'A8:W4397';
step = 10;
data = xlsread(fileName,sheet,xlRange);
index = data(1:step:end,1);
timeStamp = data(1:step:end,2);
% rigidBody 1
Marker1.x = data(1:step:end,3);Marker1.y = data(1:step :end,4);Marker1.z = data(1:step :end,5);
Marker2.x = data(1:step :end,6);Marker2.y = data(1:step :end,7);Marker2.z = data(1:step :end,8);
Marker3.x = data(1:step :end,9);Marker3.y = data(1:step :end,10);Marker3.z = data(1:step :end,11);

%rigidBody 2
Marker4.x = data(1:step :end,12);Marker4.y = data(1:step :end,13);Marker4.z = data(1:step :end,14);
Marker5.x = data(1:step :end,15);Marker5.y = data(1:step :end,16);Marker5.z = data(1:step :end,17);
Marker6.x = data(1:step :end,18);Marker6.y = data(1:step :end,19);Marker6.z = data(1:step :end,20);
Marker7.x = data(1:step :end,21);Marker7.y = data(1:step :end,22);Marker7.z = data(1:step :end,23);

% endSave
endPosSave_x = [];
endPosSave_y = [];
endPosSave_z = [];

%plot 


for i=1:length(index)
    
    % 绘制刚体 1（支架）
    rigidBody1.y = [Marker1.x(i);Marker2.x(i);Marker3.x(i)];
    rigidBody1.z = [Marker1.y(i);Marker2.y(i);Marker3.y(i)];
    rigidBody1.x = [Marker1.z(i);Marker2.z(i);Marker3.z(i)];
    figure(10);
    plot3(rigidBody1.x,rigidBody1.y,rigidBody1.z,'o','color','b','MarkerSize',10,'MarkerFaceColor','r');
    hold on
    % 绘制刚体2 （手臂运动轨迹）
    rigidBody2.y = [Marker4.x(i);Marker5.x(i);Marker6.x(i);Marker7.x(i)];
    rigidBody2.z = [Marker4.y(i);Marker5.y(i);Marker6.y(i);Marker7.y(i)];
    rigidBody2.x = [Marker4.z(i);Marker5.z(i);Marker6.z(i);Marker7.z(i)];
    plot3(rigidBody2.x,rigidBody2.y,rigidBody2.z,'o','color','b','MarkerSize',10,'MarkerFaceColor','g');
    
    endPosSave_x = [endPosSave_x ;mean(rigidBody2.x(1:3))];
    endPosSave_y = [endPosSave_y ;mean(rigidBody2.y(1:3))];
    endPosSave_z = [endPosSave_z ;mean(rigidBody2.z(1:3))];
    
    plot3(endPosSave_x,endPosSave_y,endPosSave_z,'*','color','r');
%     axis([-1.5 -1 -1.5 -1 1.25 1.7]);
    axis([-1.6 -0.8 -0.8 0 1.2 1.7]);
    view([6,2,2]);
    drawnow();
    hold off
end
% 
% figure(20),
% plot3(endPosSave_x,endPosSave_y,endPosSave_z,'*','color','r');
