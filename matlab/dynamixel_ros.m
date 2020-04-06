%% clc
clear all
clc
%% connect matlab and ros
rosinit
%% rosnode
rosnode list

%% rostopic
rostopic list 
%% rosservice
rosservice list
%% get information about /turtle1/pose
rostopic info /dxl_state_topic

%% see what data is published on the topic
vel = rostopic('echo','/dxl_state_topic')
showdetails(vel)

%% rosmsg show
rosmsg show my_dynamixel_workbench_test/dxl_state

%% create a subscriber for /turtle1/pose topic
scanner = rossubscriber('/dxl_state_topic')
%% dealwith data dxl_state topic
filename='data/10.txt';
[time,radian,velocity,current]=textread(filename,'%*s %f %*s %d %*s %f %*s %f','delimiter',':');
t=1:1:length(radian);
figure(1),
plot(t,radian')

%% 05 .txt Kp=0.03 Kp=0.04 01.txt Kp=0.05
% K = 0.03
filename='data/05.txt';
[time,radian,velocity,current]=textread(filename,'%*s %f %*s %d %*s %f %*s %f','delimiter',':');
t=1:1:length(radian);
a1 = zeros(1,512-435+1);
b1 = 1000*ones(1,600-513+1);
dr = [a1 b1];
y1 = radian(435:600,1);
dt = 1:length(dr);
figure(200),
plot(0.1*dt,dr,'linewidth',1.1,'linestyle','--','color','r')
hold on,
 plot(0.1*dt,y1','linewidth',1.1,'color',[0,0.7,0.9]);
 b2 = 17*ones(1,length(a1));
 y2 = radian(849:end,1);
 y2 = [b2 y2'];
 t=1:length(y2);
 plot(0.1*t,y2,'linewidth',1.1);
 filename='data/01.txt';
[time,radian,velocity,current]=textread(filename,'%*s %f %*s %d %*s %f %*s %f','delimiter',':');
t=1:1:length(radian);
 b3 = 2*ones(1,length(a1));
 y3 = radian(152:250,1);
 y3 = [b3 y3'];
 t = 1:length(y3);
plot(0.1*t,y3,'linewidth',1.1); 
axis([7, 14, 0 1400]) 
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time(s)') ;
ylabel('Displacement ') 
legend('desired','Kp=0.03','Kp=0.04','Kp=0.05')
%% 09.txt p=0.03 d=0.05;p=0.04 d=0.05
filename='data/09.txt';
[time,radian,velocity,current]=textread(filename,'%*s %f %*s %d %*s %f %*s %f','delimiter',':');
t=1:1:length(radian);
 a1 =  -6*ones(1,134-65+1);
 b1 = 1000*ones(1,200-135+1);
dr = [a1 b1]; 
t=1:length(dr);
figure(100)
y1 =radian(65:200,1);
plot(0.1*t,dr,'linewidth',1.1,'linestyle','--','color','r')
hold on,
plot(0.1*t,y1','linewidth',1.1,'color',[0.4940 0.1840 0.5560])
a2 = 21*ones(1,length(a1));
y2 = radian(477:550,1);
y2 =[a2 y2'];
t=1:length(y2);
plot(0.1*t,y2,'linewidth',1.1)
axis([6, 14, 0 1400]) 
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time'); 
ylabel('Displacement ') ;
legend('desired','Kp=0.03,Kd=0.05','Kp=0.04,Kd=0.05')


%% dealwith data dynamixel_statelist topic
% uint: current:2.69[mA]  velocity:0.229[rev/min]  position:1[pulse] 0.088°/Value
clc
clear
filename='data/state07.txt';
[time,radian1,velocity1,current1,radian2,velocity2,current2,radian0,velocity0,current0]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d','delimiter',',');
filename2='data/d_tra07.txt';
[time,dp1,dp2,dp0]=textread(filename2,'%f %*d %*d %*d %d %d %d','delimiter',',');
t=1:1:length(radian1);
t1=1:1:length(dp1);
figure(100),
plot(t,radian1', 'linewidth', 1.1,'color','#0072BD');
hold on
plot(t,radian2', 'linewidth', 1.1,'color','#D95319');
plot(t,radian0', 'linewidth', 1.1,'color','#EDB120');
plot(t1,dp1', 'linewidth', 1.1,'linestyle','--','color','#0072BD');
plot(t1,dp2', 'linewidth', 1.1,'linestyle','--','color','#D95319');
plot(t1,dp0', 'linewidth', 1.1,'linestyle','--','color','#EDB120');
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time'); 
ylabel('Radian ') ;
legend('id=1','id=2','id=0','id=1','id=2','id=0');
figure(200),
plot(t,velocity1', 'linewidth', 1.1);
hold on
plot(t,velocity2', 'linewidth', 1.1);
plot(t,velocity0', 'linewidth', 1.1);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') ;
xlabel('Time'); 
ylabel('Velocity ') ;
legend('id=1','id=2','id=0');
figure(300),
plot(t,current1', 'linewidth', 1.1);
hold on
plot(t,current2', 'linewidth', 1.1);
plot(t,current0', 'linewidth', 1.1);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time'); 
ylabel('Current ') ;
legend('id=1','id=2','id=0');
figure(400),
plot3(dp0,dp1,dp2,'marker','*','color','#0072BD');
hold on,
grid on,
plot3(radian0,radian1,radian2,'marker','*','color','#D95319');
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('id0'); 
ylabel('id1 ') ;
zlabel('id2');
legend('desired\_pos','actual\_pos');
%% desired trajectory
filename2='data/d_tra04.txt';
[time,dp1,dp2,dp0]=textread(filename2,'%f %*d %*d %*d %d %d %d','delimiter',',');
t=1:1:length(time);
figure(400),
plot(t,dp1', 'linewidth', 1.1,'linestyle','--','color','#0072BD');
hold on
plot(t,dp2', 'linewidth', 1.1,'linestyle','--','color','#D95319');
plot(t,dp0', 'linewidth', 1.1,'linestyle','--','color','#EDB120');
%% 3DOF count
figure(10),
plot3(dp0,dp1,dp2,'marker','*','color','#0072BD');
hold on,
grid on,
plot3(radian0,radian1,radian2,'marker','*','color','#D95319');
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('id0'); 
ylabel('id1 ') ;
zlabel('id2');
legend('desired\_pos','actual\_pos');

%% output trajectory yaml file
clc
fid = fopen('motorTra.yaml','w');
len = length(motorTheta);
motorNum = 3;
wayPointNum = len;
thetaData=motorTheta(1:wayPointNum,1:motorNum);
rowIndex = 1;
colIndex = 1;
startTime = 0.2;
dTime = 0.01;
fprintf(fid,'joint:\n');
fprintf(fid,'  names: [zero,first,second]\n');
fprintf(fid,'trajectory:\n');
fprintf(fid,'  index: [');
for i=1:(wayPointNum-1)
    fprintf(fid,'wp%d,',i);
end
fprintf(fid,'wp%d]\n',wayPointNum);
for i=1:wayPointNum
    fprintf(fid,'  wp%d:\n',i);
    fprintf(fid,'    pos: [');
    for j=1:(motorNum-1)
       % rowIndex,colIndex
        fprintf(fid,'%f,',thetaData(rowIndex,colIndex));
        colIndex=colIndex+1;
    end
    %rowIndex,colIndex
    fprintf(fid,'%f]\n',thetaData(rowIndex,colIndex));
    rowIndex=rowIndex+1;
    colIndex = 1;
    startTime = startTime+dTime;
    fprintf(fid,'    time_from_start: %f\n',startTime);
end
fclose(fid);

