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

%% 05 .txt 绘制K=0.03 K=0.04 01.txt K=0.05
% 绘制K = 0.03；曲线
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
xlabel('时间(s)','fontname','宋体') 
% ylabel('Displacement ') 
ylabel('位移','fontname','宋体')
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
xlabel('时间(s)','fontname','宋体') 
% ylabel('Displacement ') 
ylabel('位移','fontname','宋体')
legend('desired','Kp=0.03,Kd=0.05','Kp=0.04,Kd=0.05')


%% dealwith data dynamixel_statelist topic
% uint: current:2.69[mA]  velocity:0.229[rev/min]  position:1[pulse] 0.088掳/Value
clc
clear
filename='data/11.txt';
[time,radian1,velocity1,current1,radian2,velocity2,current2,radian0,velocity0,current0]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d','delimiter',',');
t=1:1:length(radian1);
figure(100),
plot(t,radian1');
hold on
plot(t,radian2');
plot(t,radian0');
figure(200),
plot(t,velocity1');
hold on
plot(t,velocity2');
plot(t,velocity0');
figure(300),
plot(t,current1');
hold on
plot(t,current2');
plot(t,current0');
%% output trajectory yaml file
clc
fid = fopen('motorTra.yaml','w');
len = length(motorTheta);
motorNum = 3;
wayPointNum = len;
thetaData=motorTheta(1:wayPointNum,1:motorNum);
rowIndex = 1;
colIndex = 1;
startTime = 0;
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
    startTime = startTime+0.2;
    fprintf(fid,'    time_from_start: %f\n',startTime);
end
fclose(fid);
%% distribute control test
goal=[1000,0,1000];
t_length=1000;

err=zeros(3,1);
last_err=zeros(1,3);
err_int=zeros(1,3);
pos=zeros(1,3);
err_com=zeros(1,3);
err_com_int=zeros(1,3);
pos_t=zeros(3,t_length);
err_com_t=zeros(3,t_length);
err_com_int_t=zeros(3,t_length);
err_t=zeros(3,t_length);

kp=0.5;
kd=0;
ki=0.1;
kc=0.1;
kci=0.01;

t=1:1:t_length;

for i=1:1:t_length
   %通讯网络
    for j=1:1:3
        for k=1:1:3
            if k~=j
                err_com(j)=err_com(j)+(pos(k)-pos(j));
            end
        end
        err_com_int(j)=err_com_int(j)+err_com(j);
        err_com_t(j,i)=err_com(j);
        err_com_int_t(j,i)=err_com_int(j);
    end
    
    for j=1:1:3
        pos_t(j,i)=pos(j);
        err_t(j,i)=err(j);
        last_err(j)=err(j);
        err(j)=goal(j)-pos(j);
        err_int(j)=err_int(j)+err(j);
        if j~=2
        pos(j)=kp*err(j)+kd*(err(j)-last_err(j))+ki*err_int(j)+kc*err_com(j)+kci*err_com_int(j);
        else
         pos(j)=kc*err_com(j)+kci*err_com_int(j);
        end
        err_com(j)=0;
    end
end

figure(100),
plot(t,pos_t);
figure(200)
plot(t,err_com_t);
figure(300)
plot(t,err_com_int_t);









