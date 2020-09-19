%% 实物实验一 （模型显示）
clear 
clc
close all
% 变量声明
alpha =0%前后伸展 正
beta =0;%沿颈肩上下伸展 负
gamma =0;% 负
coefficient = 10%1表示单位 1m10 表示单位 dm ;100表示单位 cm ;1000表示单位mm
A0 = [1.832343 0.556926 1.438875]*coefficient;
C0 = [1.8342155;0.5480115;1.373666]*coefficient %手臂末端坐标
L = norm(A0-C0');
V2 = [1.704463 0.52516 1.596931]*coefficient;
F = [1.766782 0.466348  1.537818]*coefficient;
%F V3 V4 V0 V1 在坐标系O的表示
A =[F/coefficient        ;
        1.742190 0.544417  1.538995;
        1.742190 0.546938  1.490152;
       1.891779 0.551402   1.591663;
       1.919727 0.567959  1.526999]*coefficient;
%BV2 BV3 BV4 BV0 BV1 在坐标系O的表示
oB =[1.847434 0.536718 1.392759;
        1.8929344 0.524473  1.385093;
        1.819922 0.562993  1.369919;
        1.861352 0.550960  1.381767;
         1.850785 0.569201 1.366971]*coefficient;
 % B2 B3 B4 B5 在坐标系C的表示
cB = oB - kron(ones(5,1),C0');

TA = transl(A0);
ATC = transl(C0'-A0);


Tspan=30;
% dalpha =-pi/4/Tspan;
dalpha = pi/6/Tspan;
dbeta = pi/6/Tspan;
dgamma = 0;

loop_times = 1
C1_save = [];

for k1=1:1:loop_times*Tspan
alpha = pi/4;
% beta = beta+dbeta;
% gamma = gamma + dgamma;
% beta = pi/6*sin(2*pi*k/Tspan);
% alpha = pi/6*sin(2*pi*k/Tspan)-pi/6;
gamma = pi*sin(2*pi*k1/Tspan)+pi;

% 计算部分
cBT=cB';
AT=A';

T0=[ eye(3,3) ,A0';zeros(1,3),1];
% T1表示中心的位姿 即 AcT
% 这里的 T1存在问题
T1 = TA*trotz(gamma)*troty(beta)*trotx(alpha)*ATC;
C1 = T1(1:3,end);
C1_save = [C1_save C1];

BT = homtrans(T1,cBT);
 
%  计算绳线的长度 A1B1
ML = AT-BT;
    for i=1:1:5
        stringL(k1,i)=norm(ML(:,i));
    end
stringL
norm(A0-C0')
% 零位状态各绳线的长度为
% 1.8027    2.1635    1.4407    2.1209    1.7425    dm


%  画图部分
trplot(T0,'frame','O','color','r');
axis([-3*L+T0(1,end) 3*L+T0(1,end) -3*L+T0(2,end) 3*L+T0(2,end)  -3*L+T0(3,end) 3*L+T0(3,end) ]);
hold on
trplot(T1,'frame','C','color','b');
 plot3(AT(1,:)',AT(2,:)',AT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','r');
 plot3(V2(1),V2(2),V2(3),'o','color','b','MarkerSize',10,'MarkerFaceColor','r');
 plot3(BT(1,:)',BT(2,:)',BT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
%  进行连线 Ai 和Bi连线
ABT =[AT BT];
point_nAB=length(ABT);
LinkAB=zeros(point_nAB,point_nAB);
for i=1:1:point_nAB/2
    LinkAB(i,point_nAB/2+i)=1;
    LinkAB(point_nAB/2+i,i)=1;
end
gplot23D(LinkAB,ABT');
% 绘制Bi之间的连线
LinkB = ones(5,5);
LinkB = LinkB  -eye(5,5);
LinkB(4,1)=0;
LinkB(1,4)=0;
gplot23D(LinkB,BT');
% 连接OC两点
OC =[A0' C1];
plot3(OC(1,:)',OC(2,:)',OC(3,:)','color','k','LineWidth',4);
plot3(C1_save(1,:)',C1_save(2,:)',C1_save(3,:)','*','color','r');
%连接 A1F两点
A1F=[V2' F'];
plot3(A1F(1,:)',A1F(2,:)',A1F(3,:)');
hold off
view([-1,-1.3,0.5]);
drawnow();
hold off
view([-1,-1.3,0.5]);
end 

t=1:1:loop_times*Tspan;
figure(100),
plot(t,stringL,'linewidth',1.1);
% axis([1,50,10,24]);
set(gca,'linewidth',1.1,'fontsize',14,'fontname','宋体')
xlabel('时间/\fontname{Times New Roman}unit');
ylabel('绳长/\fontname{Times New Roman}unit');
legend('FB1', 'A2B2','A3B3','A4B4','A5B5','fontname','Times');
% set (gca,'position',[0.1,0.12,0.7,0.8] );
title('各绳线长度随时间变化图');

motorR = 10/1000*coefficient;
%以初始状态的长度为基准
motorTheta=zeros(loop_times*Tspan,5);
for i=1:1:loop_times*Tspan
    motorTheta(i,:)=(stringL(i,:)-stringL(1,:))/motorR*180/pi;
%     motorTheta(i,:)=(stringL(i,:)-stringL(1,:))/2;
end
% motorTheta
figure(200),
plot(t,motorTheta,'linewidth',1.1);
set(gca,'linewidth',1.1,'fontsize',14,'fontname','宋体');
title('各舵机转角随时间变化图');
xlabel('时间/\fontname{Times New Roman}unit');
ylabel('舵机转角/\fontname{Times New Roman}°');
legend('A1', 'A2','A3','A4','A5','fontname','Times');
% set (gca,'position',[0.1,0.12,0.7,0.8] );
xlswrite('trajactory.xlsx',[motorTheta;stringL]);

%% 实物轨迹配置文件生成，时间自定义
clc
fid = fopen('RealMotorTraOpti02.yaml','w');
motorThetaTemp = motorTheta/180*pi;
motorThetaTemp = [motorThetaTemp(1:10:250,:) motorThetaTemp(251:20:end,:) motorThetaTemp(end,:)];

DataSize = size(motorThetaTemp);
len = DataSize(1);
motorNum = 5;
wayPointNum = len;
thetaData=motorThetaTemp(1:wayPointNum,1:end);

rowIndex = 1;
colIndex = 1;
startTime = 0;
dTime = 0.02;
fprintf(fid,'joint:\n');
fprintf(fid,'  names: [second,third,fourth,zero,first]\n');
% fprintf(fid,'  names: [zero,first,second,third,fourth]\n');
fprintf(fid,'trajectory:\n');
fprintf(fid,'  index: [');
for i=1:(wayPointNum-1)
    fprintf(fid,'wp%d,',i);
end
fprintf(fid,'wp%d]\n',wayPointNum);
for i=1:wayPointNum
    if i > 26
        coeff = 1;
        dTime = 0.1;
    else
        coeff = 1;
        dTime = 0.02;
        
    end
    fprintf(fid,'  wp%d:\n',i);
    fprintf(fid,'    pos: [');
    for j=1:(motorNum-1)
       % rowIndex,colIndex
        fprintf(fid,'%f,',thetaData(rowIndex,colIndex));
        colIndex=colIndex+1;
    end
    %rowIndex,colIndex
    fprintf(fid,'%f]\n',thetaData(rowIndex,colIndex));
    if rowIndex==1||rowIndex==wayPointNum
        fprintf(fid,'    vel: [');
        for j=1:(motorNum-1)
           % rowIndex,colIndex
            fprintf(fid,'0.0,');
        end
        fprintf(fid,'0.0]\n');
    else
        colIndex=1;
        fprintf(fid,'    vel: [');
        for j=1:(motorNum-1)
           
            fprintf(fid,'%f,',(thetaData(rowIndex+1,colIndex)-thetaData(rowIndex-1,colIndex))/(2*dTime*coeff));
            colIndex=colIndex+1;
        end
        fprintf(fid,'%f]\n',(thetaData(rowIndex+1,colIndex)-thetaData(rowIndex-1,colIndex))/(2*dTime*coeff));
    end
    rowIndex=rowIndex+1;
    colIndex = 1;
    startTime = startTime+dTime;
    fprintf(fid,'    time_from_start: %f\n',startTime);
end
fclose(fid)


%% 实物轨迹配置文件生成，带速度
clc
% clear
% load('Tra811.mat')
fid = fopen('RealMotorTraOpti01.yaml','w');
% motorTheta=motorTheta/180*pi;
% motorTheta=motorTheta(1:3,:);
motorThetaTemp = motorTheta/180*pi;
motorThetaTemp = motorThetaTemp(1:10:end,:);
DataSize = size(motorThetaTemp);
len = DataSize(1);
motorNum = 5;
wayPointNum = len;
thetaData=motorThetaTemp(1:wayPointNum,1:end);

rowIndex = 1;
colIndex = 1;
startTime = 0;
dTime = 0.02;
fprintf(fid,'joint:\n');
fprintf(fid,'  names: [second,third,fourth,zero,first]\n');
% fprintf(fid,'  names: [zero,first,second,third,fourth]\n');
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
    if rowIndex==1||rowIndex==wayPointNum
        fprintf(fid,'    vel: [');
        for j=1:(motorNum-1)
           % rowIndex,colIndex
            fprintf(fid,'0.0,');
        end
        fprintf(fid,'0.0]\n');
    else
        colIndex=1;
        fprintf(fid,'    vel: [');
        for j=1:(motorNum-1)
           
            fprintf(fid,'%f,',(thetaData(rowIndex+1,colIndex)-thetaData(rowIndex-1,colIndex))/(2*dTime));
            colIndex=colIndex+1;
        end
        fprintf(fid,'%f]\n',(thetaData(rowIndex+1,colIndex)-thetaData(rowIndex-1,colIndex))/(2*dTime));
    end
    rowIndex=rowIndex+1;
    colIndex = 1;
    startTime = startTime+dTime;
    fprintf(fid,'    time_from_start: %f\n',startTime);
end
fclose(fid)

%% 实物轨迹配置文件生成,无速度
clc
% clear
% load('Tra811.mat')
fid = fopen('RealMotorTra04.yaml','w');
% motorTheta=motorTheta/180*pi;
% motorTheta=motorTheta(1:3,:);
motorThetaTemp = motorTheta;
motorThetaTemp = motorThetaTemp(1:5:end,:);
DataSize = size(motorThetaTemp);
len = DataSize(1);
motorNum = 5;
wayPointNum = len;
thetaData=motorThetaTemp(1:wayPointNum,1:end);

rowIndex = 1;
colIndex = 1;
startTime = 0;
dTime = 0.02;
fprintf(fid,'joint:\n');
fprintf(fid,'  names: [second,third,fourth,zero,first]\n');
% fprintf(fid,'  names: [zero,first,second,third,fourth]\n');
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
fclose(fid)
%% 数据处理
% motorTheta = [radian0 radian1 radian2 radian3 radian4]
clc
clear
close all
filename='data/stateRecord17.txt';
% 时间戳记录的是nsec ,时间的单位为纳秒，10-9
[time,radian1,velocity1,current1,radian4,velocity4,current4,radian2,velocity2,current2,radian3,velocity3,current3,radian0,velocity0,current0]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d','delimiter',',');
time1 = (time-time(1))/10^9;
filename2='data/d_traRecord17.txt';
[time,dp1,dp4,dp2,dp3,dp0]=textread(filename2,'%f %*d %*d %*d %*d %*d %d %d %d %d %d','delimiter',',');
time2 = (time-time(1))/10^9;


% 角度
figure(100),
radian0=(radian0-radian0(1))*0.088;
radian1=(radian1-radian1(1))*0.088;
radian2=(radian2-radian2(1))*0.088;
radian3=(radian3-radian3(1))*0.088;
radian4=(radian4-radian4(1))*0.088;

hold on
plot(time1,radian0', 'linewidth', 1.1,'color','#EDB120');
plot(time1,radian1', 'linewidth', 1.1,'color','#0072BD');
plot(time1,radian2', 'linewidth', 1.1,'color','#D95319');
plot(time1,radian3', 'linewidth', 1.1,'color','#7E2F8E');
plot(time1,radian4', 'linewidth', 1.1,'color','#77AC30');

dp1=(dp1)*0.088;
dp4=(dp4)*0.088;
dp2=(dp2)*0.088;
dp3=(dp3)*0.088;
dp0=(dp0)*0.088;

plot(time2,dp0', 'linewidth', 1.1,'linestyle','--','color','#EDB120');
plot(time2,dp1', 'linewidth', 1.1,'linestyle','--','color','#0072BD');
plot(time2,dp2', 'linewidth', 1.1,'linestyle','--','color','#D95319');
plot(time2,dp3', 'linewidth', 1.1,'linestyle','--','color','#7E2F8E');
plot(time2,dp4', 'linewidth', 1.1,'linestyle','--','color','#77AC30');


set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time')
ylabel('Radian ') ;


figure(200),
hold on
plot(time1,velocity0', 'linewidth', 1.1,'color','#EDB120');
plot(time1,velocity1', 'linewidth', 1.1,'color','#0072BD');
plot(time1,velocity2', 'linewidth', 1.1,'color','#D95319');
plot(time1,velocity3', 'linewidth', 1.1,'color','#7E2F8E');
plot(time1,velocity4', 'linewidth', 1.1,'color','#77AC30');
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') ;
xlabel('Time')
ylabel('Velocity ') ;


% 速度积分验证，结论经过验证，根据速度积分求解的位移需要乘以一个系数，因此可以用来指导轨迹文件生成
% 速度x时间 累加
radian0_ = [];radian1_ = [];radian2_ = [];
time_ = [];
temp0 = 0;temp1 = 0;temp2 = 0;
coeff = 1.45;
for index=1:length(velocity0)-1
    dt = time1(index+1)-time1(index);
    t = (time1(index+1)+time1(index))/2;
%     temp0 = temp0+v_ave0*dt;

    v_ave0 = (velocity0(index)+velocity0(index+1))/2;
    temp0 = temp0+coeff*dt*v_ave0;
    radian0_ = [radian0_ temp0];
    
    v_ave1 = (velocity1(index)+velocity1(index+1))/2;
    temp1 = temp1+coeff*dt*v_ave1;
    radian1_ = [radian1_ temp1];
    
    v_ave2 = (velocity2(index)+velocity2(index+1))/2;
    temp2 = temp2+coeff*dt*v_ave2;
    radian2_ = [radian2_ temp2];
    
    time_ =[time_;t];
end
figure(201)
hold on
plot(time1,radian0', 'linewidth', 1.1,'color','#EDB120');
plot(time_,radian0_', 'linewidth', 1.1,'LineStyle','--','color','#EDB120');
plot(time1,radian1', 'linewidth', 1.1,'color','#0072BD');
plot(time_,radian1_', 'linewidth', 1.1,'LineStyle','--','color','#0072BD');
plot(time1,radian2', 'linewidth', 1.1,'color','#D95319');
plot(time_,radian2_', 'linewidth', 1.1,'LineStyle','--','color','#D95319');



figure(300),
hold on
plot(time1,current0', 'linewidth', 1.1,'color','#EDB120');
plot(time1,current1', 'linewidth', 1.1,'color','#0072BD');
plot(time1,current2', 'linewidth', 1.1,'color','#D95319');
plot(time1,current3', 'linewidth', 1.1,'color','#7E2F8E');
plot(time1,current4', 'linewidth', 1.1,'color','#77AC30');
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time')
ylabel('Current ') ;

 %% 示教画图
 clc
clear
close all
% filename='data/stateRecord01.txt';
filename='data/state05.txt';
[time,radian1,velocity1,current1,radian4,velocity4,current4,radian2,velocity2,current2,radian3,velocity3,current3,radian0,velocity0,current0]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d','delimiter',',');

 t=1:1:length(radian1);
t=0.1*t  
     
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
plot(t,radian4', 'linewidth', 1.1,'color','#77AC30')  

% fprintf(fid,'  names: [second,third,fourth,zero,first]\n');
motorTheta = [radian2 radian3 radian4 radian0 radian1]

%% 示教轨迹优化
% 优化的思路：去除一些平的点，然后最后的点回归到0
 clc
clear
close all
% filename='data/stateRecord01.txt';
filename='data/state03.txt';
[time,radian1,velocity1,current1,radian4,velocity4,current4,radian2,velocity2,current2,radian3,velocity3,current3,radian0,velocity0,current0]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d','delimiter',',');

time_ = (time-time(1))/10^9;

     
 % 角度
figure(100),
radian0=(radian0-radian0(1))*0.088;
radian1=(radian1-radian1(1))*0.088;
radian2=(radian2-radian2(1))*0.088;
radian3=(radian3-radian3(1))*0.088;
radian4=(radian4-radian4(1))*0.088;

hold on
plot(time_,radian0', 'linewidth', 1.1,'color','#EDB120');
plot(time_,radian1', 'linewidth', 1.1,'color','#0072BD');
plot(time_,radian2', 'linewidth', 1.1,'color','#D95319');
plot(time_,radian3', 'linewidth', 1.1,'color','#7E2F8E');
plot(time_,radian4', 'linewidth', 1.1,'color','#77AC30')  

% fprintf(fid,'  names: [second,third,fourth,zero,first]\n');


gate = 0.5;

radian0_new = [0];
radian1_new = [0];
radian2_new = [0];
radian3_new = [0];
radian4_new = [0];
% t_new = [0];
index_save = [];
for i = 2:1:length(radian0)
    if abs(radian4(i-1) - radian4(i))>gate||abs(radian2(i-1) - radian2(i))>gate||abs(radian1(i-1) - radian1(i))>gate||abs(radian0(i-1) - radian0(i))>gate||abs(radian3(i-1) - radian3(i))>gate
        radian4_new = [radian4_new  radian4(i-1)];
         index_save = [index_save i-1];
    end
end
for i = 1:length(index_save)
    radian1_new = [radian1_new radian1(index_save(i))];
    radian2_new = [radian2_new radian2(index_save(i))];
    radian0_new = [radian0_new radian0(index_save(i))];
    radian3_new = [radian3_new radian3(index_save(i))];
%     radian4_new = [radian4_new radian4(index_save(i))];
%     t_new = [t_new t_new(end)+(time_(index_save(i+1))-time_(index_save(i)))];
end
figure(200)
hold on
t_new = 1:length(radian0_new);
plot(t_new,radian0_new', 'linewidth', 1.1,'color','#EDB120');
plot(t_new,radian1_new', 'linewidth', 1.1,'color','#0072BD');
plot(t_new,radian2_new', 'linewidth', 1.1,'color','#D95319');
plot(t_new,radian3_new', 'linewidth', 1.1,'color','#7E2F8E');
plot(t_new,radian4_new', 'linewidth', 1.1,'color','#77AC30') 



% plot( 1:length(radian0_new),radian0_new', 'linewidth', 1.1,'color','#EDB120');
% plot( 1:length(radian1_new),radian1_new', 'linewidth', 1.1,'color','#0072BD');
% plot( 1:length(radian2_new),radian2_new', 'linewidth', 1.1,'color','#D95319');
% plot( 1:length(radian3_new),radian3_new', 'linewidth', 1.1,'color','#7E2F8E');
% plot( 1:length(radian4_new),radian4_new', 'linewidth', 1.1,'color','#77AC30') 

%% 轨迹拟合 5次曲线拟合

% radian0_new(1)=0;
% radian1_new(1)=0;
% radian2_new(1)=0;
% radian3_new(1)=0;
% radian4_new(1)=0;
% radian0_new(end)=0;
% radian1_new(end)=0;
% radian2_new(end)=0;
% radian3_new(end)=0;
% radian4_new(end)=0;
t_new_ = [0];
index = 200;
for i =  1:length(radian1_new)-2
    if i <=index
        t_new_ =[t_new_ t_new_(end)+0.02];
    else
         t_new_ =[t_new_ t_new_(end)+0.04];
    end
end
t_new_ = [t_new_ t_new_(end)+0.1]


k1 = polyfit(t_new_,radian1_new,5);
k2 = polyfit(t_new_,radian2_new,5);
k3 = polyfit(t_new_,radian3_new,5);
k4 = polyfit(t_new_,radian4_new,5);
k0 = polyfit(t_new_,radian0_new,5);

y1 = k1(1)*t_new_.^5+k1(2)*t_new_.^4+k1(3)*t_new_.^3+k1(4)*t_new_.^2+k1(5)*t_new_+k1(6);
y2 = k2(1)*t_new_.^5+k2(2)*t_new_.^4+k2(3)*t_new_.^3+k2(4)*t_new_.^2+k2(5)*t_new_+k2(6);
y3 = k3(1)*t_new_.^5+k3(2)*t_new_.^4+k3(3)*t_new_.^3+k3(4)*t_new_.^2+k3(5)*t_new_+k3(6);
y4 = k4(1)*t_new_.^5+k4(2)*t_new_.^4+k4(3)*t_new_.^3+k4(4)*t_new_.^2+k4(5)*t_new_+k4(6);
y0 = k0(1)*t_new_.^5+k0(2)*t_new_.^4+k0(3)*t_new_.^3+k0(4)*t_new_.^2+k0(5)*t_new_+k0(6);

y0(1)=0;y0(end)=0;
y1(1)=0;y1(end)=0;
y2(1)=0;y2(end)=0;
y3(1)=0;y3(end)=0;
y4(1)=0;y4(end)=0;


figure(300)
plot(t_new_,y0, 'linewidth', 1.1,'color','#EDB120');
hold on
plot(t_new_,y1, 'linewidth', 1.1,'color','#0072BD');
plot(t_new_,y2, 'linewidth', 1.1,'color','#D95319');
plot(t_new_,y3, 'linewidth', 1.1,'color','#7E2F8E');
plot(t_new_,y4, 'linewidth', 1.1,'color','#77AC30') 

motorTheta = [y2' y3' y4' y0' y1'];

% % 对轨迹进行求导，得到速度大小
% Vy1 = 5*k1(1)*t_new_.^4+4*k1(2)*t_new_.^3+3*k1(3)*t_new_.^2+k1(4)*t_new_+k1(5);
% Vy2 = 5*k2(1)*t_new_.^4+4*k2(2)*t_new_.^3+3*k2(3)*t_new_.^2+k2(4)*t_new_+k2(5);
% Vy3 = 5*k3(1)*t_new_.^4+4*k3(2)*t_new_.^3+3*k3(3)*t_new_.^2+k3(4)*t_new_+k3(5);
% Vy4 = 5*k4(1)*t_new_.^4+4*k4(2)*t_new_.^3+3*k4(3)*t_new_.^2+k4(4)*t_new_+k4(5);
% Vy0 = 5*k0(1)*t_new_.^4+4*k0(2)*t_new_.^3+3*k0(3)*t_new_.^2+k0(4)*t_new_+k0(5);
% 
% figure(400)
% plot(t_new_,Vy0, 'linewidth', 1.1,'color','#EDB120');
% hold on
% plot(t_new_,Vy1, 'linewidth', 1.1,'color','#0072BD');
% plot(t_new_,Vy2, 'linewidth', 1.1,'color','#D95319');
% plot(t_new_,Vy3, 'linewidth', 1.1,'color','#7E2F8E');
% plot(t_new_,Vy4, 'linewidth', 1.1,'color','#77AC30') 
% 
% motorThetaVel = [Vy2' Vy3' Vy4' Vy0' Vy1'];
%% 曲线拟合 4次 曲线
t_new = t_new;
k1 = polyfit(t_new,radian1_new,4);
k2 = polyfit(t_new,radian2_new,4);
k3 = polyfit(t_new,radian3_new,4);
k4 = polyfit(t_new,radian4_new,4);
k0 = polyfit(t_new,radian0_new,4);

y1 = k1(1)*t_new.^4+k1(2)*t_new.^3+k1(3)*t_new.^2+k1(4)*t_new+k1(5);
y2 = k2(1)*t_new.^4+k2(2)*t_new.^3+k2(3)*t_new.^2+k2(4)*t_new+k2(5);
y3 = k3(1)*t_new.^4+k3(2)*t_new.^3+k3(3)*t_new.^2+k3(4)*t_new+k3(5);
y4 = k4(1)*t_new.^4+k4(2)*t_new.^3+k4(3)*t_new.^2+k4(4)*t_new+k4(5);
y0 = k0(1)*t_new.^4+k0(2)*t_new.^3+k0(3)*t_new.^2+k0(4)*t_new+k0(5);
figure(300)
plot(t_new,y0, 'linewidth', 1.1,'color','#EDB120');
hold on
plot(t_new,y1, 'linewidth', 1.1,'color','#0072BD');
plot(t_new,y2, 'linewidth', 1.1,'color','#D95319');
plot(t_new,y3, 'linewidth', 1.1,'color','#7E2F8E');
plot(t_new,y4, 'linewidth', 1.1,'color','#77AC30') 

motorTheta = [y2' y3' y4' y0' y1'];

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

%% 舵机导纳控制模拟
clc
clear
% 定义系统的动力学方程
J = 0.1;
b = 0.2;
A=[0 1 ;
        0 -b/J];
B = [0 1/J];

Ts = 0.1;
Tspan = 100;
% 变成离散系统方便求解
[Ad,Bd]=c2d(A,B,Ts);

K1 = 0.1;
Kp = 0.1;
Kd = 0;
u = 0;
desired_x = [1;0];
f_ext = -1;
xd_save = [];
x_save = [];
u_save = [];
xd = desired_x(1);
err = 0;
last_err = 0;
x = [0;0];

for i = 1:Tspan/Ts 
    xd_save = [xd_save;xd];
    x_save = [x_save x];
    u_save = [u_save u];
    xd = desired_x(1) + u * K1;
    err = xd -x(1);
    u = Kp * (xd - x(1)) + Kd * (err - last_err)
    x_ = Ad*x + Bd*u;
    last_err = err;
    x = x_;
end
t = 1:length(x_save);
figure(1),
plot(t,x_save);
figure(2),
plot(t,u_save);
    
%% 舵机力辨识——数据读取
clc
clear
close all

% 通过sprintf 实现字符串的拼接
% 上升阶段
mass = [500 200 100];
goal_vel = -[10 20 40 60 80 100 120];
count = 1;
for i=1:length(mass)
    for j=1:7
        clear time radian velocity current
        filename = sprintf('data/0906/g%d_0%d.txt',mass(i),j);
        [time,radian,velocity,current]...
        =textread(filename,'%f %*s %*d %d %d %d' ,'delimiter',',');
        start_index = 10;
        time1 = time(start_index:end);
        radian1 = radian(start_index:end);
        velocity1 = velocity(start_index:end);
        current1 = current(start_index:end);
        data_info(count).m = mass(i);
        data_info(count).v = goal_vel(j);
        data_info(count).time = time1;
        data_info(count).radian = radian1;
        data_info(count).velocity = velocity1;
        data_info(count).current = current1;
        data_info(count).current_ave = mean(current1);
        data_info(count).current_median = median(current1);
        data_info(count)
        count = count +1;
    end 
end

mass_ = [200];
for i = 1:1
        for j=1:7
        clear time radian velocity current
        filename = sprintf('data/0906/dg%d_0%d.txt',mass_(i),j);
        [time,radian,velocity,current]...
        =textread(filename,'%f %*s %*d %d %d %d' ,'delimiter',',');
        start_index = 10;
        time1 = time(start_index:end);
        radian1 = radian(start_index:end);
        velocity1 = velocity(start_index:end);
        current1 = current(start_index:end);
        data_info(count).m = mass_(i);
        data_info(count).v = -goal_vel(j);
        data_info(count).time = time1;
        data_info(count).radian = radian1;
        data_info(count).velocity = velocity1;
        data_info(count).current = current1;
        data_info(count).current_ave = mean(current1);
        data_info(count).current_median = median(current1);
        data_info(count)
        count = count +1;
    end 
end



% save data_info

%% 求解K的大小

delta_m = [];
delta_i = [];
% 涉及到一个排列组合
queue ={[0 1] [0 2] [1 2]};
for j =1:3
    for i = 1:7
        temp = data_info(queue{j}(1)*7+i).m-data_info(queue{j}(2)*7+i).m;
        delta_m = [delta_m temp];
        temp = data_info(queue{j}(1)*7+i).current_median-data_info(queue{j}(2)*7+i).current_median;
        delta_i = [delta_i temp];
    end
end
K_ = pinv(delta_i')*delta_m';
K = delta_m*10^-3*9.8/-delta_i  %计算结果： K=0.3887

% 100 -3.5  300 -7  400 -10.5
% 通过四次曲线拟合 K 与 I 的关系
% K = 0.3769
m_ = [0 100 300 400]*10^-3*9.8;
i_ = -[0 -4 -7 -11];
p=polyfit(i_,m_,1);
p(1)

%%  按照论文得到的摩擦力模型
% 速度 unit 0.23 rev/min
% theta(1) =     1.8672 N;
clc
mean_err_save =[];
theta_save =[];

K =  0.3769; % N/unit
unit_V = 0.23;  %rev/min
g = 9.8;


w1 = 1;
w2 = 0;
w3 = 0;

hat_f = K*data_info(1).current+data_info(1).m*10^-3*g;    %仅适合与近似匀速,匀速运动时，驱动力-重力=摩擦力（牵引）；下放的情形：重力-驱动力=摩擦力


W = [w1*sign(data_info(1).current)+w3*sign(data_info(1).velocity) w2*sign(data_info(1).current) w3*(data_info(1).velocity).^2 w3*(data_info(1).velocity)];  %速度的单位变成 rad/s

theta = pinv(W)*hat_f;
% theta_save=[theta_save theta];

tau_c = theta(1);
tau_s = theta(2);
beta_1 = theta(3);
beta_2 = theta(4);


for i = 1:length(data_info)
% data_info(1).current,data_info(1).velocity
% 检查，代入另一组数据，看是否动平衡
hat_f_ = K*data_info(i).current+data_info(i).m*10^-3*g;
W = [w1*sign(data_info(i).current)+w3*sign(data_info(i).velocity) w2*sign(data_info(i).current) w3*(data_info(i).velocity*unit_V*2*pi/60).^2 w3*(data_info(i).velocity*unit_V*2*pi/60)];
f = W*theta;
err=hat_f_ - f;
mean(err);
mean_err_save=[mean_err_save mean(err)];
end
% min(abs(mean_err_save))
mean_err_save
theta

%% 摩擦力模型辨识
m_ =[];
current_ =[];
dq =[];
% K = 0.3769; % N/unit
g =9.8;
for i=1:length(data_info)
    m_ = [m_ ; data_info(i).m];
    current_ =[current_ ; data_info(i).current_median];
     dq  = [dq ; data_info(i).v];
end

hat_f = -K*current_ - m_*10^-3*g;   %得到摩擦力为正

W_ = [-sign(dq) -dq];

theta_ = pinv(W_)*hat_f
% theta_ =[1.8090 0.0363]';
dq_ = -20:0.1:20;
tau_f = theta_(1)*sign(dq_)+theta_(2)*dq_;
plot(dq_,tau_f)
axis([-20 20 -6 6])

%% 辨识外力
clc
% clear
close all



filename='data/stateRecord17.txt';
% % 时间戳记录的是nsec ,时间的单位为纳秒，10-9
[time,radian1,velocity1,current1,radian4,velocity4,current4,radian2,velocity2,current2,radian3,velocity3,current3,radian0,velocity0,current0]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d','delimiter',',');

% filename='data/0906/dg200_02.txt';
% [time,radian0,velocity0,current0]...
% =textread(filename,'%f %*s %*d %d %d %d' ,'delimiter',',');

time1 = (time-time(1))/10^9;

% 角度
figure(100),
radian0=(radian0-radian0(1))*0.088;

hold on
plot(time1,radian0', 'linewidth', 1.1,'color',[0 0.4470 0.7410]);


figure(200),
hold on
plot(time1,velocity0', 'linewidth', 1.1,'color',[0 0.4470 0.7410]);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') ;
xlabel('Time')
ylabel('Velocity ') ;


figure(300),
hold on
plot(time1,current0', 'linewidth', 1.1,'color',[0 0.4470 0.7410]);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time')
ylabel('Current ') ;


% 质量忽略不计，也就是不考虑加速度的影响,对于静止情况下的外力还不能估计，速度为0，外力=驱动力-摩擦力

 theta_ =[1.3660 0.0284]';
 K = 0.3887;
dq = velocity0;
current =current0;
W = [-sign(dq) -dq];
tau_f= W*theta_;
tau_e = -K*current-tau_f;
tau_logic = tau_e>0;
tau_e = tau_e.*tau_logic;

figure(400),
hold on
plot(time1,tau_e', 'linewidth', 1.1,'color',[0 0.4470 0.7410]);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time')
ylabel('\tau_e ') ;

figure(500)
hold on
plot(time1,tau_f', 'linewidth', 1.1,'color',[0 0.4470 0.7410]);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time')
ylabel('\tau_f ') ;




