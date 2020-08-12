%% 实物实验,不带速度
clc
fid = fopen('RealMotorTra01.yaml','w');
motorTheta=-motorTheta/180*pi;
len = length(motorTheta);
motorNum = 5;
wayPointNum = len;
thetaData=motorTheta(1:wayPointNum,1:end);

rowIndex = 1;
colIndex = 1;
startTime = 0;
dTime = 0.01;
fprintf(fid,'joint:\n');
fprintf(fid,'  names: [second,third,fourth,zero,one]\n');
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

%% 实物实验，带速度
clc

fid = fopen('RealMotorTra02.yaml','w');
motorTheta=motorTheta/180*pi;
len = length(motorTheta);
motorNum = 5;
wayPointNum = len;
thetaData=motorTheta(1:wayPointNum,1:end);

rowIndex = 1;
colIndex = 1;
startTime = 0;
dTime = 0.02;
fprintf(fid,'joint:\n');
fprintf(fid,'  names: [second,third,fourth,zero,one]\n');
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

%% output trajectory yaml file
clc
fid = fopen('MotorTra02.yaml','w');
% t=0:pi/2:12*pi;
% Theta=3.14*sin(t);
% motorTheta=[Theta',Theta',Theta'];
motorTheta=motorTheta/180*pi;
len = length(motorTheta);
motorNum = 2;
wayPointNum = len;
thetaData=motorTheta(1:wayPointNum,4:end);

rowIndex = 1;
colIndex = 1;
startTime = 0;
dTime = 0.01;
fprintf(fid,'joint:\n');
fprintf(fid,'  names: [zero,first]\n');
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
%% include vel
clc
load('motordata.mat')
fid = fopen('CenTra02.yaml','w');
% t=0:pi/2:12*pi;
% Theta=3.14*sin(t);
% motorTheta=[Theta',Theta',Theta'];
motorTheta=motorTheta/180*pi;
len = length(motorTheta);
motorNum = 3;
wayPointNum = len;
thetaData=motorTheta(1:wayPointNum,1:3);

rowIndex = 1;
colIndex = 1;
startTime = 0;
dTime = 0.02;
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
    
%     colIndex=1;
%     fprintf(fid,'    vel: [');
%     for j=1:(motorNum-1)
%        % rowIndex,colIndex
%         fprintf(fid,'%f,',thetaData(rowIndex,colIndex));
%         colIndex=colIndex+1;
%     end
    
    
    rowIndex=rowIndex+1;
    colIndex = 1;
    startTime = startTime+dTime;
    fprintf(fid,'    time_from_start: %f\n',startTime);
end
fclose(fid)

%%
clc
clear
% filename='data/dis_state12.txt';
filename='data/test_state02.txt';
[time,radian3,velocity3,current3,radian2,velocity2,current2,radian4,velocity4,current4]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d','delimiter',',');
% filename2='data/dis_dtra12.txt';
filename2='data/test_dtra02.txt';
[time,dp3,dp2,dp4]=textread(filename2,'%f %*d %*d %*d %d %d %d','delimiter',',');
t=1:1:length(radian3);
t1=1:1:length(dp3);
t=0.1*t;
t1=0.1*t1;
figure(100),
radian3=(radian3-2048)*0.088;
radian2=(radian2-2048)*0.088;
radian4=(radian4-2048)*0.088;
dp3=(dp3-2048)*0.088;
dp2=(dp2-2048)*0.088;
dp4=(dp4-2048)*0.088;
plot(t,radian3', 'linewidth', 1.1,'color','#0072BD');
hold on
plot(t,radian2', 'linewidth', 1.1,'color','#D95319');
plot(t,radian4', 'linewidth', 1.1,'color','#EDB120');
plot(t1,dp3', 'linewidth', 1.1,'linestyle','--','color','#0072BD');
plot(t1,dp2', 'linewidth', 1.1,'linestyle','--','color','#D95319');
plot(t1,dp4', 'linewidth', 1.1,'linestyle','--','color','#EDB120');
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time'); 
ylabel('Radian ') ;
% legend('id=1','id=2','id=0','id=1','id=2','id=0');
legend('id=1','id=2','id=0');
figure(200),
plot(t,velocity3', 'linewidth', 1.1);
hold on
plot(t,velocity2', 'linewidth', 1.1);
plot(t,velocity4', 'linewidth', 1.1);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') ;
xlabel('Time'); 
ylabel('Velocity ') ;
legend('id=1','id=2','id=0');
figure(300),
plot(t,current3', 'linewidth', 1.1);
hold on
plot(t,current2', 'linewidth', 1.1);
plot(t,current4', 'linewidth', 1.1);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time'); 
ylabel('Current ') ;
legend('id=1','id=2','id=0');
% figure(400),
% plot3(dp0,dp1,dp2,'marker','*','color','#0072BD');
% hold on,
% grid on,
% plot3(radian0,radian1,radian2,'marker','*','color','#D95319');
% set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
% xlabel('id0'); 
% ylabel('id1 ') ;
% zlabel('id2');
% legend('desired\_pos','actual\_pos');
%%
clc
clear
filename='data/state14.txt';
[time,radian3,velocity3,current3,radian2,velocity2,current2]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d ','delimiter',',');
filename2='data/d_tra14.txt';
[time,dp3,dp2]=textread(filename2,'%f %*d %*d  %d %d ','delimiter',',');
t=1:1:length(radian3);
t1=1:1:length(dp3);
figure(100),
radian3=(radian3-2048)*0.088;
radian2=(radian2-2048)*0.088;
radian4=(radian4-2048)*0.088;
dp3=(dp3-2048)*0.088;
dp2=(dp2-2048)*0.088;
dp4=(dp4-2048)*0.088;
plot(t,radian3', 'linewidth', 1.1,'color','#0072BD');
hold on
plot(t,radian2', 'linewidth', 1.1,'color','#D95319');
plot(t,radian4', 'linewidth', 1.1,'color','#EDB120');
plot(t1,dp3', 'linewidth', 1.1,'linestyle','--','color','#0072BD');
plot(t1,dp2', 'linewidth', 1.1,'linestyle','--','color','#D95319');
plot(t1,dp4', 'linewidth', 1.1,'linestyle','--','color','#EDB120');
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time'); 
ylabel('Radian ') ;
% legend('id=1','id=2','id=0','id=1','id=2','id=0');
legend('id=1','id=2','id=0');
figure(200),
plot(t,velocity3', 'linewidth', 1.1);
hold on
plot(t,velocity2', 'linewidth', 1.1);
plot(t,velocity4', 'linewidth', 1.1);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') ;
xlabel('Time'); 
ylabel('Velocity ') ;
legend('id=1','id=2','id=0');
figure(300),
plot(t,current3', 'linewidth', 1.1);
hold on
plot(t,current2', 'linewidth', 1.1);
plot(t,current4', 'linewidth', 1.1);
set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
xlabel('Time'); 
ylabel('Current ') ;
legend('id=1','id=2','id=0');
% figure(400),
% plot3(dp0,dp1,dp2,'marker','*','color','#0072BD');
% hold on,
% grid on,
% plot3(radian0,radian1,radian2,'marker','*','color','#D95319');
% set(gca, 'linewidth', 1.1, 'fontsize', 16, 'fontname', 'times') 
% xlabel('id0'); 
% ylabel('id1 ') ;
% zlabel('id2');
% legend('desired\_pos','actual\_pos');

%%
load('012.mat')
 load('034.mat')
len=210;
start=1;
e=8;
radian0=radian0(start:start+len,1);
radian1=radian1(start:start+len,1);
radian2=radian2(start:start+len,1);
dp0=dp0(start:start+len,1);
dp1=dp1(start:start+len,1);
dp2=dp2(start:start+len,1);
radian3=radian3(start+e:start+len+e,1);
radian4=radian4(start+e:start+len+e,1);
dp3=dp3(start+e:start+len+e,1);
dp4=dp4(start+e:start+len+e,1);
t=1:1:length(radian3);
t1=1:1:length(dp3);
figure(100),
plot(t,radian0', 'linewidth', 1.1,'color','#0072BD');
hold on
plot(t,radian1', 'linewidth', 1.1,'color','#D95319');
plot(t,radian2', 'linewidth', 1.1,'color','#EDB120');
plot(t,radian3', 'linewidth', 1.1,'color','g');
plot(t,radian4', 'linewidth', 1.1,'color','black');
plot(t1,dp0', 'linewidth', 1.1,'linestyle','--','color','#0072BD');
plot(t1,dp1', 'linewidth', 1.1,'linestyle','--','color','#D95319');
plot(t1,dp2', 'linewidth', 1.1,'linestyle','--','color','#EDB120');
plot(t1,dp3', 'linewidth', 1.1,'linestyle','--','color','g');
plot(t1,dp4', 'linewidth', 1.1,'linestyle','--','color','black');




