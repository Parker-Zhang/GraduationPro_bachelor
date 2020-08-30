//////////////////////////    headfiles    //////////////////////////
//math
#include <math.h>
#include <Eigen/Eigen>
#include <eigen3/unsupported/Eigen/KroneckerProduct>

#include <cstdlib>

using namespace Eigen;
//ros
#include<dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include<ros/ros.h>
#include<my_dynamixel_workbench_test/dxl_state.h>
//iostream
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
//predefine
#define BAUDRATE 115200
#define ID 0
#define random(x) (rand()%x)
#define PI 3.1415926



//////////////////////////    begin    //////////////////////////
int main(int argc,char **argv)
{
    //////////////////////////	  parameters    //////////////////////////
    int eps_max  = 10;	int step_max = 104;		int step = 0;
    int update_step =  2;
    double gam = 0.85;
    double Q1 = 1000;		double Q2 = 10;
    Eigen::MatrixXd Qx(4, 4);   Qx = MatrixXd::Zero(4, 4);
    Qx(0,0) = Q1;   Qx(1,1) = Q2;   Qx(0,2) = -Q1; Qx(2,0) = -Q1;
    Qx(2,2) = Q1;   Qx(3,3) = Q2;   Qx(1,3) = -Q2; Qx(3,1) = -Q2;
    double Qu = 1;

    int  init_position = 2500;      double  init_pos = init_position*0.088/180*PI;
    int  tra_position  = 2500;      double  tra_pos  = tra_position*0.088/180*PI;
    int  tra_velocity  = 30;      double  tra_vel  = tra_velocity*0.229*2*PI/60;
    double co_pos      = 0.95;      double co_vel    = 0.90;
    double tra_vel_temp = tra_vel;  double tra_pos_temp = tra_pos;
    int tra_pos_limit_int = 400;
    double tra_pos_limit = tra_pos_limit_int*0.088/180*PI;
    double frequency = 50.0;

    int  noise_trial = 30;
    bool noise_change = true;
    bool K_limit = false;
    bool is_current_control = false;
    bool position_limit_flag = false;
    bool AC_control = true;
    bool index_jsum_change = false;
    double fixed_jsum = 0.012;
    bool test = false;

    int P_scale = 1000;
    int ini_pos = 1000;
    int dt = 300*1000;
    float K1 = 0;	float K2 = 0;          float K3 = 0;       float K4 = 0;
     
    int32_t 	limit_current = 450;	//2.69 [mA]

    std::ofstream CoeffRecoder("Z_CoeffRecoder.txt");
    CoeffRecoder <<"Qx     :" <<  Qx                 <<endl
                 <<"K      :" <<  K1 <<";  "<<K2 <<";  "<<K3    <<endl
                 <<"Qu	   :" <<  Qu                 <<endl
                 <<"gamma  :" << gam                 <<endl
                 <<"P      :" << P_scale             <<endl
                 <<"ini_pos:" << ini_pos             <<endl
                 <<"dt     :" << dt/1000             <<endl
                 <<"limit_current:"<< limit_current  <<endl
                 <<"nose   :" << noise_trial         <<"   "<< noise_change <<endl
                 <<"eps_max:" << eps_max             <<endl
                 <<"step_max:"<< step_max            <<endl
                 <<"update_step:"<< update_step                             <<endl
                 <<"is_current_control:"<<is_current_control                <<endl
                 <<"AC_control:" << AC_control       <<endl
                 <<"index_jsum_change:"<< index_jsum_change                 <<endl
                 <<"fixed_jsum: "<<fixed_jsum       <<endl;

    //////////////////////////	  on-offs    //////////////////////////
    bool check = false;
    if (test)
    {
        eps_max = 1;
        K1 = -63;	 K2 = -8;           K3 = 62;        K4 = 122;
    }


    //////////////////////////	  state variable    //////////////////////////
    Eigen::MatrixXd state(4,1) ;	Eigen::MatrixXd statePre(4,1) ;
    double f;       double u;		Eigen::MatrixXd uMat(1, 1);
    Eigen::MatrixXd K(1, 4);		Eigen::MatrixXd Kac(1, 4);
    K(0,0)=K1;      K(0,1)=K2;          K(0,2)=K3;      K(0,3)=K4;      Kac=K;
    Eigen::MatrixXd P(15, 15);		P = MatrixXd::Identity(15, 15)*P_scale;
    Eigen::MatrixXd theta(15, 1);       theta = MatrixXd::Zero(15,1);
    Eigen::MatrixXd gradient(15, 1);    gradient = MatrixXd::Zero(15, 1);
    double cost;			Eigen::MatrixXd costMat(1, 1);
    Eigen::MatrixXd xu(5,1);		Eigen::MatrixXd xuPre(5,1) ;
    Eigen::MatrixXd phi(15, 1);
    Eigen::MatrixXd H(5,5);
    Eigen::MatrixXd H21(1,4);		double H22;
    double index_Jum;			Eigen::MatrixXd Jsum(1, 4);
    Eigen::MatrixXd eye(1,1);		eye(0,0)  = 1;
    Eigen::MatrixXd temp(1,1);		temp(0,0) = 1;

    ros::init(argc,argv,"ACimpedent");
    ros::NodeHandle n;
    ros::Publisher  dxl_state_pub = n.advertise<my_dynamixel_workbench_test::dxl_state>("dxl_state_topic",100);  //定义舵机状态发布器
    my_dynamixel_workbench_test::dxl_state msg;

    //////////////////////////    舵机数据定义
    int32_t 	position_data = 0;    double 	position = 0.0;
    int32_t 	velocity_data = 0;    double 	velocity = 0.0;
    int32_t	current_data  = 0;    double 	current = 0.0;
    int32_t	goal_current;

    //////////////////////////    连接舵机，设置电流控制模式
    DynamixelWorkbench dxl_wb;
    dxl_wb.begin("/dev/ttyUSB0",BAUDRATE);
    dxl_wb.ping(ID);
    dxl_wb.ledOff(ID);
    ROS_INFO("Welcome my dynamixel workbench!");
    dxl_wb.setCurrentControlMode(ID);
    //   dxl_wb.itemRead(ID,"Current_Limit",&limit_current);
    //   dxl_wb.itemRead(ID,"Velocity_Limit",&limit_current);
    dxl_wb.torqueOn(ID);

    //////////////////////////    舵机初始化
    dxl_wb.itemRead(ID,"Present_Position",&position_data);
    // position = dxl_wb.convertValue2Position(ID,position_data);
    position = position_data*0.088/180*PI;
    dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
    // velocity = dxl_wb.convertValue2Velocity(ID,velocity_data);
    velocity = velocity_data*0.229*2*PI/60;
    dxl_wb.itemRead(ID,"Present_Current",&current_data);
    // current  = dxl_wb.convertValue2Current(ID,current_data);
    current	 = current_data*2.69;   // mA

    //////////////////////////	定义信息流
    // 在home下创建文件夹 ACimpedent
    std::ofstream StatesRecoder("Z_statusRecoder.txt");
    std::ofstream KRecoder("Z_kRecoder.txt");
    std::ofstream JsumRecoder("Z_JsumRecoder.txt");

    //////////////////////////	学习过程     //////////////////////////
    for (int eps = 0; eps < eps_max; eps++)
    {		// episode
        ROS_INFO("******      episode begin      ******");
        //////////////////////////    找到起始位置
        ROS_INFO("******      go to init pos      ******");
        dxl_wb.torqueOff(ID);
        dxl_wb.setPositionControlMode(ID);
        dxl_wb.torqueOn(ID);
        dxl_wb.itemWrite(ID,"Goal_Position",init_position+tra_pos_limit_int*2);
        sleep(2);
        dxl_wb.itemRead(ID,"Present_Position",&position_data);
        usleep(2000 * 1000);
        ROS_INFO("******      reach init pos      ******");

        //////////////////////////	   学习开始
        //修改控制模型并获得当前状态
        dxl_wb.torqueOff(ID);
        usleep(10);
        //  dxl_wb.setVelocityControlMode(ID);
        if (is_current_control)
        {            dxl_wb.setCurrentControlMode(ID);        }
        else
        {            dxl_wb.setVelocityControlMode(ID);       }
        usleep(10);
        dxl_wb.torqueOn(ID);

        dxl_wb.itemRead(ID,"Present_Position",&position_data);
        position = position_data*0.088/180*PI;
        dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
        velocity = velocity_data*0.229*2*PI/60;
        dxl_wb.itemRead(ID,"Present_Current" ,&current_data);
        current	 = current_data*2.69;   // mA

        state(0,0) = position;          state(1,0) = velocity;
        state(2,0) = tra_pos+tra_pos_limit*2;      state(3,0) = 0;
        tra_pos_temp = tra_pos+tra_pos_limit*2;    tra_vel_temp = 0;


        StatesRecoder << position << ";   " << velocity << ";   " <<tra_pos_temp<<";   "<<tra_vel_temp<<";    "
                << int16_t(current_data) << ";   " <<0<<";   "<<std::endl;
        KRecoder      << K(0,0) 	<<";   " << K(0,1)        << ";   "
                      << K(0,2) 	<<";   " << K(0,3)        << ";   " << std::endl;
        JsumRecoder   << 0              <<";   " <<	0         << ";   "
                      << 0              <<";   " <<	0         << ";   " << std::endl;


        ROS_INFO("******      start learning   episode %d   ******",eps);
        step = 0;			check = true;
        P = MatrixXd::Identity(15, 15)*P_scale;
        while (step < (step_max - 5) && ros::ok() && check )
        {	//   step
            step++;
            statePre = state;
            //////////////////////////    计算输出并发送给舵机
            //  statePre = state;
            uMat = K*statePre;
            // uMat(0,0) = state(3,0)/0.229/2/PI*60;

            if(uMat(0,0)>(limit_current-noise_trial))
            {                uMat(0,0)= (limit_current-noise_trial);            }
            if(uMat(0,0)<-(limit_current-noise_trial))
            {                uMat(0,0)= -(limit_current-noise_trial);            }
            // uMat(0,0)=0;
            if(noise_change)
            {                u        = uMat(0,0) +noise_trial*(eps_max-eps+5)/eps_max*float(rand()%1000-500)/500.0;            }
            else
            {                u        = uMat(0,0) +noise_trial*float(rand()%1000-500)/500.0;            }
            //  u =round(u);
            if(test)
            {
                u = uMat(0,0);
            }

            if(is_current_control)
            {
                goal_current = int32_t(u);
                ROS_INFO("Goal_Current:%d",goal_current);
                dxl_wb.itemWrite(ID,"Goal_Current",goal_current);
            }
            else
            {
                goal_current = int32_t(u);
                ROS_INFO("Goal_Velocityt:%d",goal_current);
                dxl_wb.itemWrite(ID,"Goal_Velocity",goal_current);
            }
            // dxl_wb.itemWrite(ID,"Goal_Velocity",goal_current);
            usleep(dt);

            //////////////////////////	  更新状态
            dxl_wb.itemRead(ID,"Present_Position",&position_data);
            position = position_data*0.088/180*PI;
            dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
            velocity = velocity_data*0.229*2*PI/60;
            dxl_wb.itemRead(ID,"Present_Current" ,&current_data);
            current	 = current_data*2.69;   // mA
            cout<< "current_data:" << int16_t(current_data)<<endl;
            // tra_pos_temp = tra_pos_temp*co_pos;       tra_vel_temp = tra_vel_temp*co_vel;
            tra_pos_temp = tra_pos/2*cos(double(step)/frequency*PI)+tra_pos/2+tra_pos_limit*2;
            tra_vel_temp = -PI/frequency*tra_pos/2*sin(double(step)/frequency*PI)/0.35;

            cout<< "tra_pos_temp" << tra_pos_temp << "tra_vel_temp" << tra_vel_temp << endl;

            // if(tra_pos_temp<tra_pos_limit)    tra_pos_temp = tra_pos_limit;

            state(0,0) = position;        state(1,0) = velocity;
            state(2,0) = tra_pos_temp;      state(3,0) = tra_vel_temp;

            //////////////////////////	  检查位置是否超限,未超限则进行调整
            if (position_limit_flag)
            {
                if ( position_data > 4000)		{check = 0;}
                if ( position_data < -1000   )		{check = 0;}
                ROS_INFO("check: %d ",check);
            }

            //////////////////////////	  更新agent
            temp(0,0) = u*Qu*u;
            costMat   = statePre.transpose()*Qx*statePre + temp; 	cost = costMat(0,0);
            std::cout << "cost      :"<< cost <<endl;
            temp(0,0) = (K*state)(0,0);
            if(temp(0,0)>limit_current)
            {                temp(0,0)=limit_current;            }
            if(temp(0,0)<-limit_current)
            {                temp(0,0)=-limit_current;           }

            xuPre(0,0) = statePre(0,0);	xu(0,0) = state(0,0);
            xuPre(1,0) = statePre(1,0);	xu(1,0) = state(1,0);
            xuPre(2,0) = statePre(2,0);	xu(2,0) = state(2,0);
            xuPre(3,0) = statePre(3,0);	xu(3,0) = state(3,0);
            xuPre(4,0) = u; 		xu(4,0) = temp(0,0);
            std::cout<<"xu"         <<xu.transpose()        <<endl;
            std::cout<<"xuPre"      <<xuPre.transpose()     <<endl;
            // std::cout<<"xu(2,0)	"<<xu(2,0)<<endl;

            phi(0,0) = xuPre(0,0)*xuPre(0,0) - gam*xu(0,0)*xu(0,0);
            phi(1,0) = xuPre(0,0)*xuPre(1,0) - gam*xu(0,0)*xu(1,0);
            phi(2,0) = xuPre(0,0)*xuPre(2,0) - gam*xu(0,0)*xu(2,0);
            phi(3,0) = xuPre(0,0)*xuPre(3,0) - gam*xu(0,0)*xu(3,0);
            phi(4,0) = xuPre(0,0)*xuPre(4,0) - gam*xu(0,0)*xu(4,0);

            phi(5,0) = xuPre(1,0)*xuPre(1,0) - gam*xu(1,0)*xu(1,0);
            phi(6,0) = xuPre(1,0)*xuPre(2,0) - gam*xu(1,0)*xu(2,0);
            phi(7,0) = xuPre(1,0)*xuPre(3,0) - gam*xu(1,0)*xu(3,0);
            phi(8,0) = xuPre(1,0)*xuPre(4,0) - gam*xu(1,0)*xu(4,0);

            phi(9,0) = xuPre(2,0)*xuPre(2,0) - gam*xu(2,0)*xu(2,0);
            phi(10,0) = xuPre(2,0)*xuPre(3,0) - gam*xu(2,0)*xu(3,0);
            phi(11,0) = xuPre(2,0)*xuPre(4,0) - gam*xu(2,0)*xu(4,0);

            phi(12,0) = xuPre(3,0)*xuPre(3,0) - gam*xu(3,0)*xu(3,0);
            phi(13,0) = xuPre(3,0)*xuPre(4,0) - gam*xu(3,0)*xu(4,0);

            phi(14,0) = xuPre(4,0)*xuPre(4,0) - gam*xu(4,0)*xu(4,0);


            temp       = eye+phi.transpose()*P*phi;             temp = temp.inverse();
            gradient   = P*phi*(costMat - phi.transpose()*theta)*temp;
            P          = P-P*phi*phi.transpose()*P*temp(0,0);
            theta      = theta + gradient;

            //////////////////////////	AC学习更新部分
            if (AC_control)
            {
                H21(0,0) = theta(4,0);    H21(0,1) = theta(8,0);
                H21(0,2) = theta(11,0);   H21(0,3) = theta(13,0);
                H22	 = theta(14,0);
                if (index_jsum_change)
                {                    index_Jum = 2*0.05/(1+gradient.norm()*10);                }
                else{                index_Jum = fixed_jsum;                }
                Jsum = index_Jum*(H21+Kac*H22);

                std::cout << "index_Jum:     " << index_Jum<<endl;
                std::cout << "Jsum:     " << Jsum<<endl;

                if(!test)
                {
                    Kac = Kac-Jsum;
                    if (step>update_step*1 && (step % update_step == 0) )	{ K = Kac;}
                }

            }

            //////////////////////////	数据显示和记录
            std::cout << "K    :   " << K  <<endl;

            // save data state
            StatesRecoder << position << ";   " << velocity << ";   " <<tra_pos_temp<<";   "<<tra_vel_temp<<";    "
                            << int16_t(current_data) << ";   " <<goal_current<<";   "<<std::endl;
            KRecoder      << K(0,0) 	<<";   " << K(0,1)        << ";   "
                << K(0,2) 	<<";   " << K(0,3)        << ";   " << std::endl;
            JsumRecoder <<	u	<<";   " <<	uMat(0,0)	<<";   " << std::endl;
            //  TestRecoder << u << ";  " << position_data <<";  " << velocity_data<< std::endl;

        }
        //////////////////////////	Q学习更新部分, 和AC学习二选一
        if (!AC_control)
        {
            H21(0,0) = theta(4,0);    H21(0,1) = theta(8,0);
            H21(0,2) = theta(11,0);   H21(0,3) = theta(13,0);
            H22	 = theta(14,0);
            K=-H21/H22;
            if(K_limit)
            {
                if(K(0,0)>0)
                    K(0,0)=0;
                if(K(0,1)>0)
                    K(0,1)=0;
            }
        }

        dxl_wb.itemWrite(ID,"Goal_Current",0);	// 关闭电流，保证安全
        ROS_INFO("******      episode end      ******");
        usleep(1000 * 1000);  // sleep for 6 ms

    }

    ROS_INFO("******      learning terminated      ******");
}





































