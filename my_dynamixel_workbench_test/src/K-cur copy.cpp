//////////////////////////    headfiles    //////////////////////////
//math
#include <math.h>
//#include <Eigen/Dense>
#include <Eigen/Eigen>
//#include <Eigen/KroneckerProduct>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
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
#define BAUDRATE 57600
#define ID 0
#define random(x) (rand()%x)
#define PI 3.1415926



//////////////////////////    begin    //////////////////////////
int main(int argc,char **argv)
{
	//////////////////////////	  parameters    //////////////////////////
	int eps_max  = 10;
	int step_max = 255;		int step = 0;
	int update_step =  20;
	double gam = 0.95;
	double Q1 = 1000;		double Q2 = 200;		
	double Qf = 1;			double Qu = 1;
	Eigen::MatrixXd Qx(3, 3);
	Qx(0,0) = Qf*Qf*Q1;		Qx(0,1) = 0;		Qx(0,2) = Qf*Q1;
	Qx(1,0) = 0;			Qx(1,1) = Q2;		Qx(1,2) = 0;
	Qx(2,0) = Qf*Q1;		Qx(2,1) = 0;		Qx(2,2) = Q1;
	double coeff = 1000.0;
	int       tar_position = 4096/2/coeff;


	//////////////////////////	  on-offs    //////////////////////////
	bool check = false;


	//////////////////////////	  state variable    //////////////////////////
	Eigen::MatrixXd state(3,1) ;	Eigen::MatrixXd statePre(3,1) ;	
	Eigen::MatrixXd dstate(3,1) ;
	double f;						double u;			Eigen::MatrixXd uMat(1, 1);	
	Eigen::MatrixXd K(1, 3);		Eigen::MatrixXd Kac(1, 3);
	K(0,0)=-32; K(0,1)=-50; K(0,2)=0;		Kac(0,0)=-1; Kac(0,1)=-1; Kac(0,2)=0;
	Eigen::MatrixXd P(16, 16);		P = MatrixXd::Identity(16,16)*10000;
	Eigen::MatrixXd theta(16, 1);   theta = MatrixXd::Zero(16,1);
	Eigen::MatrixXd gradient(16, 1);
	double cost;					Eigen::MatrixXd costMat(1, 1);	
	Eigen::MatrixXd xu(4,1);		Eigen::MatrixXd xuPre(4,1) ;
	Eigen::MatrixXd phi(16,1);
	Eigen::MatrixXd H(4,4);
	Eigen::MatrixXd H21(1,3);		double H22;
	Eigen::MatrixXd H44(4, 4);
	double index_Jum;				Eigen::MatrixXd Jsum(1, 3);
	Eigen::MatrixXd eye(1,1);		eye(0,0) = 1;
	Eigen::MatrixXd temp(1,1);		temp(0,0) = 1;

  ros::init(argc,argv,"ACimpedent");
  ros::NodeHandle n;
  ros::Publisher  dxl_state_pub = n.advertise<my_dynamixel_workbench_test::dxl_state>("dxl_state_topic",100);  //定义舵机状态发布器
  my_dynamixel_workbench_test::dxl_state msg;
  

	

  //////////////////////////    舵机数据定义
  int 		position_data = 0;
  float 	position = 0.0;
  int32_t 	velocity_data = 0;
  float 	velocity = 0.0;
  int32_t	current_data = 0;
  float 	current = 0.0;
  int32_t	 	goal_current;
  int32_t 		limit_current = 638/2;	//2.69 [mA]

 
  //////////////////////////    连接舵机，设置电流控制模式
  DynamixelWorkbench dxl_wb; 
  dxl_wb.begin("/dev/ttyUSB0",BAUDRATE);
  dxl_wb.ping(ID);
  dxl_wb.ledOff(ID);
  ROS_INFO("Welcome my dynamixel workbench!");
  dxl_wb.setCurrentControlMode(ID);
  dxl_wb.itemRead(ID,"Current_Limit",&limit_current);
  dxl_wb.torqueOn(ID);
  
  //////////////////////////    舵机初始化
  

  dxl_wb.itemRead(ID,"Present_Position",&position_data);
	// position = dxl_wb.convertValue2Position(ID,position_data);
	position_data = position_data*0.088/PI;
  dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
	// velocity = dxl_wb.convertValue2Velocity(ID,velocity_data);
	position_data = velocity_data*0.229*PI/60;
  dxl_wb.itemRead(ID,"Present_Current",&current_data);
	// current  = dxl_wb.convertValue2Current(ID,current_data);
	current	 = current_data*2.69;   // mA
  
  //////////////////////////	定义信息流
  // 在home下创建文件夹 ACimpedent
  std::ofstream StatesRecoder("/home/ACimpedent/statusRecoder.txt");
  std::ofstream KRecoder("/home/ACimpedent/kRecoder.txt");
  
  //////////////////////////	学习过程     //////////////////////////
  for (int eps = 0; eps < eps_max; eps++) 
	{		// episode
	 ROS_INFO("******      episode begin      ******");
	 //////////////////////////    找到起始位置
	 ROS_INFO("******      go to init pos      ******");
	 dxl_wb.torqueOff(ID);
	 dxl_wb.setPositionControlMode(ID);
	 dxl_wb.torqueOn(ID);
	 //dxl_wb.itemWrite(ID,"Goal_Position",2048);
	 dxl_wb.itemWrite(ID,"Goal_Position",1000);
	 sleep(2);
	 dxl_wb.itemRead(ID,"Present_Position",&position_data);
	 ROS_INFO("position:%d",position_data);
	 usleep(2000 * 1000);  	 
	 ROS_INFO("******      reach init pos      ******");
	
	
	 //////////////////////////	   学习开始
	 //修改控制模型并获得当前状态
	 dxl_wb.torqueOff(ID);
	 usleep(10);
	 dxl_wb.setCurrentControlMode(ID);
	 usleep(10);
	 dxl_wb.torqueOn(ID);

	 usleep(10 * 000);  	 
	 dxl_wb.itemRead(ID,"Present_Position",&position_data);
	 dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
	 dxl_wb.itemRead(ID,"Present_Current" ,&current_data);
	 // 获得起始状态，通过除以一个系数使得系统计算值不要太大，通过减去tar_position将目标位置调节至零位
     state(0,0) = (position_data)/coeff - tar_position;	 state(1,0) = velocity_data/coeff;	state(2,0) = 0;
	 ROS_INFO("******      start learning      ******");


	 step = 0;			check = true;
	 P = P + MatrixXd::Ones(16,16)*P.norm();
	 while (step < (step_max - 5) && ros::ok() && check ) 
		{	//   step
		 step++;
		
		 //////////////////////////    计算输出并发送给舵机
		 statePre = state;
		 uMat = K*state;
		 u        = uMat(0,0);			goal_current = int32_t(u*10);
		
		 if(goal_current>limit_current)
			{
				goal_current = limit_current;
			}
			if(goal_current<-limit_current)
			{
				goal_current = -limit_current;
			}
			ROS_INFO("goal_current:%d",goal_current);
			dxl_wb.itemWrite(ID,"Goal_Current",goal_current);
			usleep(2*1000);
		 
		 //////////////////////////	  更新状态
		 dxl_wb.itemRead(ID,"Present_Position",&position_data);
		 dxl_wb.itemRead(ID,"Present_Velocity",&velocity_data);
		 dxl_wb.itemRead(ID,"Present_Current" ,&current_data);
		 f = 0.0;        				//f =  externalF();
		 // 通过除以一个系数使得系统计算值不要太大，通过减去tar_position将目标位置调节至零位
		 state(0,0) = position_data/coeff - tar_position;	state(1,0) = velocity_data/coeff;		state(2,0) = f;
		 if(abs(state(0,0))<0.01)
		 {
			 state(0,0)=0;
		 }
		 //////////////////////////	  检查位置是否超限,未超限则进行调整
		 if ( position_data > 4000)		{check = 0;}
		 if ( position_data < 100)		{check = 0;}
		 ROS_INFO("check: %d ",check);
		//  state(0,0) = state(0,0) - 4096/2 ;
		 
		
		 //////////////////////////	  更新agent
		costMat = state.transpose()*Qx*state + uMat*Qu*uMat; 	cost = costMat(0,0);  

		xuPre(0,0) = statePre(0,0);	xu(0,0) = state(0,0);
		xuPre(1,0) = statePre(1,0);	xu(1,0) = state(1,0);
		xuPre(2,0) = statePre(2,0);	xu(2,0) = 0;
		xuPre(3,0) = u; 			xu(3,0) = (K*state)(0,0)*10;
		std::cout<<"xu"<<xu.transpose()<<endl;

		  phi        = kroneckerProduct(xuPre,xuPre)-gam*kroneckerProduct(xu,xu);
		  temp 		 = eye+phi.transpose()*P*phi;   temp = temp.inverse();
		  gradient   = P*phi*(costMat - phi.transpose()*theta)*temp;
		  P          = P-P*phi*phi.transpose()*P*temp(0,0);
		  theta      = theta + gradient;

		 //////////////////////////	AC学习更新部分
		 H21(0,0) = theta(3,0); 
		 H21(0,1) = theta(7,0); 
		 H21(0,2) = theta(11,0); 
		 H22      = theta(15,0); 

		 index_Jum = 0.05/(1+gradient.norm()*0.05);
		 index_Jum = 0.005;
		 std::cout << "index_Jum:     " << index_Jum<<endl;
		 Jsum = index_Jum*(H21+Kac*H22);
		 std::cout << "Jsum:     " << Jsum<<endl;
		 
		 Kac = Kac-Jsum;
		 Kac(0,2)=0;

		 if (step>update_step*1 && (step % update_step == 0) )	{ K = Kac;}	


		 //////////////////////////	数据显示和记录
		std::cout << "xuPre:     " << xuPre.transpose()<< std::endl;
		std::cout << "xu:        " << xu.transpose()   << std::endl;
		std::cout << "H21:       " << H21              << std::endl;
		std::cout << "H22:       " << H22              << std::endl;
		std::cout << "state:     " << state(0,0) << " " << state(1,0) << " " << state(2,0) << " " << std::endl;
		std::cout << "Kac    :   " << Kac(0,0) 		  << " " << Kac(0,1) 		  << " " << Kac(0,2) 		 << " " << std::endl;
		std::cout << "K    :   " << K  <<endl;
         // save data state
        //  StatesRecoder << position_data << ";   " << velocity_data << ";   " << current_data << ";   " << std::endl;
	    //  KRecoder      << K(0,0) 		   << ";   " << K(0,1) 		  << ";   " << K(0,2) 		<< ";   " << std::endl;
		
		}
		//////////////////////////	Q学习更新部分, 和AC学习二选一
		// K=H21+Kac*H22;
		
		
	 dxl_wb.itemWrite(ID,"Goal_Current",0);	// 关闭电流，保证安全	
	 ROS_INFO("******      episode end      ******");
	 usleep(1000 * 1000);  // sleep for 6 ms
	 
	}
	
  ROS_INFO("******      learning terminated      ******");
}
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  














