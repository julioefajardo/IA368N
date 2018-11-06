#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <math.h>

#include <iostream>

using namespace std;

double pose[3] = {0.0,0.0,0.0};	//x,y,theta
double roll, pitch, yaw;
uint8_t p_flag = 0;

double Gpose[3] = {0.0,0.0,0.0};	//x,y,theta
double Groll, Gpitch, Gyaw;
uint8_t gp_flag = 0;

geometry_msgs::Twist cmd_msg;
float linear_x = 0.0;
float angular_z = 0.0;


void processPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
  tf::Quaternion quat(
       msg->pose.orientation.x,
       msg->pose.orientation.y,
       msg->pose.orientation.z,
       msg->pose.orientation.w);
  tf::Matrix3x3 matrix(quat);
  matrix.getRPY(roll, pitch, yaw);
  pose[0] = msg->pose.position.x;
  pose[1] = msg->pose.position.y;
  pose[2] = yaw;
  p_flag = 1;
  //cout << "Estimated Pose (x,y,theta): " << pose[0] << ", " << pose[1] << ", " << pose[2]*180/M_PI << endl;
}


void processGoalPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
  tf::Quaternion quat(
       msg->pose.orientation.x,
       msg->pose.orientation.y,
       msg->pose.orientation.z,
       msg->pose.orientation.w);
  tf::Matrix3x3 matrix(quat);
  matrix.getRPY(Groll, Gpitch, Gyaw);
  Gpose[0] = msg->pose.position.x;
  Gpose[1] = msg->pose.position.y;
  Gpose[2] = Gyaw;
  gp_flag = 1;
  cout << "Goal Pose (x,y,theta): " << Gpose[0] << ", " << Gpose[1] << ", " << Gpose[2]*180/M_PI << endl;
}

void angleNormalization(double * angle){
  while(*angle>M_PI) *angle -= 2.0*M_PI;
  while(*angle<-M_PI) *angle += 2.0*M_PI;    
}

int main(int argc, char** argv){

  ros::init(argc, argv, "my_controller_node");
  ros::NodeHandle nh; //if it is necessary for passing parameters from the console add -> nh("~")

  ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("/slam_out_pose",10,&processPose);
  ros::Subscriber goalPoseSub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,&processGoalPose);

  ros::Publisher cmdPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  float k_rho_ = 0.5; 
  float k_alpha_ = 1.5; 
  float k_beta_ = -0.6; 
  bool backward_allowed_ = false;
  bool use_constant_vel_ = false;
  float constant_vel_ = 0.1;

  nh.getParam("k_rho",k_rho_); 
  nh.getParam("k_alpha",k_alpha_); 
  nh.getParam("k_beta",k_beta_); 
  nh.getParam("backward_allowed",backward_allowed_); 
  nh.getParam("use_constant_vel",use_constant_vel_); 
  nh.getParam("constant_vel",constant_vel_); 

  ros::Rate rate(5);
  
  cout << endl;
  cout << "========================" << endl;
  cout << "  Activity 2 - IA368N" << endl;
  cout << "========================" << endl;
  cout << "Parameters Configuration" << endl;
  cout << "========================" << endl;
  cout << "k_rho = " << k_rho_ << endl;
  cout << "k_alpha = " << k_alpha_ << endl;
  cout << "k_beta = " << k_beta_ << endl;
  cout << "backward_allowed: "<< backward_allowed_ << endl;
  cout << "use_constant_vel: "<< use_constant_vel_ << endl;
  cout << "constant_vel: " << constant_vel_ << endl << endl;

  cmd_msg.linear.x = 0.0;
  cmd_msg.linear.y = 0.0;
  cmd_msg.linear.z = 0.0;
  cmd_msg.angular.x = 0.0;
  cmd_msg.angular.y = 0.0;
  cmd_msg.angular.z = 0.0;  
  
  while(ros::ok()){

    if(gp_flag){
      double delta_x = Gpose[0] - pose[0];
      double delta_y = Gpose[1] - pose[1];
      double rho = sqrt(delta_x*delta_x + delta_y*delta_y);
      double lambda = atan2(delta_y, delta_x);
      double alpha = lambda - pose[2];

      //angle normalization
      angleNormalization(&alpha);
      
      //double beta = -Gpose[2] - alpha;
      double beta = -lambda + Gpose[2] ;
      
      //if(use_constant_vel_) linear_x = constant_vel_; 
      //else linear_x = k_rho_*rho;
      linear_x = (use_constant_vel_)?constant_vel_:k_rho_*rho;

      if(backward_allowed_){
        if(lambda>Gpose[2]) angular_z = k_alpha_*alpha;
        else angular_z = -k_alpha_*alpha;
      }
      else angular_z = k_alpha_*alpha + k_beta_*beta;
      
      angular_z = (angular_z<=(M_PI/4.0))?angular_z:(M_PI/4.0); 

      cmd_msg.linear.x = linear_x;
      cmd_msg.angular.z = angular_z;
      cmdPub.publish(cmd_msg);

      double delta_theta = pose[2]-Gpose[2];
      //angle normalization
      angleNormalization(&delta_theta); 
      
      //stop condition
      if((rho<0.2)&&(abs(delta_theta)<((5*M_PI)/180.0))){
        gp_flag = 0;
        cout << "Task done!" << endl;
        cout << "Robot Pose (x,y,theta): " << pose[0] << ", " << pose[1] << ", " << pose[2]*180/M_PI << endl;
      }
    } else {
      cmd_msg.linear.x = 0.0;
      cmd_msg.angular.z = 0.0;
      cmdPub.publish(cmd_msg);
    }

    rate.sleep();
    ros::spinOnce(); 
  }
}

