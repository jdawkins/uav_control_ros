#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>

#define STABILIZE_MODE 0    //Sets roll pitch yawrate and thrust
#define ALT_HOLD_MODE 1     //Sets roll pitch yawrate and climbrate
#define AUTO_MODE 2         //Sets roll pitch yaw and climbrate
#define ACRO_MODE 3         //Sets rollrate pitchrate yawrate and climbrate
#define FREE_MODE 4         //Sets rollrate pitchrate yawrate and thrust
#define PI (3.14159)
#define MAX_YAW_RATE PI     //Pi radians/sec half turn in a second
#define MAX_RP_ANGLE 20*PI/180       //degrees max lean angle converted to radians
#define MAX_RP_RATE PI
#define HOVER_THRUST 0.3;



ros::Subscriber joy_sub;
ros::Subscriber acc_sub;
ros::Subscriber pose_sub;
ros::Publisher cmd_pub;

nav_msgs::Odometry uav_cmd;

float LX,LY,LT,RX,RY,RT,DPX,DPY;
bool BA,BB,BX,BY;
int flight_mode;
std::string uav_type;
nav_msgs::Odometry pose;
double Kp,Kd,Kpsi;
bool auto_control;


void joyCallBack(const sensor_msgs::Joy &joy_msg){
    auto_control = false;

    LX = joy_msg.axes[0];
    LY = joy_msg.axes[1];
    RX = -joy_msg.axes[3];
    RY = joy_msg.axes[4];

    BA = joy_msg.buttons[0];
    BB = joy_msg.buttons[1];
    BX = joy_msg.buttons[2];
    BY = joy_msg.buttons[3];

    if(BA){flight_mode = STABILIZE_MODE;}
    if(BB){flight_mode = ALT_HOLD_MODE;}
    if(BX){auto_control = true;}
    if(BY){flight_mode = ACRO_MODE;}

    switch(flight_mode){

        case STABILIZE_MODE:
        {
            if(LY>0){ //only use positive values
             uav_cmd.twist.twist.linear.z = LY; //Thust, use z velocity

            }else{
             uav_cmd.twist.twist.linear.z = 0; //set to zero just to be sure
            }

            uav_cmd.twist.twist.angular.z = MAX_YAW_RATE*LX;
            uav_cmd.pose.pose.orientation.x = MAX_RP_ANGLE*RX;
            uav_cmd.pose.pose.orientation.y = MAX_RP_ANGLE*RY;
            break;

        }
        case ALT_HOLD_MODE:
        {
            uav_cmd.twist.twist.linear.z = LY; //Thust, use z velocity
            uav_cmd.twist.twist.angular.z = MAX_YAW_RATE*LX;
            uav_cmd.pose.pose.orientation.x = MAX_RP_ANGLE*RX;
            uav_cmd.pose.pose.orientation.y = MAX_RP_ANGLE*RY;
            break;
        }
        case AUTO_MODE:
        {
            uav_cmd.twist.twist.linear.z = LY; //Thust, use z velocity
            uav_cmd.pose.pose.orientation.z = LX;
            uav_cmd.pose.pose.orientation.x = MAX_RP_ANGLE*RX;
            uav_cmd.pose.pose.orientation.y = MAX_RP_ANGLE*RY;
            break;
        }
        case ACRO_MODE:
        {

            uav_cmd.twist.twist.linear.z = LY; //Thust, use z velocity
            uav_cmd.twist.twist.angular.z = MAX_YAW_RATE*LX;
            uav_cmd.twist.twist.angular.x = MAX_RP_RATE*RX;
            uav_cmd.twist.twist.angular.y = MAX_RP_RATE*RY;
            break;
        }
        default:
        break;
    }

}

void poseCallBack(const nav_msgs::Odometry &pose_msg){
    pose = pose_msg;

}

void accelCallBack(const geometry_msgs::Vector3 &des_accel){

    //RollVeldes  = (1/g)*( Xcom(1,1)*sin(Ydes(3,1))- Xcom(2,1)*cos(Ydes(3,1)) );
    //PitchVeldes = (1/g)*( Xcom(1,1)*cos(Ydes(3,1))- Xcom(2,1)*sin(Ydes(3,1)) );
    //u1 = m*( g+Xaccel(3,1)+Kdz(Xvel(3,1)-zvel)+Kpz(Xpos(3,1)-zpos) );
    if(auto_control){
    uav_cmd.pose.pose.orientation.x = des_accel.x*cos(pose.pose.pose.orientation.z) - des_accel.y*sin(pose.pose.pose.orientation.z);
    uav_cmd.pose.pose.orientation.y = des_accel.x*sin(pose.pose.pose.orientation.z) - des_accel.y*cos(pose.pose.pose.orientation.z);
    uav_cmd.twist.twist.angular.z = atan2(des_accel.y,des_accel.x) - pose.pose.pose.orientation.z;
    uav_cmd.twist.twist.linear.z = (0.2*des_accel.z)+0.5;
    }

}
int main(int argc, char **argv){

    flight_mode = STABILIZE_MODE;
    ros::init(argc, argv, "uav_control_node");

    ros::NodeHandle nh;
    auto_control = false;

    cmd_pub = nh.advertise<nav_msgs::Odometry>("uav_cmd",100);
    joy_sub = nh.subscribe("joy",1000,joyCallBack);
    pose_sub = nh.subscribe("uav_pose",1000,poseCallBack);
    acc_sub = nh.subscribe("des_accel",1000,accelCallBack);


    if(nh.getParam("uav_type",uav_type)){
        ROS_INFO("Using parameter %s",uav_type.c_str());
    }else{
        uav_type = "generic";
        ROS_WARN("Defaulting to Uav type %s",uav_type.c_str());
    }

    if(nh.getParam("gains/Kp",Kp)){
        ROS_INFO("Using parameter Kp = %2.3f",Kp);
    }else{
        Kp = 0.5;
        ROS_WARN("Defaulting to Kp Gain %2.3f",Kp);
    }
    if(nh.getParam("gains/Kd",Kd)){
        ROS_INFO("Using parameter Kp = %2.3f",Kd);
    }else{
        Kd = 0.1;
        ROS_WARN("Defaulting to Kd Gain %2.3f",Kd);
    }

    if(nh.getParam("gains/Kpsi",Kpsi)){
        ROS_INFO("Using parameter Kpsi = %2.3f",Kpsi);
    }else{
        Kpsi = 0.1;
        ROS_WARN("Defaulting to Kpsi Gain %2.3f",Kpsi);
    }

    ros::Rate loop_rate(100);
    while(ros::ok()){

        cmd_pub.publish(uav_cmd);
        ros::spinOnce();// Allow ROS to check for new ROS Messages
        loop_rate.sleep(); //Sleep for some amount of time determined by loop_rate

    }

    return 0;
}
