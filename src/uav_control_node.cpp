#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>

#define STABILIZE_MODE 0    //Sets roll pitch yawrate and thrust
#define ALT_HOLD_MODE 1     //Sets roll pitch yawrate and climbrate
#define AUTO_MODE 2         //Sets roll pitch yaw and climbrate
#define ACRO_MODE 3         //Sets rollrate pitchrate yawrate and climbrate
#define FREE_MODE 4         //Sets rollrate pitchrate yawrate and thrust
#define PI (3.14159)
#define MAX_YAW_RATE PI     //Pi radians/sec half turn in a second
#define MAX_RP_ANGLE 20*PI/180       //degrees max lean angle converted to radians
#define MAX_RP_RATE PI
#define DEFAULT_HOVER_THRUST 0.5;



ros::Subscriber joy_sub;
ros::Subscriber acc_sub;
ros::Subscriber pose_sub;
ros::Publisher cmd_pub;

nav_msgs::Odometry uav_cmd;

float LX,LY,LT,RX,RY,RT,DPX,DPY;
bool BA,BB,BX,BY,BL1,BR1;
int flight_mode;
std::string uav_type, uav_name,vicon_sub_msg;
//nav_msgs::Odometry pose;
geometry_msgs::TransformStamped  pose;
double Kp,Kd,Kpsi,Khov,Kidle,thrust_range;
bool armed;

float roll_cmd;
float pitch_cmd;
float yawrate_cmd;


void saturateCommands(nav_msgs::Odometry &cmd){

//Saturate Thrust Commands
    if(cmd.twist.twist.linear.z > 1){
       cmd.twist.twist.linear.z = 1;
    }
    if(cmd.twist.twist.linear.z < 0){
       cmd.twist.twist.linear.z = 0;
    }

//Saturate Roll and Pitch Commands
    if(cmd.pose.pose.orientation.x > MAX_RP_ANGLE){
        cmd.pose.pose.orientation.x = MAX_RP_ANGLE;
    }else if(cmd.pose.pose.orientation.x < -MAX_RP_ANGLE){
        cmd.pose.pose.orientation.x = -MAX_RP_ANGLE;
    }else{

    }

    if(cmd.pose.pose.orientation.y > MAX_RP_ANGLE){
        cmd.pose.pose.orientation.y = MAX_RP_ANGLE;
    }else if(cmd.pose.pose.orientation.y < -MAX_RP_ANGLE){
        cmd.pose.pose.orientation.y = -MAX_RP_ANGLE;
    }else{

    }

////Saturate YawRate Commands
    if(cmd.twist.twist.angular.z > MAX_YAW_RATE){
        cmd.twist.twist.angular.z = MAX_YAW_RATE;
    }else if(cmd.twist.twist.angular.z < -MAX_YAW_RATE){
        cmd.twist.twist.angular.z = -MAX_YAW_RATE;
    }else{

    }
}

void joyCallBack(const sensor_msgs::Joy &joy_msg){
    //auto_control = false;
    //alt_control = false;

    LX = joy_msg.axes[0];
    LY = joy_msg.axes[1];
    RX = -joy_msg.axes[3];
    RY = joy_msg.axes[4];

    BA = joy_msg.buttons[0];
    BB = joy_msg.buttons[1];
    BX = joy_msg.buttons[2];
    BY = joy_msg.buttons[3];
    BL1 = joy_msg.buttons[4];
    BR1 = joy_msg.buttons[5];

    if(BA){flight_mode = STABILIZE_MODE;
        armed = !armed;
    }
    if(BB){flight_mode = STABILIZE_MODE;}
    if(BX){flight_mode = STABILIZE_MODE;}
    if(BY){flight_mode = STABILIZE_MODE;}



    if(armed){

        if(BL1){flight_mode = ALT_HOLD_MODE;
            ROS_INFO("Alt Hold Mode");
        }
        if(BR1){flight_mode = AUTO_MODE;
            ROS_INFO("Auto Mode Set");
        }

        switch(flight_mode){

            case STABILIZE_MODE:
            {
                if(LY>0){ //only use positive values
                 uav_cmd.twist.twist.linear.z = LY*(1 - Kidle) + Kidle; //Thust, use z velocity

                }else{
                 uav_cmd.twist.twist.linear.z = Kidle; //set to zero just to be sure
                }

                uav_cmd.twist.twist.angular.z = MAX_YAW_RATE*LX;
                uav_cmd.pose.pose.orientation.x = MAX_RP_ANGLE*RX;
                uav_cmd.pose.pose.orientation.y = MAX_RP_ANGLE*RY;
                break;

            }
            case ALT_HOLD_MODE:
            {
                //uav_cmd.twist.twist.linear.z = LY; //Thust, use z velocity
                uav_cmd.twist.twist.angular.z = MAX_YAW_RATE*LX;
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
    }else{
        uav_cmd.twist.twist.linear.z = 0;
        uav_cmd.twist.twist.angular.z = 0;
        uav_cmd.pose.pose.orientation.x = 0;
        uav_cmd.pose.pose.orientation.y = 0;

    }
}

void accelCallBack(const geometry_msgs::Vector3 &des_accel){

    switch(flight_mode){


        case ALT_HOLD_MODE:
        {
            uav_cmd.twist.twist.linear.z = des_accel.z+Khov; //Thust, use z velocity
            saturateCommands(uav_cmd);
            //uav_cmd.twist.twist.angular.z = MAX_YAW_RATE*LX;
            //uav_cmd.pose.pose.orientation.x = MAX_RP_ANGLE*RX;
            //uav_cmd.pose.pose.orientation.y = MAX_RP_ANGLE*RY;
            break;
        }
        case AUTO_MODE:
        {

        uav_cmd.pose.pose.orientation.x = -des_accel.x*sin(pose.transform.rotation.z) + des_accel.y*cos(pose.transform.rotation.z);
        uav_cmd.pose.pose.orientation.y = -des_accel.x*cos(pose.transform.rotation.z) + des_accel.y*sin(pose.transform.rotation.z);

        //uav_cmd.pose.pose.orientation.y = des_accel.x*cos(pose.transform.rotation.z) + des_accel.y*sin(pose.transform.rotation.z);
        //uav_cmd.pose.pose.orientation.x = -des_accel.x*sin(pose.transform.rotation.z) + des_accel.y*cos(pose.transform.rotation.z);
        uav_cmd.twist.twist.angular.z = Kpsi*(atan2(des_accel.x,des_accel.y) - pose.transform.rotation.z);
        uav_cmd.twist.twist.linear.z = des_accel.z+Khov;
        saturateCommands(uav_cmd);
            break;
        }

        default:
        break;
    }

    //RollVeldes  = (1/g)*( Xcom(1,1)*sin(Ydes(3,1))- Xcom(2,1)*cos(Ydes(3,1)) );
    //PitchVeldes = (1/g)*( Xcom(1,1)*cos(Ydes(3,1))- Xcom(2,1)*sin(Ydes(3,1)) );
    //u1 = m*( g+Xaccel(3,1)+Kdz(Xvel(3,1)-zvel)+Kpz(Xpos(3,1)-zpos) );

}
int main(int argc, char **argv){


    flight_mode = STABILIZE_MODE;
    ros::init(argc, argv, "uav_control_node");

    ros::NodeHandle nh;
    armed = false;

    cmd_pub = nh.advertise<nav_msgs::Odometry>("uav_cmd",100);
    joy_sub = nh.subscribe("joy",1000,joyCallBack);
    acc_sub = nh.subscribe("des_accel",1000,accelCallBack);


    if(nh.getParam("uav_type",uav_type)){
        ROS_INFO("Using parameter %s",uav_type.c_str());
    }else{
        uav_type = "generic";
        ROS_WARN("Defaulting to Uav type %s",uav_type.c_str());
    }

    if(nh.getParam("uav_name",uav_name)){
        ROS_INFO("Using parameter %s",uav_name.c_str());
    }else{
        uav_name = "generic_uav";
        ROS_ERROR("You must define a UAV name which TF will be published on");
    }


    if(nh.getParam("gains/Khov",Khov)){
        ROS_INFO("Using parameter hover_thrust = %2.3f",Khov);
    }else{
        Khov = DEFAULT_HOVER_THRUST;
        ROS_WARN("Defaulting to hover thrust %2.3f",Khov);
    }

    if(nh.getParam("gains/Kidle",Kidle)){
        ROS_INFO("Using parameter hover_thrust = %2.3f",Kidle);
    }else{
        Kidle = 0.1;
        ROS_WARN("Defaulting to hover thrust %2.3f",Kidle);
    }


    if(nh.getParam("gains/Kpsi",Kpsi)){
        ROS_INFO("Using parameter Kpsi = %2.3f",Kpsi);
    }else{
        Kpsi = 0.1;
        ROS_WARN("Defaulting to Kpsi Gain %2.3f",Kpsi);
    }

    vicon_sub_msg = "vicon/";
    vicon_sub_msg.append(uav_name.c_str());
    vicon_sub_msg.append("/Body");

    thrust_range = 1 - Kidle;

    tf::TransformListener listener;

    ros::Rate loop_rate(100);
    while(ros::ok()){


        tf::StampedTransform transform;
        try{
          listener.lookupTransform("world",vicon_sub_msg,
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }

        tf::Quaternion q = transform.getRotation();

        tf::Matrix3x3(q).getRPY(pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z);


        cmd_pub.publish(uav_cmd);
        ros::spinOnce();// Allow ROS to check for new ROS Messages
        loop_rate.sleep(); //Sleep for some amount of time determined by loop_rate

    }

    return 0;
}
