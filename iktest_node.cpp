#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <sstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_joint_velocity_wobbler");
    ros::NodeHandle n;
    cout << "Initializing node... " << endl;

    //publisher
    ros::Publisher pub_rate = n.advertise<std_msgs::UInt16>("robot/joint_state_publish_rate", 10);
    ros::Publisher pub_speed_ratio = n.advertise<std_msgs::Float64>("robot/limb/left/set_speed_ratio", 10);
    ros::Publisher pub_joint_cmd = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 10);
    ros::Publisher pub_joint_cmd_timeout = n.advertise<std_msgs::Float64>("robot/limb/left/joint_command_timeout", 10);

    //service
    ros::ServiceClient client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/left/PositionKinematicsNode/IKService");

    //msg
    baxter_core_msgs::JointCommand joint_cmd;
    joint_cmd.names.resize(7);
    joint_cmd.command.resize(7);
    string joint_names[7] = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    //~ float joint_angles[7] = {0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0};
    
    //srv
    baxter_core_msgs::SolvePositionIK iksvc;
    iksvc.request.pose_stamp.resize(1);
    iksvc.response.joints.resize(1);
    geometry_msgs::PoseStamped a;
    double deg = 90.0; 
    iksvc.request.pose_stamp[0].header.stamp = ros::Time::now();
    iksvc.request.pose_stamp[0].header.frame_id = "base";
    iksvc.request.pose_stamp[0].pose.position.x = 0.657579481614;
    iksvc.request.pose_stamp[0].pose.position.y = 0.451981417433;
    iksvc.request.pose_stamp[0].pose.position.z = 0.3388352386502;
    //~ iksvc.request.pose_stamp[0].pose.orientation.x = -0.366894936773 / sin((149.60317/2.0)*M_PI/180.0);
    //~ iksvc.request.pose_stamp[0].pose.orientation.y = 0.085980397775 / sin((149.60317/2.0)*M_PI/180.0);
    //~ iksvc.request.pose_stamp[0].pose.orientation.z = 0.008155782462 / sin((149.60317/2.0)*M_PI/180.0);
    iksvc.request.pose_stamp[0].pose.orientation.x = 1.0 * sin((deg/2.0)*M_PI/180.0);
    iksvc.request.pose_stamp[0].pose.orientation.y = 0.0 * sin((deg/2.0)*M_PI/180.0);
    iksvc.request.pose_stamp[0].pose.orientation.z =0.0 * sin((deg/2.0)*M_PI/180.0);
    //~ iksvc.request.pose_stamp[0].pose.orientation.w = 0.262162481772;
    iksvc.request.pose_stamp[0].pose.orientation.w = cos((deg/2.0)*M_PI/180.0);
    if(client.call(iksvc)){
        ROS_INFO("SUCCESS to call service");
    }
    else{
        ROS_ERROR("FAILED to call service");
        return 1;
    }
    
    if(iksvc.response.isValid[0]){
        ROS_INFO("SUCCESS - Valide Joint Sloution Found:");
    }
    else{
        ROS_ERROR("INVALID POSE");
    }
    
    
    
    for(int i=0; i<7; i++){
        joint_cmd.names[i] = joint_names[i];
        joint_cmd.command[i] = iksvc.response.joints[0].position[i];
        cout << joint_cmd.names[i] << iksvc.response.joints[0].position[i] << endl;
    }

    joint_cmd.mode = 1;//position
    std_msgs::Float64 speed_ratio,cmd_timeout;
    speed_ratio.data=0.1;
    cmd_timeout.data=0.2;
    std_msgs::UInt16 rate;
    rate.data = 100;

    for(int i=0; i<50 ;i++){
        pub_rate.publish(rate); //The rate at which the joints are published can be controlled by publishing a frequency on this topic. Default rate is 100Hz; Maximum is 1000Hz
        pub_speed_ratio.publish(speed_ratio); //set joint speed default =0.3 range= 0.0-1.0
        pub_joint_cmd_timeout.publish(cmd_timeout);
        pub_joint_cmd.publish(joint_cmd);
        ros::Rate loop_rate(5);
        loop_rate.sleep(); //sleep
    }
    
    cout << "end command" << endl;
    ros::spinOnce(); //callbackyoudesu
    return 0;
}
