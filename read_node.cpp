#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sstream>
#include <iostream>
#include <string>
#include <fstream>
#define SPEED_RATIO 0.1
#define CMD_TIMEOUT 0.5

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 2){
        cout << "***error few parameter need datfilename" << endl; 
        return -1;
    }
    
    ros::init(argc, argv, "test_button_state");
    ros::NodeHandle n;
    cout << "Initializing node... " << endl;
    
    float tmp,tmp2,tmp3;
    
    //publisher
    ros::Publisher pub_rate = n.advertise<std_msgs::UInt16>("robot/joint_state_publish_rate", 10);
    
    ros::Publisher pub_speed_ratio_l = n.advertise<std_msgs::Float64>("robot/limb/left/set_speed_ratio", 10);
    ros::Publisher pub_joint_cmd_l = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 10);
    ros::Publisher pub_joint_cmd_timeout_l = n.advertise<std_msgs::Float64>("robot/limb/left/joint_command_timeout", 10);
    
    ros::Publisher pub_speed_ratio_r = n.advertise<std_msgs::Float64>("robot/limb/right/set_speed_ratio", 10);
    ros::Publisher pub_joint_cmd_r = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 10);
    ros::Publisher pub_joint_cmd_timeout_r = n.advertise<std_msgs::Float64>("robot/limb/right/joint_command_timeout", 10);
    
    std::ifstream ifs( argv[1], std::ios::out | std::ios::app );
    
    if (!ifs) {
            cout << "***error  Can not open the file\n";
            exit(1);
    }
    
    //msg変数
    baxter_core_msgs::JointCommand joint_cmd_l;
    joint_cmd_l.names.resize(7);
    joint_cmd_l.command.resize(7);
    string joint_names_l[7] = {"left_e0", "left_e1", "left_s0", "left_s1", "left_w0", "left_w1", "left_w2"};

    baxter_core_msgs::JointCommand joint_cmd_r;
    joint_cmd_r.names.resize(7);
    joint_cmd_r.command.resize(7);
    string joint_names_r[7] = {"right_e0", "right_e1", "right_s0", "right_s1", "right_w0", "right_w1", "right_w2"};

    for(int i=0; i<7; i++){
        joint_cmd_l.names[i] = joint_names_l[i];
        joint_cmd_r.names[i] = joint_names_r[i];
    }

    joint_cmd_l.mode = 1;//position-mode
    joint_cmd_r.mode = 1;//position-mode
    std_msgs::Float64 speed_ratio,cmd_timeout;
    speed_ratio.data = SPEED_RATIO;//joint speed
    cmd_timeout.data = CMD_TIMEOUT;
    std_msgs::UInt16 rate;
    rate.data = 100;

    
    while(true){
        ifs >> tmp; //trash head_node_angle
        ifs >> tmp2;//trash head_pan_angle
        
        //read angles from a datfile
        for(int i=0; i<7; i++){
            ifs >> joint_cmd_l.command[i];
            cout << joint_cmd_l.command[i] << " ";
        }
        cout << endl; 
        
        for(int i=0; i<7; i++){
            ifs >> joint_cmd_r.command[i];
            cout << joint_cmd_r.command[i] << " ";
        }
        cout << endl;   
        
        ifs >> tmp3;//trash torso_angle
        
        for(int i=0; i<30 ;i++){
            pub_rate.publish(rate); //The rate at which the joints are published can be controlled by publishing a frequency on this topic. Default rate is 100Hz; Maximum is 1000Hz
            pub_speed_ratio_l.publish(speed_ratio); //set joint speed default =0.3 range= 0.0-1.0
            pub_joint_cmd_timeout_l.publish(cmd_timeout);
            pub_speed_ratio_r.publish(speed_ratio); //set joint speed default =0.3 range= 0.0-1.0
            pub_joint_cmd_timeout_r.publish(cmd_timeout);      
            pub_joint_cmd_l.publish(joint_cmd_l);
            pub_joint_cmd_r.publish(joint_cmd_r);
            ros::Rate loop_rate(10);
            loop_rate.sleep(); //sleep
        }
        //~ 
        if(ifs.eof()) break;
    }

    cout << "end command" << endl;
    //~ ros::spinOnce(); //callback
    return 0;
}
