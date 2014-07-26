#include <ros/ros.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sstream>
#include <iostream>
#include <string>
#include <fstream>
#define SPEED_RATIO 0.1
#define CMD_TIMEOUT 0.5

using namespace std;

class Read_node{
    private:
    ros::NodeHandle n;
	ros::Publisher pub_rate, pub_speed_ratio_l,pub_joint_cmd_l, pub_joint_cmd_timeout_l, pub_speed_ratio_r, pub_joint_cmd_r, pub_joint_cmd_timeout_r;
    float tmp,tmp2,tmp3;

     //msg
    baxter_core_msgs::JointCommand joint_cmd_l;
    string joint_names_l[7];
    baxter_core_msgs::JointCommand joint_cmd_r;
    string joint_names_r[7];
    std_msgs::Float64 speed_ratio,cmd_timeout;
    std_msgs::UInt16 rate;
    std::fstream ifs;

    public:
    char ns[30];
    
    Read_node{
		pub_rate = n.advertise<std_msgs::UInt16>("robot/joint_state_publish_rate", 10);
		pub_speed_ratio_l = n.advertise<std_msgs::Float64>("robot/limb/left/set_speed_ratio", 10);
		pub_joint_cmd_l = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 10);
		pub_joint_cmd_timeout_l = n.advertise<std_msgs::Float64>("robot/limb/left/joint_command_timeout", 10);
		pub_speed_ratio_r = n.advertise<std_msgs::Float64>("robot/limb/right/set_speed_ratio", 10);
		pub_joint_cmd_r = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 10);
		pub_joint_cmd_timeout_r = n.advertise<std_msgs::Float64>("robot/limb/right/joint_command_timeout", 10);
		
        ifs.open( ns, std::ios::out | std::ios::app );
        if (!ifs) {
                cout << "***error  Can not open the file\n";
                exit(1);
        }
        
        joint_cmd_l.names.resize(7);
        joint_cmd_l.command.resize(7);
        joint_cmd_r.names.resize(7);
        joint_cmd_r.command.resize(7);
        joint_names_l = {"left_e0", "left_e1", "left_s0", "left_s1", "left_w0", "left_w1", "left_w2"};
        joint_names_r = {"right_e0", "right_e1", "right_s0", "right_s1", "right_w0", "right_w1", "right_w2"};
        speed_ratio.data = SPEED_RATIO;//joint speed
        cmd_timeout.data = CMD_TIMEOUT;
        joint_cmd_l.mode = 1;//position-mode
        joint_cmd_r.mode = 1;//position-mode    
        rate.data = 100;
                
        for(int i=0; i<7; i++){
            joint_cmd_l.names[i] = joint_names_l[i];
            joint_cmd_r.names[i] = joint_names_r[i];
        }
    }
    
    void move_arm(){
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
            if(ifs.eof()) break;
        }
     
        cout << "end command" << endl;
        ros::spinOnce(); //callback
    }
};

int main(int argc, char **argv)
{
    if(argc != 2){
        cout << "***error few parameter need datfilename" << endl; 
        return -1;
    }
    
    ros::init(argc, argv, "test_button_state");
    cout << "Initializing node... " << endl;
    
    Read_node read_node;    
    strcpy(read_node.ns, argv[1]);
    read_node.move_arm();
    
    return 0;
}
