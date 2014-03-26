#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;

class Write_node{
    public:
    double joint_angles[17];
    std::ofstream ofs;
    
    Write_node(){
        ofs.open( "/home/kaminuno/rw.dat", std::ios::out | std::ios::app );
        if (!ofs){
                cout << "***error  Can not open the file\n";
                exit(1);
        }
    }

    void on_joint_states(const sensor_msgs::JointState msg){
        for(int i=0; i<17; i++){
            joint_angles[i] = msg.position[i];
        }
    }
    
    void on_OKButton_states(const baxter_core_msgs::DigitalIOState OKButton_states){
        if(OKButton_states.state == OKButton_states.PRESSED){
            for(int i=0; i<17; i++){
                ofs << joint_angles[i];
                cout << joint_angles[i];
                if(i!=16){
                     ofs << " ";
                     cout << " ";
                }
            }
            ofs << endl;
            cout << endl;
            sleep(1);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_button_state");
    cout << "Initializing node... " << endl;
    Write_node write_node;
    
    //subscriber 
    ros::NodeHandle n;   
    ros::Subscriber joint_state_topic = n.subscribe("robot/joint_states", 10, &Write_node::on_joint_states , &write_node);
    ros::Subscriber cuff_left_OKButton_state_topic = n.subscribe("robot/digital_io/left_lower_button/state", 10, &Write_node::on_OKButton_states, &write_node);
    ros::Subscriber cuff_right_OKButton_state_topic = n.subscribe("robot/digital_io/right_lower_button/state", 10, &Write_node::on_OKButton_states, &write_node);   
    ros::spin(); //callback
    return 0;
}
