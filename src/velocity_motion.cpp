#include <ros/ros.h>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using XBot::Logger;

const double RADIUS = 0.075;


void joy_callback(const sensor_msgs::Joy::ConstPtr& joy, 
                  Eigen::Vector3d& ref_twist);

int main(int argc, char ** argv)
{
    /* Init ROS */
    ros::init(argc, argv, "demo_joy_control");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/xbotcore/cartesian/pelvis/velocity_reference", 1);
    
    /* Joystick communication */
    Eigen::Vector3d ref_twist;
    ref_twist.setZero();
    
    auto joy_sub_callback = std::bind(joy_callback, std::placeholders::_1, 
                                             std::ref(ref_twist)
                                            );
    
    ros::Subscriber joystick_feedback = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joy_sub_callback);
    
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
    
    
    
    tf::TransformListener listener;
    
    /* Looping */
    Logger::info("Started looping...\n");
    
    geometry_msgs::TwistStamped msg;
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        
        tf::StampedTransform transform;
        Eigen::Affine3d T;
        try{
        listener.lookupTransform("/pelvis", "ci/world_odom",  
                                ros::Time(0), transform);
        
        tf::transformTFToEigen(transform, T);
        
        //std::cout<<"T pelvis in world: /n"<<T.matrix()<<std::endl;
        
        Eigen::Vector2d ref_xy_twist;
        ref_xy_twist << ref_twist[0], ref_twist[1];
        ref_xy_twist = (T.inverse()).matrix().block(0,0,2,2)*ref_xy_twist;
        
       msg.twist.linear.x = ref_xy_twist[0];
       msg.twist.linear.y = ref_xy_twist[1];
       msg.twist.linear.z = ref_twist[2];
       msg.twist.angular.z = ref_twist[5];
       pub.publish(msg);
        
        
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }
        
        
        
    }
    
}



void joy_callback(const sensor_msgs::JoyConstPtr& msg, Eigen::Vector3d& ref_twist)
{
    double v_max = 0.15;
    double zv_max = 0.05;
    double thetadot_max = 0.2;
    
    int x = 1;
    int y = 0;
    int z = 7;
    int yaw = 2;

    ref_twist[0] = v_max * msg->axes[x];
    ref_twist[1] = v_max * msg->axes[y];
    
    
    ref_twist[5] = thetadot_max * msg->axes[yaw];
    
    int val = msg->buttons[7];
    if (val == 0) ref_twist[2]=0.0; else ref_twist[2] = zv_max * msg->axes[z];
    
    std::cout<<"x: "<<ref_twist[0] << " y: "<<ref_twist[1]<<" z: "<<ref_twist[2]<<" yaw: "<<ref_twist[5]<<std::endl;
    
}
