#include <ros/ros.h>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <cartesian_interface/SetTaskInfo.h>

using XBot::Logger;

const double RADIUS = 0.075;


void joy_callback(const sensor_msgs::Joy::ConstPtr& joy, 
                  Eigen::Vector3d& ref_twist);
int state = 0;

std::map<int,ros::Publisher> topic_map;
bool mode[4];
int start = 0;

int main(int argc, char ** argv)
{
    /* Init ROS */
    ros::init(argc, argv, "demo_joy_control");
    ros::NodeHandle nh;
    
    //ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/xbotcore/cartesian/pelvis/velocity_reference", 1);
    topic_map[0] = nh.advertise<geometry_msgs::TwistStamped>("/xbotcore/cartesian/pelvis/velocity_reference", 1);
    topic_map[1] = nh.advertise<geometry_msgs::TwistStamped>("/xbotcore/cartesian/arm1_8/velocity_reference", 1);
    topic_map[2] = nh.advertise<geometry_msgs::TwistStamped>("/xbotcore/cartesian/arm2_8/velocity_reference", 1);
    

    ros::ServiceClient client1 = nh.serviceClient<cartesian_interface::SetTaskInfo>("/xbotcore/cartesian/pelvis/set_task_properties");
    ros::ServiceClient client2 = nh.serviceClient<cartesian_interface::SetTaskInfo>("/xbotcore/cartesian/arm1_8/set_task_properties");
    ros::ServiceClient client3 = nh.serviceClient<cartesian_interface::SetTaskInfo>("/xbotcore/cartesian/arm2_8/set_task_properties");
    cartesian_interface::SetTaskInfo srv;
    
   
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
        
        if (start ==1){
            
            srv.request.base_link = "world";
            srv.request.control_mode = "Velocity";
            client1.call(srv);
            srv.request.base_link = "pelvis";
            client2.call(srv);
            client3.call(srv);
            
        }
        
        tf::StampedTransform transform;
        Eigen::Affine3d T;
        try{
        listener.lookupTransform("/pelvis", "ci/world_odom",  
                                ros::Time(0), transform);
        
        tf::transformTFToEigen(transform, T);
        
        //std::cout<<"T pelvis in world: /n"<<T.matrix()<<std::endl;
        
        Eigen::Vector2d ref_xy_twist;
        ref_xy_twist << ref_twist[0], ref_twist[1];
        if( state == 0){ 
            ref_xy_twist = (T.inverse()).matrix().block(0,0,2,2)*ref_xy_twist;
        }
        
       msg.twist.linear.x = ref_xy_twist[0];
       msg.twist.linear.y = ref_xy_twist[1];
       msg.twist.linear.z = ref_twist[2];

       msg.twist.angular.x = ref_twist[3];
       msg.twist.angular.y = ref_twist[4];
       msg.twist.angular.z = ref_twist[5];
      
       if( state == 0){           
           topic_map[0].publish(msg);
        }else if(state == 1){
           topic_map[1].publish(msg);
        }else if(state == 2){
           topic_map[2].publish(msg);
        }else if(state == 3){
           topic_map[1].publish(msg);
           topic_map[2].publish(msg);
        }
       
        
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }
        
        
        
    }
    
}



void joy_callback(const sensor_msgs::JoyConstPtr& msg, Eigen::Vector3d& ref_twist)
{
    double v_max = 0.35;//0.15;
    double zv_max = 0.2;//0.05;
    double thetadot_max = 0.8;//0.2;
    
    int x = 1;
    int y = 0;
    int z = 7;
    int yaw_pelvis = 2;
    int pitch = 3;
    int roll = 2;
    int yaw = 6;

   
    
    int ba = msg->buttons[0];
    int bb = msg->buttons[1];
    int bx = msg->buttons[3];
    int by = msg->buttons[4];
    start = msg->buttons[11];
            
    if (by == 1) state = 0;
    if (bx ==1) state = 1;
    if (bb ==1) state = 2;
    if (ba ==1) state = 3;
    
    int val = msg->buttons[7];
    if (state == 0){
    if (val == 0) ref_twist[2]=0.0; else ref_twist[2] = zv_max * msg->axes[z];
        v_max = 0.35;
        zv_max = 0.2;
        thetadot_max = 0.8;
        ref_twist[3] = 0.0;
        ref_twist[4] = 0.0;
        ref_twist[5] = thetadot_max * msg->axes[yaw_pelvis];
        ref_twist[0] = v_max * msg->axes[x];
        ref_twist[1] = v_max * msg->axes[y];
    }else{
        v_max = 0.15;
        zv_max = 0.05;
        thetadot_max = 0.2;
        ref_twist[2] = zv_max * msg->axes[z];
        ref_twist[3] = thetadot_max * msg->axes[roll];
        ref_twist[4] = thetadot_max * msg->axes[pitch];
        ref_twist[5] = thetadot_max * msg->axes[yaw];
        ref_twist[0] = v_max * msg->axes[x];
        ref_twist[1] = v_max * msg->axes[y];
    }
    
    
    
    std::cout<<"x: "<<ref_twist[0] << " y: "<<ref_twist[1]<<" z: "<<ref_twist[2]<<" yaw: "<<ref_twist[5]<<std::endl;
    
}
