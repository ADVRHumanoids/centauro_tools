#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>

std::vector<Eigen::Vector3d> g_forces;

void on_force_recv(const geometry_msgs::WrenchStampedConstPtr& msg, int i)
{
    g_forces[i] << msg->wrench.force.x,
                   msg->wrench.force.y,
                   msg->wrench.force.z;
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "centauro_zmp_estimation");
    ros::NodeHandle nh("cartesian");
    ros::NodeHandle nh_priv("~");
    
    nh_priv.setParam("is_model_floating_base", true);
    nh_priv.setParam("model_type", "RBDL");
    auto cfg = XBot::ConfigOptionsFromParamServer();
    auto robot = XBot::RobotInterface::getRobot(cfg);
    
    std::vector<std::string> wheel_names = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    std::vector<ros::Subscriber> subs;
    
    for(int i = 0; i < wheel_names.size(); i++)
    {
        auto sub = nh.subscribe<geometry_msgs::WrenchStamped>("force_estimation/" + wheel_names[i],
                                                              1, 
                                                              boost::bind(on_force_recv, _1, i)
        );
        
        subs.push_back(sub);
    }
    
    auto zmp_pub = nh.advertise<geometry_msgs::PointStamped>("zmp_estimation", 1);
    auto com_pub = nh.advertise<geometry_msgs::PointStamped>("com_projection", 1);
    
    g_forces.assign(4, Eigen::Vector3d(0, 0, 250.0));
    
    double rate = nh_priv.param("rate", 30.0);
    
    ros::Rate loop_rate(rate);
    
    while(ros::ok())
    {
        ros::spinOnce();
        robot->sense();
        
        
        /* Compute COP */
        double fz = 0.0;
        double ground_z = 0.0;
        Eigen::Vector3d zmp(0, 0, 0);
        
        for(int i = 0; i < wheel_names.size(); i++)
        {
            Eigen::Affine3d w_T_f;
            robot->model().getPose(wheel_names[i], "pelvis", w_T_f);
            
            double fz_i = w_T_f.linear().row(2).dot(g_forces[i]);
            
            zmp += w_T_f.translation() * fz_i;
            
            fz += fz_i;
        }
        
        zmp /= fz;
        
        {
            geometry_msgs::PointStamped msg;
            msg.header.frame_id = "pelvis";
            msg.header.stamp = ros::Time::now();
            msg.point.x = zmp.x();
            msg.point.y = zmp.y();
            msg.point.z = zmp.z();
            
            zmp_pub.publish(msg);
        }
        
        /* Compute COM */
        Eigen::Vector3d com;
        Eigen::Affine3d w_T_pelvis;
        robot->model().getCOM(com);
        robot->model().getPose("pelvis", w_T_pelvis);
        com = w_T_pelvis.inverse() * com;
        
        {
            geometry_msgs::PointStamped msg;
            msg.header.frame_id = "pelvis";
            msg.header.stamp = ros::Time::now();
            msg.point.x = com.x();
            msg.point.y = com.y();
            msg.point.z = zmp.z();
            
            com_pub.publish(msg);
        }
        
        loop_rate.sleep();
    }
}