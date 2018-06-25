#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Dense>
#include <XBotInterface/Utils.h>
#include <XBotInterface/MatLogger.hpp>

Eigen::Vector2d ref(.0, .0);
Eigen::Vector2d state_xy(.0, .0);
double state_theta = .0;
bool state_received = false;

void ref_callback(const geometry_msgs::PoseStampedConstPtr& msg);
void state_callback(const geometry_msgs::PoseStampedConstPtr& msg);

int main(int argc, char **argv)
{
    auto logger = XBot::MatLogger::getLogger("/tmp/unicycle_log");
    ros::init(argc, argv, "unicycle_tracking");
    ros::NodeHandle nh, nhpriv("~");
    
    ros::Subscriber state_sub = nh.subscribe("pelvis/state", 1, state_callback);
    ros::Subscriber ref_sub = nh.subscribe("unicycle/reference", 1, ref_callback);
    ros::Publisher ref_pub = nh.advertise<geometry_msgs::TwistStamped>("pelvis/velocity_reference", 1);
    
    double b = nhpriv.param("ref_point_distance", 1.0);
    double Kfb = nhpriv.param("gain", 1.0);
    double filter_cutoff = nhpriv.param("filter_cutoff", 2.0);
    double u_max = nhpriv.param("u_max", 1.0);
    ros::Rate loop_rate(100);
    double dt = loop_rate.expectedCycleTime().toSec();
    
    while(!state_received)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    state_sub.shutdown();
    
    double ctheta = std::cos(state_theta);
    double stheta = std::sin(state_theta);
    Eigen::Vector2d i_vec(ctheta, stheta);
    ref = state_xy + b*i_vec;
    
    XBot::Utils::SecondOrderFilter<Eigen::Vector2d> filt;
    filt.setDamping(1.0);
    filt.setOmega(filter_cutoff);
    filt.setTimeStep(loop_rate.expectedCycleTime().toSec());
    filt.reset(ref);
    
    Eigen::Vector2d ref_limited = ref;
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        ref_limited = ref_limited.array() + ((ref - ref_limited)/dt).array().max(-0.3).min(0.3)*dt;
        
        double ctheta = std::cos(state_theta);
        double stheta = std::sin(state_theta);
        Eigen::Vector2d i_vec(ctheta, stheta);
        Eigen::Vector2d y = state_xy + b*i_vec;
        Eigen::Vector2d u = Kfb*(filt.process(ref_limited) - y);
        
        double theta_dot = Eigen::Vector2d(-stheta, ctheta).dot(u)/b;
        
        Eigen::Matrix2d Tinv;
        Tinv <<    ctheta,   stheta,
                -stheta/b, ctheta/b;
        
        double v_des = (Tinv.topRows(1)*u).value();
        double thetadot_des = (Tinv.bottomRows(1)*u).value();
        
        geometry_msgs::TwistStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.twist.linear.x = v_des * ctheta;
        msg.twist.linear.y = v_des * stheta;
        msg.twist.angular.z = thetadot_des;
        
        state_xy += i_vec*v_des*dt;
        state_theta += thetadot_des*dt;
        
        ref_pub.publish(msg);
        
        logger->add("state_xy", state_xy);
        logger->add("ref", ref);
        logger->add("ref_limited", ref_limited);
        logger->add("ref_filt", filt.getOutput());
        logger->add("v_des", v_des);
        logger->add("thetadot_des", thetadot_des);
        
        
        loop_rate.sleep();
    }
    
    logger->flush();
}

void ref_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    ref << msg->pose.position.x, msg->pose.position.y;
}

void state_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    Eigen::Affine3d w_T_pelvis;
    tf::poseMsgToEigen(msg->pose, w_T_pelvis);
    
    state_xy = w_T_pelvis.translation().head<2>();
    state_theta = std::atan2(w_T_pelvis.linear()(1,0), w_T_pelvis.linear()(0,0));
    state_received = true;
}
