#include <ros/ros.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>
#include <vector>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "centauro_force_estimation");
    ros::NodeHandle nh, nh_priv("~");
    auto xbot_cfg = XBot::ConfigOptions::FromConfigFile(XBot::Utils::getXBotConfig());
    
    auto logger = XBot::MatLogger::getLogger("/tmp/centauro_force_estimation_log");
    auto robot = XBot::RobotInterface::getRobot(xbot_cfg);
    auto model = XBot::ModelInterface::getModel(xbot_cfg);
    auto imu = robot->getImu().at("imu_link");
    
    std::vector<std::string> wheels = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    std::vector<ros::Publisher> pubs;
    
    for(int i = 0; i < 4; i++)
    {
        auto pub = nh.advertise<geometry_msgs::WrenchStamped>("estimated_force_" + std::to_string(i+1), 1);
        pubs.push_back(pub);
    }
    
    ros::Rate loop_rate(100);
    
    Eigen::VectorXd tau, nl;
    Eigen::MatrixXd J;
    
    while(ros::ok())
    {
        robot->sense();
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        model->setFloatingBaseState(imu);
        model->update();
        
        model->getJointEffort(tau);
        model->computeNonlinearTerm(nl);
        
        for(int i = 0; i < 4; i++)
        {
            Eigen::Matrix3d w_R_wheel;
            model->getOrientation(wheels[i], w_R_wheel);
            model->getJacobian(wheels[i], J);
            int start_dof = model->getDofIndex("hip_yaw_" + std::to_string(i+1));
            auto Jt = J.topRows<3>().middleCols(start_dof, 5).transpose();
            
            Eigen::Vector3d f_est = Jt.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve((nl - tau).segment(start_dof, 5));
            
            logger->add("f_est_" + std::to_string(i+1), f_est);
            logger->add("tau_residual_" + std::to_string(i+1), (nl - tau).segment(start_dof, 5));
            
            f_est = w_R_wheel.transpose() * f_est;
            
            geometry_msgs::WrenchStamped msg;
            msg.header.frame_id = wheels[i];
            msg.header.stamp = ros::Time::now();
            
            msg.wrench.force.x = f_est.x();
            msg.wrench.force.y = f_est.y();
            msg.wrench.force.z = f_est.z();
            
            pubs[i].publish(msg);
            
        }
        
        
        loop_rate.sleep();
    }
    
    logger->flush();
    
}