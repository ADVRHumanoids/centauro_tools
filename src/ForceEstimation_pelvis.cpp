#include <ros/ros.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RobotInterface.h>
#include <vector>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/Trigger.h>

Eigen::VectorXd g_tau_offset;
XBot::ModelInterface::Ptr g_model;
XBot::RobotInterface::Ptr g_robot;

bool compute_tau_offset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse&  res);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "centauro_force_estimation_pelvis");
    ros::NodeHandle nh, nh_priv("~");
    auto xbot_cfg = XBot::ConfigOptions::FromConfigFile(XBot::Utils::getXBotConfig());
    
    auto logger = XBot::MatLogger::getLogger("/tmp/centauro_force_estimation_log");
    auto robot = g_robot = XBot::RobotInterface::getRobot(xbot_cfg);
    auto model = g_model = XBot::ModelInterface::getModel(xbot_cfg);
    auto imu = robot->getImu().at("imu_link");
    
    
    
    std::vector<std::string> wheels = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    std::vector<ros::Publisher> pubs;
    
    for(int i = 0; i < 4; i++)
    {
        auto pub = nh.advertise<geometry_msgs::WrenchStamped>("estimated_force_wheel" + std::to_string(i+1), 1);
        pubs.push_back(pub);
    }
    
    auto pub = nh.advertise<geometry_msgs::WrenchStamped>("estimated_wrench_pelvis", 1);
    pubs.push_back(pub);
    
    auto srv = nh.advertiseService("offset_compensation", compute_tau_offset);
    
    ros::Rate loop_rate(100);
    
    Eigen::VectorXd tau, nl;
    Eigen::MatrixXd J;
    Eigen::MatrixXd J_w1, J_w2, J_w3, J_w4, J_pelvis, A;
    g_tau_offset.setZero(model->getJointNum());
    
    while(ros::ok())
    {
        robot->sense();
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        model->setFloatingBaseState(imu);
        model->update();
        
        model->getJointEffort(tau);
        model->computeNonlinearTerm(nl);
        

//         for(int i = 0; i < 4; i++)
//         {
//             Eigen::Matrix3d w_R_wheel;
//             model->getOrientation(wheels[i], w_R_wheel);
//             model->getJacobian(wheels[i], J);
//             
//             int start_dof = model->getDofIndex("hip_yaw_" + std::to_string(i+1));
//             auto Jt = J.topRows<3>().middleCols(start_dof, 5).transpose();
//             
//             
//             Eigen::Vector3d f_est = Jt.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve((nl - tau).segment(start_dof, 5));
//             
//             logger->add("f_est_" + std::to_string(i+1), f_est);
//             logger->add("tau_residual_" + std::to_string(i+1), (nl - tau).segment(start_dof, 5));
//             
//             f_est = w_R_wheel.transpose() * f_est;
//             
//             geometry_msgs::WrenchStamped msg;
//             msg.header.frame_id = wheels[i];
//             msg.header.stamp = ros::Time::now();
//             
//             msg.wrench.force.x = f_est.x();
//             msg.wrench.force.y = f_est.y();
//             msg.wrench.force.z = f_est.z();
//             
//             pubs[i].publish(msg);
//             
//         }

       Eigen::Matrix3d w_R_wheel1, w_R_wheel2, w_R_wheel3, w_R_wheel4, w_R_pelvis;
       
       model->getOrientation("pelvis", w_R_pelvis);
       model->getJacobian("pelvis", J_pelvis);
       
       model->getOrientation(wheels[0], w_R_wheel1);
       model->getJacobian(wheels[0], J_w1);
       
       model->getOrientation(wheels[1], w_R_wheel2);
       model->getJacobian(wheels[1], J_w2);
       
       model->getOrientation(wheels[2], w_R_wheel3);
       model->getJacobian(wheels[2], J_w3);
       
       model->getOrientation(wheels[3], w_R_wheel4);
       model->getJacobian(wheels[3], J_w4);
       
       A.setZero(18, model->getJointNum());
       A << J_pelvis, J_w1.topRows<3>(), J_w2.topRows<3>(), J_w3.topRows<3>(), J_w4.topRows<3>();
      
       auto At = A.transpose();
       
       Eigen::VectorXd f_est = At.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve((nl - tau - g_tau_offset));
       
       Eigen::Vector3d f_est_pelvis = f_est.head(3);
       Eigen::Vector3d torque_est_pelvis = f_est.segment<3>(3);
       
       logger->add("wrench_est_pelvis", f_est.head(6));
       
            
        for(int i = 0; i < 4; i++)
        {
            Eigen::Matrix3d w_R_wheel;
            model->getOrientation(wheels[i], w_R_wheel);
            
            Eigen::Vector3d f_est_wheel = f_est.segment<3>(6+(3*i));
            
            logger->add("f_est_wheel" + std::to_string(i+1), f_est_wheel);
            
            f_est_wheel = w_R_wheel.transpose() * f_est_wheel;
            
            geometry_msgs::WrenchStamped msg;
            msg.header.frame_id = wheels[i];
            msg.header.stamp = ros::Time::now();
            
            msg.wrench.force.x = f_est_wheel.x();
            msg.wrench.force.y = f_est_wheel.y();
            msg.wrench.force.z = f_est_wheel.z();
            
            pubs[i].publish(msg);
            
        }
        
        f_est_pelvis = w_R_pelvis.transpose() * f_est_pelvis;
        torque_est_pelvis = w_R_pelvis.transpose() * torque_est_pelvis;
        
        geometry_msgs::WrenchStamped msg;
        msg.header.frame_id = "pelvis";
        msg.header.stamp = ros::Time::now();
            
        msg.wrench.force.x = f_est_pelvis.x();
        msg.wrench.force.y = f_est_pelvis.y();
        msg.wrench.force.z = f_est_pelvis.z();
        msg.wrench.torque.x = torque_est_pelvis.x();
        msg.wrench.torque.y = torque_est_pelvis.y();
        msg.wrench.torque.z = torque_est_pelvis.z();
        
        pubs[4].publish(msg);
            

        loop_rate.sleep();
    }
    
    logger->flush();
    
}


bool compute_tau_offset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    auto imu = g_robot->getImu().at("imu_link");
    
    g_tau_offset.setZero(g_model->getJointNum());
    Eigen::VectorXd tau, nl;
    
    const int ITER = 100;
    
    for(int i = 0; i < ITER; i++)
    {
        g_robot->sense();
        g_model->syncFrom(*g_robot, XBot::Sync::All, XBot::Sync::MotorSide);
        g_model->setFloatingBaseState(imu);
        g_model->update();
        
        g_model->getJointEffort(tau);
        g_model->computeNonlinearTerm(nl);
        
        g_tau_offset += (nl - tau);
        
    }
    
    g_tau_offset /= ITER;
    
    g_tau_offset.head<6>().setZero();
    
    return true;
    
}
