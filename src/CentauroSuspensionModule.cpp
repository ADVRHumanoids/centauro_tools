#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <OpenSoT/utils/ForceOptimization.h>

void on_joint_ref_recv(const sensor_msgs::JointStateConstPtr& msg, 
                       XBot::JointNameMap * jmap);


int main(int argc, char ** argv)
{
    
    ros::init(argc, argv, "centauro_suspensions");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    auto cfg = XBot::ConfigOptionsFromParamServer();
    auto robot = XBot::RobotInterface::getRobot(cfg);
    auto model = XBot::ModelInterface::getModel(cfg);
    
    XBot::JointNameMap jmap;
    robot->sense();
    robot->getMotorPosition(jmap);
    robot->setControlMode(XBot::ControlMode::Effort());
    
    auto jref_sub = nh.subscribe<sensor_msgs::JointState>("cartesian/solution", 1, 
                                 std::bind(on_joint_ref_recv, std::placeholders::_1, &jmap)
                                );
    
    std::vector<std::string> ankles = {"ankle1_1", 
                                       "ankle1_2", 
                                       "ankle1_3", 
                                       "ankle1_4"};
                                       
    std::vector<std::string> contacts = {"wheel_1", 
                                       "wheel_2", 
                                       "wheel_3", 
                                       "wheel_4"};
                                       
    Eigen::Matrix3d K;
    K.setZero();
    
    K.diagonal() << nh_priv.param("kx", 300.0), 
                    nh_priv.param("ky", 300.0),
                    nh_priv.param("kz", 300.0);
    
    ros::Rate loop_rate(200);
    
    Eigen::MatrixXd J;
    Eigen::VectorXd tau;
    std::vector<Eigen::Vector6d> forces(4);
    
    XBot::ModelInterface::Ptr fb_model_ptr(&(robot->model()), [](XBot::ModelInterface *){});
    OpenSoT::utils::ForceOptimization contact_f_opt(fb_model_ptr, contacts, false);
    
    while(ros::ok())
    {
        ros::spinOnce();
        robot->sense();
        
        model->setJointPosition(jmap);
        model->update();
        
        robot->model().computeGravityCompensation(tau);
        
        contact_f_opt.compute(tau, forces, tau);
        
        for(int i = 0; i < ankles.size(); i++)
        {
            Eigen::Affine3d T_ref;
            model->getPose(ankles[i], "pelvis", T_ref);
            
            Eigen::Affine3d T_meas;
            robot->model().getPose(ankles[i], "pelvis", T_meas);
            
            Eigen::Vector3d p_err = T_ref.translation() - T_meas.translation();
            Eigen::Vector3d F = K*p_err;
            
            robot->model().getRelativeJacobian(ankles[i], "pelvis", J);
            
            tau += J.transpose()*F;
                        
        }
        
        robot->setEffortReference(tau.tail(robot->getJointNum()));
        robot->move();
        
        loop_rate.sleep();
    }
    
    
}

void on_joint_ref_recv(const sensor_msgs::JointStateConstPtr& msg, XBot::JointNameMap* jmap)
{
    for(int i = 0; i < msg->name.size(); i++)
    {
        (*jmap)[msg->name[i]] = msg->position[i];
    }
}
