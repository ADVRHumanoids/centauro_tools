#include <ros/ros.h>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <sensor_msgs/Joy.h>

using XBot::Logger;

const double RADIUS = 0.075;

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy, 
                  Eigen::Vector3d& ref_twist);

int main(int argc, char ** argv)
{
    /* Init ROS */
    ros::init(argc, argv, "demo_joy_control");
    ros::NodeHandle nh;
    
    /* Get xbot robot and model */
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
    
    auto robot = XBot::RobotInterface::getRobot(path_to_config_file);
    auto model = XBot::ModelInterface::getModel(path_to_config_file);
    
    robot->sense();
    model->syncFrom(*robot);
    
    Eigen::VectorXd q0;
    model->getJointPosition(q0);
    
    Logger::info() << "Initial q: " << q0.transpose() << Logger::endl();
    
    /* Logger */
    auto logger = XBot::MatLogger::getLogger("/tmp/simple_wheeled_motion");
    
    
    /* Joystick communication */
    Eigen::Vector3d ref_twist;
    ref_twist.setZero();
    
    auto joy_sub_callback = std::bind(joy_callback, std::placeholders::_1, 
                                             std::ref(ref_twist)
                                            );
    
    ros::Subscriber joystick_feedback = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joy_sub_callback);
    
    
    /* Define wheels */
    std::vector<std::string> wheels = {"wheel_1",
                                       "wheel_2", 
                                       "wheel_3", 
                                       "wheel_4"};
                                       
    std::vector<std::string> wheels_parent = {"ankle2_1",
                                       "ankle2_2", 
                                       "ankle2_3", 
                                       "ankle2_4"};
                                       
    std::vector<std::string> wheel_steering_joints = {"ankle_yaw_1",
                                       "ankle_yaw_2", 
                                       "ankle_yaw_3", 
                                       "ankle_yaw_4"};
                                       
    std::vector<std::string> wheels_spinning_joints = {"j_wheel_1",
                                       "j_wheel_2", 
                                       "j_wheel_3", 
                                       "j_wheel_4"};
                                       
    std::string waist = "pelvis";
    
    /* Local spinning axes */
    std::vector<Eigen::Vector3d> wheel_spinning_axes;
    std::vector<Eigen::Vector3d> world_steering_axes;
    
    for(auto w: wheels){
        auto axis = model->getUrdf().getLink(w)->parent_joint->axis;
        wheel_spinning_axes.emplace_back(axis.x, axis.y, axis.z);
        
        Logger::info() << "Spinning axis: " << wheel_spinning_axes.back().transpose() << Logger::endl();
    }
    
    for(int i = 0; i < wheels.size(); i++){
        auto axis = model->getUrdf().getJoint(wheel_steering_joints[i])->axis;
        world_steering_axes.emplace_back(axis.x, axis.y, axis.z);
        Eigen::Matrix3d w_R_parent;
        model->getOrientation(wheels_parent[i], w_R_parent);
        world_steering_axes.back() = w_R_parent * world_steering_axes.back();
        Logger::info() << "Steering axis: " << world_steering_axes.back().transpose() << Logger::endl();
    }
    
    
    
    std::map<std::string, int> steering_idx;
    
    for(auto w: wheel_steering_joints)
    {
        steering_idx[w] = model->getDofIndex(w);
    }
    
    /* Wheels to zero */
    Logger::info("Setting wheels orientation to zero...\n");
    XBot::JointNameMap steering_ref;
    
    {
        ros::Rate loop_rate(100);
        for(int i = 0; i < 100; i++)
        {
            for(auto w: wheel_steering_joints)
            {
                steering_ref[w] = q0(steering_idx.at(w)) * (99.0-i)/99.0;
            }
            
            robot->setPositionReference(steering_ref);
            robot->move();
            
            loop_rate.sleep();
        }
        
        /* Set model wheels to zero */
        model->setJointPosition(steering_ref);
        model->update();
        
    }
    
    Logger::success("Wheels orientation is zero! \n");
    
    /* Filter */
    ros::Rate loop_rate(100);
    
    XBot::Utils::SecondOrderFilter<Eigen::VectorXd> q_filter;
    q_filter.reset(q0);
    q_filter.setOmega(6.0);
    q_filter.setTimeStep(loop_rate.expectedCycleTime().toSec());
    q_filter.setDamping(1.0);
    
    /* Looping */
    Logger::info("Started looping...\n");
    
    
    while(ros::ok())
    {
        ros::spinOnce();
        logger->add("v_ref", ref_twist);
        
        /* Current model q */
        model->getJointPosition(q0);
        
        /* Compute reference velocities */
        Eigen::Vector3d omega;
        omega << 0.0, 0.0, ref_twist.z();
        
        Eigen::Vector3d v_waist;
        v_waist << ref_twist.x(), ref_twist.y(), 0.0;
        
        /* Compute wheels linear velocity */
        std::vector<Eigen::Vector3d> v_wheels;
        for(auto w: wheels)
        {
            Eigen::Affine3d waist_T_wheel;
            model->getPose(w, waist, waist_T_wheel);
            v_wheels.push_back( v_waist + XBot::Utils::skewSymmetricMatrix(omega)*(waist_T_wheel.translation()) );
            
            logger->add("vel_" + w, v_wheels.back());
        }
        
        /* Compute yaw angles */
        for(int i = 0; i < wheel_steering_joints.size(); i++)
        {
            double angle_1 = std::atan2(v_wheels[i].y(), v_wheels[i].x()) * world_steering_axes
            [i].z();
            
            double current_angle = q0(steering_idx.at(wheel_steering_joints[i]));
            
            double angle_2 = angle_1 > 0 ? angle_1 - M_PI : angle_1 + M_PI;
            
            double angle_close = std::fabs(angle_1 - current_angle) <= std::fabs(angle_2 - current_angle) ? angle_1 : angle_2;
            
            if(model->getJointByName(wheel_steering_joints[i])->checkJointLimits(angle_close))
            {
                steering_ref[wheel_steering_joints[i]] = angle_close;
            }
            else if(model->getJointByName(wheel_steering_joints[i])->checkJointLimits(angle_1))
            {
                steering_ref[wheel_steering_joints[i]] = angle_1;
            }
            else if(model->getJointByName(wheel_steering_joints[i])->checkJointLimits(angle_2))
            {
                steering_ref[wheel_steering_joints[i]] = angle_2;
            }
            
            logger->add("angle1_" + std::to_string(i), angle_1);
            logger->add("angle2_" + std::to_string(i), angle_2);
            logger->add("angleClose_" + std::to_string(i), angle_close);
            logger->add("angleCurr_" + std::to_string(i), current_angle);
            logger->add("angleRef_" + std::to_string(i), steering_ref[wheel_steering_joints[i]]);
            
        }
        
        /* Compute wheels spinning velocity */
        XBot::JointNameMap wheel_vel_ref;
        for(int i = 0; i < wheels.size(); i++)
        {
            Eigen::Matrix3d w_R_wheel;
            model->getOrientation(wheels[i], w_R_wheel);
            
            Eigen::Vector3d world_spinnin_axis = w_R_wheel * wheel_spinning_axes[i];
            world_spinnin_axis /= world_spinnin_axis.norm();
            Eigen::Vector3d plane_normal(0,0,1);
            
            Eigen::Vector3d a = RADIUS * XBot::Utils::skewSymmetricMatrix(world_spinnin_axis) * plane_normal;
            Eigen::Vector3d b = v_wheels[i];
            
            logger->add("axis_" + wheels[i], world_spinnin_axis);
            logger->add("a_" + wheels[i], a);
            logger->add("b_" + wheels[i], b);
            
            double qdot = b.dot(a)/a.dot(a);
            
            logger->add("qdot_" + wheels[i], qdot);
            
            wheel_vel_ref[wheels_spinning_joints[i]] = qdot;
            
        }
        
        /* Set new yaw angles and spinning vels to model */
        model->setJointVelocity(wheel_vel_ref);
        
        Eigen::VectorXd dq;
        model->mapToEigen(wheel_vel_ref, dq);
        
        q0 += loop_rate.expectedCycleTime().toSec() * dq;
        model->mapToEigen(steering_ref, q0);
        
        model->setJointPosition(q_filter.process(q0));
        
        logger->add("q_ref", q_filter.getOutput());
        
        model->update();
        
        /* Send commands to robot */
        robot->setReferenceFrom(*model);
        robot->move();
        
        loop_rate.sleep();
        
        
    }
    
    logger->flush();
    
    
}



void joy_callback(const sensor_msgs::JoyConstPtr& msg, Eigen::Vector3d& ref_twist)
{
    double v_max = 0.15;
    double thetadot_max = 1.0;
    
    int fwd_bck = 7;
    int l_r = 6;
    int up_down = 3;
    int yaw = 2;

    ref_twist[0] = v_max * msg->axes[fwd_bck];
    ref_twist[1] = v_max * msg->axes[l_r];
    ref_twist[2] = v_max * msg->axes[up_down];
    
    ref_twist[5] = thetadot_max * msg->axes[yaw];
}
