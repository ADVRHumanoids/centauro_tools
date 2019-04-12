#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <ros/ros.h>
#include <cartesian_interface/ros/RosImpl.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_orientation_control");
    ros::NodeHandle nh("~");
    
    std::vector<std::string> wheels = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    
    auto opt = XBot::ConfigOptionsFromParamServer();
    opt.set_parameter<std::string>("framework", "ROS");
    auto robot = XBot::RobotInterface::getRobot(opt);
    robot->sense();
    
    XBot::ImuSensor::ConstPtr imu = robot->getImu().begin()->second;
    
    if(!imu)
    {
        throw std::runtime_error("unable to retrieve imu");
    }
    
    double gain = nh.param("gain", 1.0);

    // set initial orientation as reference
    Eigen::Matrix3d R0;
    imu->getOrientation(R0);
    
    // go to velocity control
    XBot::Cartesian::RosImpl ci;
    ci.setControlMode("pelvis", XBot::Cartesian::ControlType::Velocity);
    
    // set control rate
    ros::Rate r(200);
    
    while(ros::ok())
    {
        // get imu readings
        robot->sense();
        
        // get imu orientation 
        Eigen::Matrix3d R;
        imu->getOrientation(R);
        
        // compute orientation error (quaterion-based)
        Eigen::Vector3d e_o;
        XBot::Utils::computeOrientationError(R0, R, e_o);
        
        // set omega = k * e_o
        Eigen::Vector6d vref;
        vref.setZero();
        vref.tail<3>() = gain * e_o;
        vref[5] = 0.0; // don't control the yaw motion
        
        // compute com and pelvis position
        Eigen::Vector3d com;
        robot->model().getCOM(com);
        Eigen::Vector3d pelvis_pos;
        robot->model().getPointPosition("pelvis", Eigen::Vector3d::Zero(), pelvis_pos);
        
        // compute contact plane estimate
        Eigen::Matrix<double, 4, 3> A;
        Eigen::Vector4d b;
        
        A.setZero();
        b.setZero();
        
        int n_stance = 0;
        for(auto w : wheels)
        {
            Eigen::Vector3d w_t_foot;
            
            robot->model().getPointPosition(w, Eigen::Vector3d::Zero(), w_t_foot);
            
            A.row(n_stance) << w_t_foot.x(), 
                               w_t_foot.y(), 
                               1;
                        
            b(n_stance) = w_t_foot.z();
            
            n_stance++;
        }
        
        
        Eigen::Vector3d a = A.fullPivHouseholderQr().solve(b);
        Eigen::Vector3d n(a[0], a[1], -1.);
        double d = -a[2];
        
        // compute com projection to the ground plane
        // n'(p - gt) + d = 0 -> t = (n'p + d)/n'g
        // p - gt = p - g(n'p + d)/n'g
        Eigen::Vector3d g = R.row(2).transpose();
        Eigen::Vector3d com_proj;
        com_proj = com - g*(n.dot(com) - d)/n.dot(g);
        
        // get vector from com projection to pelvis
        Eigen::Vector3d pelvis_t_com_proj = pelvis_pos - com_proj;
        
        // set v = omega x r (we rotate the pelvis about the com projection point)
        vref.head<3>() = vref.tail<3>().cross(pelvis_t_com_proj);
        
        // send command to cartesio
        ci.setVelocityReference("pelvis", vref);
        
        // sleep
        r.sleep();
    }
    
}
