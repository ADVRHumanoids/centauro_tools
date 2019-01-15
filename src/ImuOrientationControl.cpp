#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <ros/ros.h>
#include <cartesian_interface/ros/RosImpl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_orientation_control");
    ros::NodeHandle nh("~");
    
    auto opt = XBot::ConfigOptions::FromConfigFile(XBot::Utils::getXBotConfig());
    auto robot = XBot::RobotInterface::getRobot(opt);
    robot->sense();
    
    XBot::ImuSensor::ConstPtr imu = robot->getImu().begin()->second;
    
    if(!imu)
    {
        throw std::runtime_error("unable to retrieve imu");
    }
    
    double gain = nh.param("gain", 0.1);
    
    Eigen::Matrix3d R0;
    imu->getOrientation(R0);
    
    XBot::Cartesian::RosImpl ci;
    ci.loadController("WheeledMotion");
    ci.setControlMode("pelvis", XBot::Cartesian::ControlType::Velocity);
    
    ros::Rate r(200);
    
    while(ros::ok())
    {
        robot->sense();
        
        Eigen::Matrix3d R;
        imu->getOrientation(R);
        
        Eigen::Vector6d vref;
        vref.setZero();
        Eigen::Vector3d e_o;
        XBot::Utils::computeOrientationError(R0, R, e_o);
        vref(0) = 0.1;
        vref.tail<3>() = e_o;
        vref[5] = 0.0;
        
        ci.setVelocityReference("pelvis", vref);
        
        r.sleep();
    }
    
}