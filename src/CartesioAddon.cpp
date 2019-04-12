#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/open_sot/TaskInterface.h>
#include <OpenSoT/tasks/velocity/CentauroAnkleSteering.h>
#include <OpenSoT/tasks/velocity/PureRolling.h>
#include <boost/make_shared.hpp>

using namespace XBot::Cartesian;

struct CentauroSteeringTask : public TaskDescription
{
    std::string wheel_name;
    double max_steering_speed;
    
    CentauroSteeringTask();
};

CentauroSteeringTask::CentauroSteeringTask():
    TaskDescription(TaskInterface::None, 
                    "CentauroSteering", 
                    1)
{
}

extern "C" TaskDescription * CentauroSteeringTaskDescriptionFactory(YAML::Node task_node, 
                                                     XBot::ModelInterface::ConstPtr model)
{
    CentauroSteeringTask * task_desc = new CentauroSteeringTask;
    
    task_desc->wheel_name = task_node["wheel_name"].as<std::string>();
    
    if(task_node["max_steering_speed"])
    {
        task_desc->max_steering_speed = task_node["max_steering_speed"].as<double>();
    }
    else
    {
        task_desc->max_steering_speed = 3.0;
    }
    
    return task_desc;
    
}

class CentauroSteeringOpenSot : public SoT::TaskInterface
{
    
public:
    
    CentauroSteeringOpenSot(TaskDescription::Ptr task_desc, 
                            XBot::ModelInterface::ConstPtr model):
        SoT::TaskInterface(task_desc, model)
    {
        
        auto steering_desc = std::dynamic_pointer_cast<CentauroSteeringTask>(task_desc);
        
        _task = boost::make_shared<OpenSoT::tasks::velocity::CentauroAnkleSteering>
                                (steering_desc->wheel_name,
                                 model,
                                 0.01,
                                 steering_desc->max_steering_speed
                                );
    }
    
    SoT::TaskPtr getTaskPtr() const override
    {
        return _task;
    }
    
    bool setBaseLink(const std::string & ee_name, const std::string & base_link) override
    {
        return false;
    }
    
    bool setControlMode(const std::string & ee_name, ControlType ctrl_mode) override
    {
        return false;
    }
    
    bool update(const CartesianInterface * ci, double time, double period) override
    {
        return true;
    }
    
private:
    
    SoT::TaskPtr _task;
    
    
};


extern "C" SoT::TaskInterface * CentauroSteeringOpenSotTaskFactory(TaskDescription::Ptr task_desc, 
                                                            XBot::ModelInterface::ConstPtr model)
{
    return new CentauroSteeringOpenSot(task_desc, model);
}


struct WheelRollingTask : public TaskDescription
{
    std::string wheel_name;
    double wheel_radius;
    
    WheelRollingTask();
};

WheelRollingTask::WheelRollingTask():
    TaskDescription(TaskInterface::None, 
                    "WheelRolling", 
                    2)
{
}

extern "C" TaskDescription * WheelRollingTaskDescriptionFactory(YAML::Node task_node, 
                                                     XBot::ModelInterface::ConstPtr model)
{
    WheelRollingTask * task_desc = new WheelRollingTask;
    
    task_desc->wheel_name = task_node["wheel_name"].as<std::string>();
    
    task_desc->wheel_radius = task_node["wheel_radius"].as<double>();
    
    return task_desc;
    
}

class WheelRollingOpenSot : public SoT::TaskInterface
{
    
public:
    
    WheelRollingOpenSot(TaskDescription::Ptr task_desc, 
                            XBot::ModelInterface::ConstPtr model):
        SoT::TaskInterface(task_desc, model)
    {
        
        auto rolling_desc = std::dynamic_pointer_cast<WheelRollingTask>(task_desc);
        
        _task = boost::make_shared<OpenSoT::tasks::velocity::PureRollingPosition>
                                (rolling_desc->wheel_name,
                                 rolling_desc->wheel_radius,
                                 *model
                                );
    }
    
    SoT::TaskPtr getTaskPtr() const override
    {
        return _task;
    }
    
    bool setBaseLink(const std::string & ee_name, const std::string & base_link) override
    {
        return false;
    }
    
    bool setControlMode(const std::string & ee_name, ControlType ctrl_mode) override
    {
        return false;
    }
    
    bool update(const CartesianInterface * ci, double time, double period) override
    {
        return true;
    }
    
private:
    
    SoT::TaskPtr _task;
    
    
};


extern "C" SoT::TaskInterface * WheelRollingOpenSotTaskFactory(TaskDescription::Ptr task_desc, 
                                                            XBot::ModelInterface::ConstPtr model)
{
    return new WheelRollingOpenSot(task_desc, model);
}
