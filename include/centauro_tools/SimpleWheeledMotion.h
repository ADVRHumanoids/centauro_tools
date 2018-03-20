#ifndef __CENTAURO_SIMPLE_WHEELED_MOTION_H__
#define __CENTAURO_SIMPLE_WHEELED_MOTION_H__

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/velocity/PureRolling.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <XBotInterface/Utils.h>


namespace XBot { namespace Cartesian {
    
    
    class CustomRelativeCartesian : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>
    {
        
    public:
        
        typedef boost::shared_ptr<CustomRelativeCartesian> Ptr;
        
        CustomRelativeCartesian(const ModelInterface& robot, 
                                std::string distal_link, 
                                std::string base_link);
        
        void setReference(const Eigen::Vector3d& ref);
        
        const std::string& getDistalLink() const;
        
        Eigen::Vector3d getError() const;
        
    private:
      
        virtual void _update(const Eigen::VectorXd& x);
        
        Eigen::MatrixXd _Jbase, _Jdistal;
        const ModelInterface& _robot;
        std::string _base, _distal;
        
        Eigen::Vector3d _ref;
        Eigen::Vector3d _error;
        
        
    };
    
const std::string& CustomRelativeCartesian::getDistalLink() const
{
    return _distal;
}


void CustomRelativeCartesian::setReference(const Eigen::Vector3d& ref)
{
    _ref = ref;
}


CustomRelativeCartesian::CustomRelativeCartesian(const ModelInterface& robot, string distal_link, string base_link): 
    Task< Eigen::MatrixXd, Eigen::VectorXd >("CUSTOM_REL_" + distal_link, robot.getJointNum()),
    _robot(robot),
    _base(base_link),
    _distal(distal_link)
{
    
    Eigen::Vector3d distal_pos, base_pos;
    
    _robot.getPointPosition(_base, Eigen::Vector3d::Zero(), base_pos);
    _robot.getPointPosition(_distal, Eigen::Vector3d::Zero(), distal_pos);
    
    _ref = distal_pos - base_pos;
    
    _update(Eigen::VectorXd());
    _W.setIdentity(3,3);
}

Eigen::Vector3d CustomRelativeCartesian::getError() const
{
    return _error;
}



void CustomRelativeCartesian::_update(const Eigen::VectorXd& x)
{
    Eigen::Vector3d distal_pos, base_pos;
    Eigen::Matrix3d w_R_base;
    
    _robot.getPointPosition(_base, Eigen::Vector3d::Zero(), base_pos);
    _robot.getPointPosition(_distal, Eigen::Vector3d::Zero(), distal_pos);
    _robot.getOrientation(_base, w_R_base);
    
    _robot.getJacobian(_base, _Jbase);
    _robot.getJacobian(_distal, _Jdistal);
    
    _A = _Jdistal.topRows(3);
    _A -= (_Jbase.topRows(3) -   Utils::skewSymmetricMatrix(distal_pos - base_pos).col(2)*_Jbase.bottomRows(1));
    
    
    _error = w_R_base*_ref - (distal_pos - base_pos);
    _b = _lambda * (_error);
   
}

    
    class SimpleSteering 
    {
      
    public:
        
        SimpleSteering(ModelInterface::ConstPtr model, 
                       std::string wheel_name);
        
        
        
        double computeSteeringAngle(const Eigen::Vector3d& waist_vel, const Eigen::Vector3d& wheel_vel);
        double getDofIndex() const;
        
    private:
        
        static double wrap_angle(double q);
        static double sign(double x);
        
        int _steering_id;
        Eigen::Vector3d _world_steering_axis, _wheel_spinning_axis;
        ModelInterface::ConstPtr _model;
        Joint::ConstPtr _steering_joint;
        std::string _wheel_name;
        std::string _waist_name;
        Eigen::VectorXd _q;
        
        
    };
    


    class WheeledMotionImpl : public CartesianInterfaceImpl
    {

    public:

        WheeledMotionImpl(ModelInterface::Ptr model);

        virtual bool update(double time, double period);
        
        virtual ~WheeledMotionImpl();

    private:
        
        const int NUM_WHEELS = 4;
        const float RADIUS = 0.078;

        static std::vector<std::pair<std::string, std::string>> __generate_tasks();

        typedef OpenSoT::tasks::velocity::Cartesian CartesianTask;
        typedef OpenSoT::tasks::velocity::PureRollingPosition RollingTask;
        typedef OpenSoT::SubTask SubTask;

        std::string get_parent(std::string link);

        Eigen::VectorXd _q, _dq, _ddq, _qpostural;

        CartesianTask::Ptr _waist_cart;
        std::vector<CartesianTask::Ptr> _cartesian_tasks;
        std::vector<CartesianTask::Ptr> _pp_cart, _wheel_cart;
        std::vector<CustomRelativeCartesian::Ptr> _wheel_cart_rel;
        std::vector<SubTask::Ptr> _wheel_pos_xy, _pp_or, _wheel_pos_z;
        std::vector<RollingTask::Ptr> _rolling;
        OpenSoT::tasks::velocity::Postural::Ptr _postural;
        
        std::vector<SimpleSteering> _steering;

        OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr _solver;
        OpenSoT::AutoStack::Ptr _autostack;

        XBot::MatLogger::Ptr _logger;

    };


} }


#endif






