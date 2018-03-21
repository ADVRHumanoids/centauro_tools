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
    
    class HysteresisComparator
    {
      
    public:
        
        HysteresisComparator(double th_lo = -1., 
                             double th_hi =  1., 
                             bool init_lo = true);
        
        bool compare(double value);
        
    private:
        
        const double _th_lo, _th_hi;
        double _th_curr;
        
    };
    

    
    class SimpleSteering 
    {
      
    public:
        
        SimpleSteering(ModelInterface::ConstPtr model, 
                       std::string wheel_name);
        
        
        
        double computeSteeringAngle(const Eigen::Vector3d& waist_vel, const Eigen::Vector3d& wheel_vel);
        double getDofIndex() const;
        void log(XBot::MatLogger::Ptr logger);
        
    private:
        
        static double wrap_angle(double q);
        static double sign(double x);
        
        HysteresisComparator _comp;
        int _steering_id;
        Eigen::Vector3d _world_steering_axis, _wheel_spinning_axis;
        ModelInterface::ConstPtr _model;
        Joint::ConstPtr _steering_joint;
        std::string _wheel_name;
        std::string _waist_name;
        Eigen::VectorXd _q;
        
        Eigen::Vector3d _vdes;
        
        
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






