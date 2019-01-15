#include <centauro_tools/SimpleWheeledMotion.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/TaskToConstraint.h>

extern "C" XBot::Cartesian::CartesianInterface* create_instance(XBot::ModelInterface::Ptr model,
                                                                XBot::Cartesian::ProblemDescription pb)
{
    return new XBot::Cartesian::WheeledMotionImpl(model);
}

extern "C" void destroy_instance( XBot::Cartesian::CartesianInterface* instance )
{
    delete instance;
}

namespace XBot { namespace Cartesian {

WheeledMotionImpl::WheeledMotionImpl(ModelInterface::Ptr model):
    CartesianInterfaceImpl(model, __generate_tasks()),
    _logger(MatLogger::getLogger("/tmp/wheeled_motion_log"))
{
    _model->getJointPosition(_q);
    _qpostural = _q;
    _dq_steering = _ddq = _dq = _q*0;
    
    std::vector<string> joints_out;
    Eigen::VectorXd _q_motor;
    _model->getMotorPosition(_q_motor);
    _model->checkJointLimits(_q, joints_out );
    for( const auto& j : joints_out ) {
        XBot::Logger::error() << j << " -> motor position is outside of range " << XBot::Logger::endl();
    }

    
    std::vector<std::string> steering_joints = {"ankle_yaw_1", "ankle_yaw_2", "ankle_yaw_3", "ankle_yaw_4"};
    std::vector<bool> disable_steering(_model->getJointNum(), true);
    std::list<uint> steering_joints_ids;
    
    for(auto j : steering_joints)
    {
        disable_steering[ _model->getDofIndex(j) ] = false;
        steering_joints_ids.push_back(_model->getDofIndex(j));

    }
    
    std::vector<bool> spinning_only(_model->getJointNum(), false);
    spinning_only[0] = true;
    spinning_only[1] = true;
    spinning_only[2] = true;
    spinning_only[5] = true;
    setActiveJoints(_model, "j_wheel_1", spinning_only);
    setActiveJoints(_model, "j_wheel_2", spinning_only);
    setActiveJoints(_model, "j_wheel_3", spinning_only);
    setActiveJoints(_model, "j_wheel_4", spinning_only);

    auto pos_idx = OpenSoT::Indices::range(0,2);
    auto pos_idx_xy = OpenSoT::Indices::range(0,1);
    auto pos_idx_z = OpenSoT::Indices::range(2,2);
    auto or_idx = OpenSoT::Indices::range(3,5);
    auto or_xy_idx = OpenSoT::Indices::range(3,4);
    auto pos_rotz_idx = pos_idx.asList();
    pos_rotz_idx.push_back(5);

    std::vector<CartesianTask::TaskPtr> ee_tasks;

    for(int i = 0; i < _model->arms(); i++)
    {
        std::string ee_name = "arm" + std::to_string(i+1) + "_8";
        
        auto ee_cart = boost::make_shared<CartesianTask>("ARM_CART_" + std::to_string(i),
                                                         _q,
                                                         *_model,
                                                         ee_name,
                                                         "pelvis"
                                                        );
        
        ee_cart->setLambda(0.1);
        _cartesian_tasks.push_back(ee_cart);
        ee_tasks.push_back(ee_cart);
    }

    for(int i = 0; i < NUM_WHEELS; i++)
    {
        std::string wheel_name = _model->leg(i).getTipLinkName();
        
        _steering.emplace_back(_model, wheel_name);

        auto wheel_cartesian_rel = boost::make_shared<CustomRelativeCartesian>(*_model,
                                                                 wheel_name,
                                                                 "pelvis"
                                                                );
        wheel_cartesian_rel->setActiveJointsMask(disable_steering);

        wheel_cartesian_rel->setLambda(0.01);
        
        auto wheel_pos_rel_xy = wheel_cartesian_rel % pos_idx_xy;
        
        
        
        
        auto wheel_cartesian = boost::make_shared<CartesianTask>("WHEEL_CART_" + std::to_string(i),
                                                                 _q,
                                                                 *_model,
                                                                 wheel_name,
                                                                 "world"
                                                                );
        
        wheel_cartesian->setActiveJointsMask(disable_steering);

        wheel_cartesian->setLambda(0.1);

        auto wheel_pos_z = wheel_cartesian % pos_idx_z;

        auto wheel_rolling =  boost::make_shared<RollingTask>(_model->leg(i).getTipLinkName(), RADIUS, *_model);

        std::string pp_link = get_parent(get_parent(wheel_name));

        auto pp_cartesian =   boost::make_shared<CartesianTask>("PP_CART_" + std::to_string(i),
                                                                 _q,
                                                                 *_model,
                                                                 pp_link,
                                                                 "world"
                                                                );
        pp_cartesian->setLambda(0.03);
        auto pp_or = pp_cartesian % or_xy_idx;
        
        auto p_cartesian =  boost::make_shared<CartesianTask>("P_CART_" + std::to_string(i),
                                                               _q,
                                                              *_model,
                                                               get_parent(wheel_name),
                                                               "world"
                                                               );
        
        p_cartesian->setLambda(0.1);

        _wheel_cart_rel.push_back(wheel_cartesian_rel);
        _wheel_cart.push_back(wheel_cartesian);
        _wheel_pos_xy.push_back(wheel_pos_rel_xy);
        _wheel_pos_z.push_back(wheel_pos_z);
        _rolling.push_back(wheel_rolling);
        _pp_cart.push_back(pp_cartesian);
        _p_cart.push_back(p_cartesian);
        _cartesian_tasks.push_back(p_cartesian);
        _pp_or.push_back(pp_or);
//         _cartesian_tasks.push_back(wheel_cartesian);

    }

    auto pp_or_xy_aggr = _pp_or[0] + _pp_or[1] + _pp_or[2] + _pp_or[3];
    auto wheel_pos_aggr = _wheel_pos_xy[0] + _wheel_pos_xy[1] + _wheel_pos_xy[2] + _wheel_pos_xy[3];
    auto rolling_aggr = _rolling[0] + _rolling[1] + _rolling[2] + _rolling[3];
    auto wheel_z_aggr = _wheel_pos_z[0] + _wheel_pos_z[1] + _wheel_pos_z[2] + _wheel_pos_z[3];
    auto ee_aggr = ee_tasks[0] + ee_tasks[1];
    auto p_pos_z_aggr = _p_cart[0]%pos_idx_z + _p_cart[1]%pos_idx_z + _p_cart[2]%pos_idx_z + _p_cart[3]%pos_idx_z;

    _waist_cart = boost::make_shared<CartesianTask>("WAIST_CART",
                                                    _q,
                                                    *_model,
                                                    "pelvis",
                                                    "world"
                                                   );
     _waist_cart->setLambda(0.05);
     _cartesian_tasks.push_back(_waist_cart);

    _postural = boost::make_shared<OpenSoT::tasks::velocity::Postural>(_q);
    _postural->setLambda(0.05);

    _steering_task = boost::make_shared<OpenSoT::tasks::velocity::Postural>(_q);
    _steering_task->setLambda(0.0);



    Eigen::VectorXd qdotmax, qmin, qmax;
    _model->getVelocityLimits(qdotmax);
    _model->getJointLimits(qmin, qmax);
    auto velocity_lims = boost::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(qdotmax, 0.01);
    auto joint_lims = boost::make_shared<OpenSoT::constraints::velocity::JointLimits>(_q, qmax, qmin);

    _autostack = (  
                    ( _waist_cart + p_pos_z_aggr + _steering_task%steering_joints_ids ) / 
                    ( rolling_aggr + pp_or_xy_aggr + ee_aggr ) /
                      _postural
                 ) << velocity_lims 
                   << joint_lims
                   << boost::make_shared<OpenSoT::constraints::TaskToConstraint>(wheel_pos_aggr);
                 
    _autostack->update(_q);


    /* Create solver */
    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(_autostack->getStack(),
                                                         _autostack->getBounds(),
                                                         1e8,
                                                         OpenSoT::solvers::solver_back_ends::OSQP
                                                        );
    
    /* Fill lambda map */
    for(auto t : _cartesian_tasks)
    {
        _lambda_map[t->getDistalLink()] = t->getLambda();
    }
    
    for(auto t : _wheel_cart_rel)
    {
        _lambda_map[t->getDistalLink()] = t->getLambda();
    }
}

double SimpleSteering::sign(double x)
{
    return (x > 0) - (x < 0);
}


bool WheeledMotionImpl::setBaseLink(const std::string& ee_name, const std::string& new_base_link)
{
    if(!XBot::Cartesian::CartesianInterfaceImpl::setBaseLink(ee_name, new_base_link))
    {
        return false;
    }
    
    bool success = false;
    
    for(auto t : _cartesian_tasks)
    {
        if(t->getDistalLink() == ee_name)
        {
            success = t->setBaseLink(new_base_link);
        }
    }
    
    return success;
    
}

bool WheeledMotionImpl::setControlMode(const std::string& ee_name, ControlType ctrl_type)
{
    if(!XBot::Cartesian::CartesianInterfaceImpl::setControlMode(ee_name, ctrl_type))
    {
        return false;
    }
    
    OpenSoT::tasks::Aggregated::TaskPtr task_ptr;
    
    if(ee_name == "com")
    {
        XBot::Logger::error("Com task undefined\n");
        return false;
    }
    
    for(const auto t : _cartesian_tasks)
    {
        if(t->getDistalLink() == ee_name)
        {
            task_ptr = t;
        }
    }
    
    for(const auto t : _wheel_cart_rel)
    {
        if(t->getDistalLink() == ee_name)
        {
            task_ptr = t;
        }
    }
    
    if(task_ptr)
    {
        
        switch(ctrl_type)
        {
            case ControlType::Disabled:
                task_ptr->setActive(false);
                break;
                
            case ControlType::Velocity:
                task_ptr->setActive(true);
                _lambda_map[ee_name] = task_ptr->getLambda();
                task_ptr->setLambda(0.0);
                break;
                
            case ControlType::Position:
                if( _lambda_map.find(ee_name) == _lambda_map.end() )
                {
                    XBot::Logger::error("No lambda value for task %s defined, contact the developers\n", ee_name.c_str());
                    return false;
                }
                task_ptr->setActive(true);
                task_ptr->setLambda(_lambda_map.at(ee_name));
                break;
                
            default:
                break;
            
        }
        
        return true;
    }
    
    return false;
}

bool WheeledMotionImpl::reset(double time)
{
    if(!XBot::Cartesian::CartesianInterfaceImpl::reset(time))
    {
        throw std::runtime_error("CartesianInterfaceImpl::reset returned error");
    }
    
    Logger::info("WheeledMotion: resetting references for tasks\n");
    for(auto t :_wheel_cart_rel)
    {
        t->reset();
        Eigen::Vector3d ref = t->getReference();
        
        Eigen::Affine3d curr_ref;
        getPoseReference(t->getDistalLink(), curr_ref);
        
        curr_ref.translation() = ref;
        setPoseReferenceRaw(t->getDistalLink(), curr_ref);
    }
    
    for(auto t : _wheel_cart)
    {
        Eigen::Affine3d T;
        _model->getPose(t->getDistalLink(), T);
        t->setReference(T);
    }
    
    for(auto t : _pp_cart)
    {
        Eigen::Affine3d T;
        _model->getPose(t->getDistalLink(), T);
        t->setReference(T);
    }
    
    return true;
}



bool WheeledMotionImpl::update(double time, double period)
{
    bool success = true;

    XBot::Cartesian::CartesianInterfaceImpl::update(time, period);

    _model->getJointPosition(_q);

    /* Update reference for all cartesian tasks */
    for(auto cart_task : _cartesian_tasks)
    {
        Eigen::Affine3d T_ref;
        Eigen::Vector6d v_ref, a_ref;

        if(!getPoseReference(cart_task->getDistalLink(), T_ref, &v_ref, &a_ref))
        {
            Logger::warning("WheeledMotion: no reference available for task %s\n", cart_task->getDistalLink().c_str());
            continue;
        }

        cart_task->setReference(T_ref, v_ref*period);

    }
    
    for(auto cart_task : _wheel_cart_rel)
    {
        Eigen::Affine3d T_ref;
        Eigen::Vector6d v_ref, a_ref;

        if(!getPoseReference(cart_task->getDistalLink(), T_ref, &v_ref, &a_ref))
        {
            Logger::warning("WheeledMotion: no reference available for task %s\n", cart_task->getDistalLink().c_str());
            continue;
        }

        cart_task->setReference(T_ref.translation(), v_ref.head<3>()*period);
    }
    
    
    _autostack->update(_q);
    _autostack->log(_logger);

    if(!_solver->solve(_dq))
    {
        _dq.setZero(_dq.size());
        XBot::Logger::error("OpenSot: unable to solve\n");
        success = false;
    }
    
    /* Steering */
    Eigen::Vector3d waist_vref;
    waist_vref << _waist_cart->getError().head<2>(), _waist_cart->getError()(5);
    Eigen::Vector3d wheel_vref(0, 0, 0);
    _logger->add("waist_vref", waist_vref);

    for(int i = 0; i < NUM_WHEELS; i++)
    {
        auto& steering = _steering[i];
        std::string ankle_name = get_parent(steering.getWheelName());
        Eigen::Vector6d wheel_vel;
        _model->getVelocityTwist(ankle_name, wheel_vel);
        _logger->add(steering.getWheelName() + "_vel", wheel_vel);
        Eigen::Matrix3d w_R_ankle;
        _model->getOrientation(ankle_name, w_R_ankle);
        wheel_vel.head<3>() += wheel_vel.tail<3>().cross(w_R_ankle * Eigen::Vector3d(0.0, 0.0, 0.30));
        
        steering.log(_logger);
        
        
        
        
        wheel_vref = _wheel_cart_rel[i]->getError().head<3>();
        _qpostural(steering.getDofIndex()) = steering.computeSteeringAngle(waist_vref*0, wheel_vel.head<3>());
        
        const double STEERING_GAIN = 0.1;
        const double MAX_STEERING_SPEED = 3.0;
        const double MAX_STEERING_DQ = MAX_STEERING_SPEED*period;
        double dq = STEERING_GAIN*(_qpostural(steering.getDofIndex()) - _q(steering.getDofIndex()));
        _dq_steering(steering.getDofIndex()) = std::min(std::max(dq, -MAX_STEERING_DQ), MAX_STEERING_DQ);
        _steering_task->setReference(_q, _dq_steering);
        
        
        
        
        _logger->add("wheel_vref_"+std::to_string(i+1), wheel_vref);
        _logger->add("steering_angle_"+std::to_string(i+1), _qpostural(steering.getDofIndex()));
    }

    _dq /= period;
    _model->setJointVelocity(_dq);
    _model->setJointAcceleration(_ddq);


    return success;


}

std::string WheeledMotionImpl::get_parent(std::string link)
{
    return _model->getUrdf().getLink(link)->parent_joint->parent_link_name;
}

ProblemDescription WheeledMotionImpl::__generate_tasks()
{
    AggregatedTask tasks;
    
    tasks.push_back( MakeCartesian("pelvis"  ,"world" ));
    tasks.push_back( MakeCartesian("wheel_1" ,"pelvis"));
    tasks.push_back( MakeCartesian("wheel_2" ,"pelvis"));
    tasks.push_back( MakeCartesian("wheel_3" ,"pelvis"));
    tasks.push_back( MakeCartesian("wheel_4" ,"pelvis"));
    tasks.push_back( MakeCartesian("ankle2_1","world" ));
    tasks.push_back( MakeCartesian("ankle2_2","world" ));
    tasks.push_back( MakeCartesian("ankle2_3","world" ));
    tasks.push_back( MakeCartesian("ankle2_4","world" ));
    tasks.push_back( MakeCartesian("arm1_8"  ,"pelvis"));
    tasks.push_back( MakeCartesian("arm2_8"  ,"pelvis"));

    return ProblemDescription(tasks);
}

WheeledMotionImpl::~WheeledMotionImpl()
{
    _logger->flush();
}

SimpleSteering::SimpleSteering(XBot::ModelInterface::ConstPtr model, 
                               std::string wheel_name):
    _model(model),
    _wheel_name(wheel_name),
    _comp(0.0025, 0.01)
{
    auto spinning_axis = _model->getUrdf().getLink(wheel_name)->parent_joint->axis;
    _wheel_spinning_axis << spinning_axis.x, spinning_axis.y, spinning_axis.z;
    
    std::string wheel_parent_name = _model->getUrdf().getLink(wheel_name)->parent_joint->parent_link_name;
    auto steering_axis = _model->getUrdf().getLink(wheel_parent_name)->parent_joint->axis;
    _world_steering_axis << steering_axis.x, steering_axis.y, steering_axis.z;
    Eigen::Matrix3d w_R_wp;
    _model->getOrientation(wheel_parent_name, w_R_wp);
    _world_steering_axis = w_R_wp * _world_steering_axis;
    
    
    _steering_id = _model->getDofIndex(_model->getUrdf().getLink(wheel_parent_name)->parent_joint->name);
    _steering_joint = _model->getJointByName(_model->getUrdf().getLink(wheel_parent_name)->parent_joint->name);
    
    _model->getFloatingBaseLink(_waist_name);
}

double SimpleSteering::wrap_angle(double x)
{
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0)
        x += 2 * M_PI;
    return x - M_PI;
}

double SimpleSteering::getDofIndex() const
{
    return _steering_id;
}

namespace 
{
    double dead_zone(double x, double th)
    {
        if(x > th)
        {
            return x - th;
        }
        
        if(x < -th)
        {
            return x + th;
        }
        
        return 0.0;
    }
}

double SimpleSteering::computeSteeringAngle(const Eigen::Vector3d& waist_vel, 
                                            const Eigen::Vector3d& __wheel_vel)
{
    Eigen::Vector3d wheel_vel = __wheel_vel;
    
    
    /* Wheel orientation */
    Eigen::Affine3d w_T_wheel;
    _model->getPose(_wheel_name, w_T_wheel);
    
    Eigen::Affine3d w_T_waist;
    _model->getPose(_waist_name, w_T_waist);
    
    /* Steering joint angle */
    _model->getJointPosition(_q);
    double q = _q(_steering_id);
    
    /* Current angle */
    Eigen::Vector3d wheel_forward = (w_T_wheel.linear()*_wheel_spinning_axis).cross(Eigen::Vector3d::UnitZ());
    wheel_forward.normalize();
    double theta = std::atan2(wheel_forward.y(), wheel_forward.x());
    
    /* Desired angle */
    Eigen::Vector3d r = w_T_wheel.translation() - w_T_waist.translation();
    _vdes = wheel_vel + waist_vel + waist_vel.z()*Eigen::Vector3d::UnitZ().cross(r);
    
    Eigen::Vector3d vdes_th = _vdes;
    
    vdes_th.x() = dead_zone(vdes_th.x(), 0.01);
    vdes_th.y() = dead_zone(vdes_th.y(), 0.01);
    
    if( vdes_th.head(2).norm() == 0.0 ) 
    {
        vdes_th << 1.0, 0.0, 0.0;
        vdes_th = w_T_waist.linear() * vdes_th;
    }
    
    double des_theta_1 = std::atan2(vdes_th.y(), vdes_th.x());
    
    double des_q_1 = q  + (des_theta_1 - theta)*_world_steering_axis.z();
    des_q_1 = wrap_angle(des_q_1);
    
    double des_q_2 = des_q_1 + M_PI;
    des_q_2 = wrap_angle(des_q_2);
    
    double closest_q = std::fabs(des_q_1-q) < std::fabs(des_q_2-q) ? des_q_1 : des_q_2;
    
    /* Return value inside joint lims */
    if(_steering_joint->checkJointLimits(closest_q))
    {
        return closest_q;
    }
    else if(_steering_joint->checkJointLimits(des_q_1))
    {
        return des_q_1;
    }
    else if(_steering_joint->checkJointLimits(des_q_2))
    {
        return des_q_2;
    }
    else{
        throw std::runtime_error("Unable to find steering angle");
    }

    
}

void SimpleSteering::log(MatLogger::Ptr logger)
{
    logger->add("vdes_"+_wheel_name, _vdes);
    logger->add("vdes_norm_"+_wheel_name, double(_vdes.head<2>().norm()));
    logger->add("threshold_"+_wheel_name, _comp.getCurrentThreshold());
}



const std::string& CustomRelativeCartesian::getDistalLink() const
{
    return _distal;
}


void CustomRelativeCartesian::setReference(const Eigen::Vector3d& ref, const Eigen::Vector3d& vref)
{
    
    _ref = ref; //(w_ref - w_T_base.translation());
    _vref = vref;
    
}

bool XBot::Cartesian::CustomRelativeCartesian::reset()
{
    Eigen::Vector3d distal_pos, base_pos;
    
    _robot.getPointPosition(_base, Eigen::Vector3d::Zero(), base_pos);
    _robot.getPointPosition(_distal, Eigen::Vector3d::Zero(), distal_pos);
    
    Eigen::Matrix3d w_R_base;
    _robot.getOrientation(_base, w_R_base);
    
    double theta_base = std::atan2(w_R_base(1,0), w_R_base(0,0));
    Eigen::Matrix3d w_R_horz = Eigen::AngleAxisd(theta_base, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    
    _ref = w_R_horz.transpose()*(distal_pos - base_pos);
    _vref.setZero();
    
//     std::cout << __func__ << "  _ref: " << _ref.transpose() << std::endl;
}


CustomRelativeCartesian::CustomRelativeCartesian(const XBot::ModelInterface& robot, 
                                                 std::string distal_link, 
                                                 std::string base_link): 
    Task< Eigen::MatrixXd, Eigen::VectorXd >("CUSTOM_REL_" + distal_link, robot.getJointNum()),
    _robot(robot),
    _base(base_link),
    _distal(distal_link)
{
    
    reset();

    _update(Eigen::VectorXd());
    _W.setIdentity(3,3);
}

Eigen::Matrix3d CustomRelativeCartesian::getHorzFrameRotation() const
{
    Eigen::Matrix3d w_R_base;
    _robot.getOrientation(_base, w_R_base);
    
    double theta_base = std::atan2(w_R_base(1,0), w_R_base(0,0));
    return Eigen::AngleAxisd(theta_base, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}


Eigen::Vector3d CustomRelativeCartesian::getError() const
{
    return _error;
}

Eigen::Vector3d CustomRelativeCartesian::getReference() const
{
    return _ref;
}


void CustomRelativeCartesian::_update(const Eigen::VectorXd& x)
{
    Eigen::Vector3d distal_pos, base_pos;
    Eigen::Matrix3d w_R_base;
    
    _robot.getPointPosition(_base, Eigen::Vector3d::Zero(), base_pos);
    _robot.getPointPosition(_distal, Eigen::Vector3d::Zero(), distal_pos);
    _robot.getOrientation(_base, w_R_base);
    
    double theta_base = std::atan2(w_R_base(1,0), w_R_base(0,0));
    Eigen::Matrix3d w_R_horz = Eigen::AngleAxisd(theta_base, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    _robot.getJacobian(_base, _Jbase);
    _robot.getJacobian(_distal, _Jdistal);
    
    _Jbase.bottomRows(3).topRows(2) = Eigen::MatrixXd::Zero(2, _robot.getJointNum());
    
    _A = _Jdistal.topRows(3);
    _A -= (_Jbase.topRows(3) -   Utils::skewSymmetricMatrix(distal_pos - base_pos)*_Jbase.bottomRows(3));
    
    _error = w_R_horz*_ref - (distal_pos - base_pos);

    _b = _lambda * (_error) + w_R_horz * _vref;
    _vref.setZero();
    
//     std::cout << __func__ << "  _ref: " << _ref.transpose() << "   error: " << _error.transpose() << std::endl;
   
}

void CustomRelativeCartesian::_log(MatLogger::Ptr logger)
{
    logger->add(_task_id + "_error", _error);
    logger->add(_task_id + "_ref", _ref);
}



HysteresisComparator::HysteresisComparator(double th_lo, double th_hi, bool init_lo):
     _th_lo(th_lo),
     _th_hi(th_hi),
     _th_curr(init_lo ? th_lo : th_hi)
{
    if(_th_hi < _th_lo)
    {
        throw std::invalid_argument("upper threshold < lower threshold");
    }
}

bool HysteresisComparator::compare(double value)
{
    bool ret = value > _th_curr;
    _th_curr = ret ? _th_lo : _th_hi;
    return ret;
}

double HysteresisComparator::getCurrentThreshold() const
{
    return _th_curr;
}


} }
