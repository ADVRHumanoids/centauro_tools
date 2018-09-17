#include <centauro_tools/CentauroForceEstimation.h>



centauro::ForceEstimation::ForceEstimation(XBot::ModelInterface::Ptr model, double th_lo, double th_hi):
    _model(model),
    _logger(XBot::MatLogger::getLogger("/tmp/centauro_force_estimation_log")),
    _forces(4, Eigen::Vector3d::Zero()),
    _res(4, Eigen::VectorXd::Zero(5)),
    _comp(4, HysteresisComparator(th_lo, th_hi, true)),
    _J(6, model->getJointNum())
{
    for(int i = 0; i < 4; i++)
    {
        _wheels.push_back(_model->leg(i).getTipLinkName());
    }
    
    compute();
}


void centauro::ForceEstimation::compute()
{

    _model->getJointEffort(_tau);
    _model->computeNonlinearTerm(_nl);
    
    static std::vector<std::string> hips = {"hip_yaw_1", "hip_yaw_2","hip_yaw_4","hip_yaw_3"};
    for(int i = 0; i < 4; i++)
    {
        Eigen::Matrix3d w_R_wheel;
        _model->getOrientation(_wheels[i], w_R_wheel);
        _model->getJacobian(_wheels[i], _J);
        int start_dof = _model->getDofIndex(hips[i]);
        auto Jt = _J.topRows<3>().middleCols<5>(start_dof).transpose();
        
        _svd.compute(Jt, Eigen::ComputeFullU|Eigen::ComputeFullV);
        
        auto tau_residual = (_nl - _tau).segment<5>(start_dof);
        Eigen::Vector3d f_est = _svd.solve(tau_residual);
        
        _logger->add("f_est_" + std::to_string(i+1), f_est);
        _logger->add("tau_residual_" + std::to_string(i+1), tau_residual);
        
        _forces[i] = f_est;
        _res[i] = tau_residual;
        
    }
}

Eigen::Vector3d centauro::ForceEstimation::getForce(int i, Eigen::VectorXd* tau_res) const
{
    if(tau_res) *tau_res = _res.at(i);
    return _forces.at(i);
}

centauro::ForceEstimation::~ForceEstimation()
{
    _logger->flush();
}

centauro::HysteresisComparator::HysteresisComparator(double th_lo, double th_hi, bool init_lo):
     _th_lo(th_lo),
     _th_hi(th_hi),
     _th_curr(init_lo ? th_lo : th_hi)
{
    if(_th_hi < _th_lo)
    {
        throw std::invalid_argument("upper threshold < lower threshold");
    }
}

bool centauro::HysteresisComparator::compare(double value)
{
    bool ret = value > _th_curr;
    _th_curr = ret ? _th_lo : _th_hi;
    return ret;
}

double centauro::HysteresisComparator::getCurrentThreshold() const
{
    return _th_curr;
}

bool centauro::ForceEstimation::getEstimatedContactState(int i)
{
    return _comp[i].compare(_res[i].array().abs().sum());
}

