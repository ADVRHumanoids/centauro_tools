#ifndef __MPL_CENTAURO_FORCE_EST_H__
#define __MPL_CENTAURO_FORCE_EST_H__

#include <XBotInterface/RobotInterface.h>
#include <vector>

namespace centauro {

    class HysteresisComparator
    {
      
    public:
        
        HysteresisComparator(double th_lo = -1., 
                             double th_hi =  1., 
                             bool init_lo = true);
        
        bool compare(double value);
        double getCurrentThreshold() const;
        
    private:
        
        const double _th_lo, _th_hi;
        double _th_curr;
        
    };
    
    class ForceEstimation 
    {
        
    public:
        
        typedef std::shared_ptr<ForceEstimation> Ptr;
        
        /**
         * @brief Contruct the force estimation class.
         * 
         * @param model Model that is kept updated with the robot state
         * @param contact_thresh_lo Low threshold for contact detection
         * @param contact_thresh_hi High threshold for contact detection
         */
        ForceEstimation(XBot::ModelInterface::Ptr model, 
                        double contact_thresh_lo = 45, 
                        double contact_thresh_hi = 55);
        
        /**
         * @brief Call this function to compute an estimate based on the current model state.
         */
        void compute();
        
        Eigen::Vector3d getForce(int i, Eigen::VectorXd * tau_res = nullptr) const;
        
        bool getEstimatedContactState(int i);
        
       
        ~ForceEstimation();
        
    private:
        
        XBot::ModelInterface::Ptr _model;
        XBot::MatLogger::Ptr _logger;
        
        std::vector<std::string> _wheels;
        std::vector<Eigen::Vector3d> _forces;
        std::vector<Eigen::VectorXd> _res;
        std::vector<HysteresisComparator> _comp;
        
        Eigen::JacobiSVD<Eigen::Matrix<double, 5, 3>> _svd;
        
        Eigen::VectorXd _tau, _nl;
        Eigen::MatrixXd _J;
        
    };
    
}


#endif