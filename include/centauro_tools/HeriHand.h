/*
 * Copyright (C) 2018 IIT-HHCM
 * Author: Luca Muratore
 * email:  luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef __CENTAURO_TOOLS_HERI_HAND_H__
#define __CENTAURO_TOOLS_HERI_HAND_H__

#include <XCM/XBotControlPlugin.h>

#include <centauro_tools/HeriHandControl.h>
#include <centauro_tools/HeriHandState.h>

#include <atomic>

namespace CentauroTools {

class HeriHand : public XBot::XBotControlPlugin {

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual void on_start(double time);

    virtual void control_loop(double time, double period);

    virtual bool close();


private:

    double _start_time;
    int seq = 0;
    Eigen::VectorXd _q0, _q;
    
    int _heri_esc_id_1, _heri_esc_id_2;
    
    XBot::RobotInterface::Ptr _robot;
    XBot::MatLogger::Ptr _logger;
    
    XBot::IXBotHand::Ptr _hand;
    
    XBot::RosUtils::SubscriberWrapper::Ptr _sub_rt;
    std::atomic<int> _primitive;
    std::atomic<double> _percentage;
    
    XBot::RosUtils::PublisherWrapper::Ptr _pub_rt;
    centauro_tools::HeriHandState state;
    
    void callback(const centauro_tools::HeriHandControlConstPtr& msg);
    
    bool getHeriHandIds( const srdf_advr::Model& srdfdom );
 
};

}

#endif //__CENTAURO_TOOLS_HERI_HAND_H__