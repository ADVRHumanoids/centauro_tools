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
#include <centauro_tools/heri_hand_control.h>

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
    Eigen::VectorXd _q0, _q;
    
    XBot::RobotInterface::Ptr _robot;
    XBot::MatLogger::Ptr _logger;
    
    XBot::IXBotHand::Ptr _hand;
    
    XBot::RosUtils::SubscriberWrapper::Ptr _sub_rt;
    std::atomic<int> _primitive;
    std::atomic<double> _percentage;
    
    void callback(const centauro_tools::heri_hand_controlConstPtr& msg);

};

}

#endif //__CENTAURO_TOOLS_HERI_HAND_H__