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

#include <centauro_tools/HeriHand.h>

#include <centauro_tools/heri_hand_control.h>

REGISTER_XBOT_PLUGIN(HeriHand, CentauroTools::HeriHand)

void CentauroTools::HeriHand::callback(const centauro_tools::heri_hand_controlConstPtr& msg)
{
    // primitive
    if(msg->primitive == "grasp") {
        _primitive.store(0);
    }
    else if(msg->primitive == "pinch") {
        _primitive.store(1);    
    }
    else if(msg->primitive == "trigger") {
        _primitive.store(2);
    }
    else {
        _primitive.store(-1);
    }
    
    // percentage
    _percentage.store(msg->percentage);
    
}


bool CentauroTools::HeriHand::init_control_plugin(XBot::Handle::Ptr handle)
{
    // get robot
    _robot = handle->getRobotInterface();
    // instantiate logger
    _logger = XBot::MatLogger::getLogger("/tmp/HeriHand_logger");
    // get the low level interface for the hand HACK hardcoded id
    _hand = get_xbotcore_halInterface()->getHandId(110);
    // RT ROS subscriber
    _sub_rt = handle->getRosHandle()->subscribe<centauro_tools::heri_hand_control>("/heri_hand_control", 
                                                                                   1, 
                                                                                   &CentauroTools::HeriHand::callback, 
                                                                                   this);
    _primitive.store(-1);
    _percentage.store(0.0);

    return true;
}

void CentauroTools::HeriHand::on_start(double time)
{
    _start_time = time;
    
}

void CentauroTools::HeriHand::control_loop(double time, double period)
{
        
    if(_primitive.load() == 0) {
        
        _hand->grasp(110, _percentage.load());
        _hand->grasp(111, _percentage.load());
    }
    else if(_primitive.load() == 1) {
        
        _hand->move_finger(110, 2, _percentage.load());
        _hand->move_finger(111, 2, _percentage.load());
    }
    else if(_primitive.load() == 2) {
        
        _hand->move_finger(111, 1, _percentage.load());
        
        _hand->move_finger(110, 1, 1.0);
        _hand->move_finger(110, 2, 1.0);
        _hand->move_finger(111, 2, 1.0);
    }
    
    _logger->add("time", time);
}

bool CentauroTools::HeriHand::close()
{
    _logger->flush();
}
