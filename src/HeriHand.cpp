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

REGISTER_XBOT_PLUGIN(HeriHand, CentauroTools::HeriHand)

bool CentauroTools::HeriHand::getHeriHandIds(const srdf_advr::Model& srdfdom)
{

    std::vector<srdf_advr::Model::Group> actual_groups = srdfdom.getGroups();
    int group_num = actual_groups.size();

    srdf_advr::Model::Group HeriHand_group;
    bool found = false; 
    
    // find HeriHand group NOTE expected to be unique
    for(int i = 0; i < group_num && !found; i++) {
        if( actual_groups[i].name_ == "HeriHand"){
            HeriHand_group = actual_groups[i];
            found = true;
        }
    }
    if( !found ){
        XBot::Logger::error() << "ERROR while trying to retrive HERI Hand ESC ids: HeriHand group not found in the srdf : " << srdfdom.getName() << XBot::Logger::endl();
        return false;
    }

    // NOTE expecting 2 joints (ESCs)
    if(HeriHand_group.joints_.size() == 2) { 

        // NOTE meanwhile we do not have a model we assume the joint name is the same as the ESC id
        _heri_esc_id_1 = std::stoi(HeriHand_group.joints_[0]);
        _heri_esc_id_2 = std::stoi(HeriHand_group.joints_[1]);
    }
    else {
        XBot::Logger::error() << "ERROR while trying to retrive HERI Hand ESC ids: HeriHand group does not contain exactly 2 joints in the srdf : " << srdfdom.getName() << XBot::Logger::endl();
        return false;
    }

}



void CentauroTools::HeriHand::callback(const centauro_tools::HeriHandControlConstPtr& msg)
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
    else if(msg->primitive == "tool_grasp") {
        _primitive.store(3);
    }
    else if(msg->primitive == "tool_trigger") {
        _primitive.store(4);
    }
    else if(msg->primitive == "f**k") {
        _primitive.store(5);
    }
    else if(msg->primitive == "finger_1") {
        _primitive.store(11);
    }
    else if(msg->primitive == "finger_2") {
        _primitive.store(12);
    }
    else if(msg->primitive == "finger_3") {
        _primitive.store(13);
    }
    else if(msg->primitive == "finger_4") {
        _primitive.store(14);
    }
    else if(msg->primitive == "tool_smart_grasp") {
        _primitive.store(15);
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
    // get HERI Hand ESC ids
    if( !(getHeriHandIds(handle->getRobotInterface()->getSrdf())) ) {
        XBot::Logger::error() << "ERROR while trying to retrive HERI Hand ESC ids: check that you specified the group HeriHand properly in your srdf, identified in the config file : " << handle->getPathToConfigFile() << XBot::Logger::endl();
        return false;
    }
    // get the low level interface for the hand HACK hardcoded id
    _hand = get_xbotcore_halInterface()->getHandId(_heri_esc_id_1);
    // RT ROS subscriber
    _sub_rt = handle->getRosHandle()->subscribe<centauro_tools::HeriHandControl>("/heri_hand_control", 
                                                                                   1, 
                                                                                   &CentauroTools::HeriHand::callback, 
                                                                                   this);
    //RT ROS publisher
    _pub_rt = handle->getRosHandle()->advertise<centauro_tools::HeriHandState>("/heri_hand_state", 10);
    
    // atomic initialization
    _primitive.store(-1);
    _percentage.store(0.0);

    return true;
}

void CentauroTools::HeriHand::on_start(double time)
{
    _start_time = time;
    XBot::Logger::info() << _heri_esc_id_1 << " - " << _heri_esc_id_2 << " - " << _hand << XBot::Logger::endl(); 
}

void CentauroTools::HeriHand::control_loop(double time, double period)
{
    // TX command the requested grasping primitive
    if(_primitive.load() == 0) {
        
        _hand->grasp(_heri_esc_id_1, _percentage.load());
        _hand->grasp(_heri_esc_id_2, _percentage.load());
    }
    else if(_primitive.load() == 1) {
        
        _hand->move_finger(_heri_esc_id_1, 2, _percentage.load());
        _hand->move_finger(_heri_esc_id_2, 2, _percentage.load());
    }
    else if(_primitive.load() == 2) {
        
        _hand->move_finger(_heri_esc_id_2, 1, _percentage.load());
        
        _hand->move_finger(_heri_esc_id_1, 1, 1.0);
        _hand->move_finger(_heri_esc_id_1, 2, 1.0);
        _hand->move_finger(_heri_esc_id_2, 2, 1.0);
    }
    else if(_primitive.load() == 3) {
        
        _hand->move_finger(_heri_esc_id_1, 1, _percentage.load());
        _hand->move_finger(_heri_esc_id_1, 2, _percentage.load());
        _hand->move_finger(_heri_esc_id_2, 2, _percentage.load());
    }
    else if(_primitive.load() == 4) {
        
        _hand->move_finger(_heri_esc_id_2, 1, _percentage.load());
    }
    else if(_primitive.load() == 5) {
        
        _hand->move_finger(_heri_esc_id_1, 1, _percentage.load());
        _hand->move_finger(_heri_esc_id_2, 1, _percentage.load());
        _hand->move_finger(_heri_esc_id_2, 2, _percentage.load());
    }
    else if(_primitive.load() == 11) {
        
        _hand->move_finger(_heri_esc_id_1, 1, _percentage.load());
    }
    else if(_primitive.load() == 12) {
        
        _hand->move_finger(_heri_esc_id_1, 2, _percentage.load());
    }
    else if(_primitive.load() == 13) {
        
        _hand->move_finger(_heri_esc_id_2, 1, _percentage.load());
    }
    else if(_primitive.load() == 14) {
        
        _hand->move_finger(_heri_esc_id_2, 2, _percentage.load());
    }
    else if(_primitive.load() == 15) {
        
        _hand->move_finger(_heri_esc_id_1, 1, _percentage.load());
        _hand->move_finger(_heri_esc_id_1, 2, _percentage.load());
    }

    
    // RX publish the hand data
    state.seq = seq;
//     state.stamp = time; //TBD check it
    
    // fill the state message
    double motor_pos, current, an1, an2, an3, pos_ref;
    centauro_tools::HeriHandFingerAnalogs analogs;
    centauro_tools::HeriHandFinger finger;
    // TBD iterate over the fingers and fill the state 
    // NOTE | _heri_esc_id_1.1 -> 1 | _heri_esc_id_1.2 -> 2 | _heri_esc_id_2.1 -> 3 | _heri_esc_id_2.2 -> 4 |
    
    _hand->get_finger_motor_position(_heri_esc_id_1, 1, motor_pos);
    _hand->get_finger_current(_heri_esc_id_1, 1, current);
    _hand->get_finger_analog_sensors(_heri_esc_id_1, 1, an1, an2, an3);
    _hand->get_finger_position_reference(_heri_esc_id_1, 1, pos_ref);
    analogs.analog_1 = an1;
    analogs.analog_2 = an2;
    analogs.analog_3 = an3;
    finger.analogs = analogs;
    finger.current = current;
    finger.motor_position = motor_pos;
    finger.position_reference = pos_ref;
    state.finger_1 = finger;
    _logger->add("finger_1_position_reference", pos_ref);
    _logger->add("finger_1_motor_pos", motor_pos);
    _logger->add("finger_1_current", current);
    _logger->add("finger_1_analog_1", an1);
    _logger->add("finger_1_analog_2", an2);
    _logger->add("finger_1_analog_3", an3);

    _hand->get_finger_motor_position(_heri_esc_id_1, 2, motor_pos);
    _hand->get_finger_current(_heri_esc_id_1, 2, current);
    _hand->get_finger_analog_sensors(_heri_esc_id_1, 2, an1, an2, an3);
    _hand->get_finger_position_reference(_heri_esc_id_1, 2, pos_ref);
    analogs.analog_1 = an1;
    analogs.analog_2 = an2;
    analogs.analog_3 = an3;
    finger.analogs = analogs;
    finger.current = current;
    finger.motor_position = motor_pos;
    finger.position_reference = pos_ref;
    state.finger_2 = finger;
    _logger->add("finger_2_position_reference", pos_ref);
    _logger->add("finger_2_motor_pos", motor_pos);
    _logger->add("finger_2_current", current);
    _logger->add("finger_2_analog_1", an1);
    _logger->add("finger_2_analog_2", an2);
    _logger->add("finger_2_analog_3", an3);
    
    _hand->get_finger_motor_position(_heri_esc_id_2, 1, motor_pos);
    _hand->get_finger_current(_heri_esc_id_2, 1, current);
    _hand->get_finger_analog_sensors(_heri_esc_id_2, 1, an1, an2, an3);
    _hand->get_finger_position_reference(_heri_esc_id_2, 1, pos_ref);
    analogs.analog_1 = an1;
    analogs.analog_2 = an2;
    analogs.analog_3 = an3;
    finger.analogs = analogs;
    finger.current = current;
    finger.motor_position = motor_pos;
    finger.position_reference = pos_ref;
    state.finger_3 = finger;
    _logger->add("finger_3_position_reference", pos_ref);
    _logger->add("finger_3_motor_pos", motor_pos);
    _logger->add("finger_3_current", current);
    _logger->add("finger_3_analog_1", an1);
    _logger->add("finger_3_analog_2", an2);
    _logger->add("finger_3_analog_3", an3);
    
    _hand->get_finger_motor_position(_heri_esc_id_2, 2, motor_pos);
    _hand->get_finger_current(_heri_esc_id_2, 2, current);
    _hand->get_finger_analog_sensors(_heri_esc_id_2, 2, an1, an2, an3);
    _hand->get_finger_position_reference(_heri_esc_id_2, 2, pos_ref);
    analogs.analog_1 = an1;
    analogs.analog_2 = an2;
    analogs.analog_3 = an3;
    finger.analogs = analogs;
    finger.current = current;
    finger.motor_position = motor_pos;
    finger.position_reference = pos_ref;
    state.finger_4 = finger;
    _logger->add("finger_4_position_reference", pos_ref);
    _logger->add("finger_4_motor_pos", motor_pos);
    _logger->add("finger_4_current", current);
    _logger->add("finger_4_analog_1", an1);
    _logger->add("finger_4_analog_2", an2);
    _logger->add("finger_4_analog_3", an3);

    _pub_rt->pushToQueue(state);
    
    _logger->add("time", time);
    seq++;
}

bool CentauroTools::HeriHand::close()
{
    _logger->flush();
}
