#pragma once

#include "retroid_gamepad.h"
#include "user_command_interface.h"
#include "custom_types.h"

using namespace interface;
using namespace types;

class RetroidGamepadInterface : public UserCommandInterface{
private:
    std::shared_ptr<RetroidGamepad> gamepad_ptr_;
    RetroidKeys rt_keys_record_, rt_keys_;
    UserCommand usr_cmd_;
    bool transform_cmd_flag_ = true;
    std::thread transform_thread_;
    bool first_flag_ = true;

    bool IsKeysEqual(const RetroidKeys& a, const RetroidKeys& b){
        if(a.value != b.value) return false;
        for(int i=0;i<kAxisChannlSize;++i){
            if(a.axis_values[i] != b.axis_values[i]) return false;
        }
        return true;
    }
public:
    RetroidGamepadInterface(int port){
        std::cout << "Using Retroid Gamepad Command Interface" << std::endl;
        gamepad_ptr_ = std::make_shared<RetroidGamepad>(port);
        std::memset(&usr_cmd_, 0, sizeof(usr_cmd_));
    }
    ~RetroidGamepadInterface(){}
    virtual void Start(){
        gamepad_ptr_->StartDataThread();
        transform_thread_ = std::thread(std::bind(&RetroidGamepadInterface::TransformRetroidToUserCommand, this));
    }
    virtual void Stop(){
        gamepad_ptr_->StopDataThread();
        transform_cmd_flag_ = false;
        transform_thread_.join();
    }
    virtual UserCommand GetUserCommand(){return usr_cmd_;}

    void TransformRetroidToUserCommand();
    void SetMotionStateFeedback(const MotionStateFeedback& msfb){
        msfb_ = msfb;
        usr_cmd_.target_mode = msfb.current_state;
    }

    void PrintGamepadData(RetroidKeys *data){
        std::cout << "\nAxis value: \t";
        for(auto i : data->axis_values) std::cout << i << ",\t";
        std::cout << "\nShoulder keys: \t";
        std::cout << bool(data->L1) << ",\t" << bool(data->L2) << ",\t" << bool(data->R1) << ",\t" << bool(data->R2) << ",\t";
        std::cout << "\nLeft keys: \t";
        std::cout << bool(data->up) << ",\t" << bool(data->down) << ",\t" << bool(data->left) << ",\t" << bool(data->right) << ",\t";
        std::cout << "\nRight keys: \t";
        std::cout << bool(data->A) << ",\t" << bool(data->B) << ",\t" << bool(data->X) << ",\t" << bool(data->Y) << ",\t";
        std::cout << "\nStart keys: \t";
        std::cout << bool(data->select) << ",\t" << bool(data->start) << ",\t";
        std::cout << "\nAxis keys: \t";
        std::cout << bool(data->left_axis_button) << ",\t" << bool(data->right_axis_button) << ",\t";
        std::cout << "\n";
    }
};


void RetroidGamepadInterface::TransformRetroidToUserCommand(){
    while (transform_cmd_flag_) {
        rt_keys_ = gamepad_ptr_->GetKeys();
        // PrintGamepadData(&rt_keys_);
        if(first_flag_) {
            rt_keys_record_ = rt_keys_;
            first_flag_ = false;
            continue;
        }
        usr_cmd_.forward_vel_scale = rt_keys_.left_axis_y;
        usr_cmd_.side_vel_scale = -rt_keys_.left_axis_x;
        usr_cmd_.turnning_vel_scale = -rt_keys_.right_axis_x;
        if (!IsKeysEqual(rt_keys_, rt_keys_record_)) {
            switch (msfb_.current_state){
            case RobotMotionState::WaitingForStand:
                if(rt_keys_.Y != rt_keys_record_.Y) {
                    usr_cmd_.target_mode = int(RobotMotionState::StandingUp); 
                }
                break;
            case RobotMotionState::StandingUp:
                if(rt_keys_.A != rt_keys_record_.A){
                    usr_cmd_.target_mode = int(RobotMotionState::RLControlMode);
                }else if(rt_keys_.X != rt_keys_record_.X){
                    usr_cmd_.target_mode = int(RobotMotionState::LieDown);
                }
                break;
            case RobotMotionState::RLControlMode:
                if(rt_keys_.X != rt_keys_record_.X){
                    usr_cmd_.target_mode = int(RobotMotionState::LieDown);
                }
                break;
            
            default:
                break;
            }
            if(bool(rt_keys_.left_axis_button)&&bool(rt_keys_.right_axis_button)){
                usr_cmd_.target_mode = int(RobotMotionState::JointDamping);
            }
            rt_keys_record_ = rt_keys_;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

