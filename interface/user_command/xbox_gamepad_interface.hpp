#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include <cstdio>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#define AXIS_STEP 0.1
#define JOYSTICK_DEADZONE 0.1

using namespace interface;
using namespace types;

class XboxGamepadInterface : public UserCommandInterface
{
    /**
     * @brief Xbox手柄按键映射说明
     * 
     * 左摇杆 X轴 → A/D键 (左右平移速度)
     * 左摇杆 Y轴 → W/S键 (前后移动速度)
     * 右摇杆 X轴 → Q/E键 (转向速度)
     * 左肩键 LB → Z键 (站立)
     * 右肩键 RB → C键 (切换到RL控制模式)
     * 左扳机 LT → R键 (关节阻尼模式)
     */
private:
    UserCommand usr_cmd_;
    bool start_thread_flag_;
    std::thread gp_thread_;
    int js_fd_;
    std::mutex mtx_;

    struct GamepadState {
        float left_axis_x = 0.0f;
        float left_axis_y = 0.0f;
        float right_axis_x = 0.0f;
        float right_axis_y = 0.0f;
        bool lb_pressed = false;
        bool rb_pressed = false;
        bool lt_pressed = false;
        bool x_pressed = false;
    } gp_state_;
    
    bool connection_mode_detected_ = false;
    bool is_bluetooth_ = false;

    void ClipNumber(float &num, float low, float up){
        if(low > up) std::cerr << "error clip" << std::endl;
        if(num < low) num = low;
        if(num > up) num = up;
    }

    float ApplyDeadzone(float value, float deadzone = JOYSTICK_DEADZONE){
        if(std::abs(value) < deadzone) return 0.0f;
        return (value - std::copysign(deadzone, value)) / (1.0f - deadzone);
    }

    double GetCurrentTimeStamp(){
        static timespec startup_timestamp;
        timespec now_timestamp;
        if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
            clock_gettime(CLOCK_MONOTONIC,&startup_timestamp);
        }
        clock_gettime(CLOCK_MONOTONIC,&now_timestamp);
        return (now_timestamp.tv_sec-startup_timestamp.tv_sec)*1e3 
            + (now_timestamp.tv_nsec-startup_timestamp.tv_nsec)/1e6;
    }

public:
    XboxGamepadInterface(const std::string& js_device = "/dev/input/js0"){
        std::memset(&usr_cmd_, 0, sizeof(usr_cmd_));
        js_fd_ = -1;
        std::cout << "Using Xbox Gamepad Command Interface" << std::endl;
        std::cout << "Opening device: " << js_device << std::endl;
        
        js_fd_ = open(js_device.c_str(), O_RDONLY | O_NONBLOCK);
        if(js_fd_ < 0){
            std::cerr << "Failed to open gamepad device: " << js_device << std::endl;
            std::cerr << "Make sure the gamepad is connected and you have permission to access it." << std::endl;
        } else {
            std::cout << "Gamepad opened successfully!" << std::endl;
            DetectConnectionMode();
        }
    }

    void DetectConnectionMode(){
        js_event js_ev;
        std::cout << "[Gamepad] Detecting connection mode..." << std::endl;
        std::cout << "[Gamepad] Please move any stick or press any button..." << std::endl;
        
        while(connection_mode_detected_ == false){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            while(read(js_fd_, &js_ev, sizeof(js_event)) == sizeof(js_event)){
                if(js_ev.type == JS_EVENT_AXIS){
                    if(js_ev.number == 2){
                        if(std::abs(js_ev.value) > 30000){
                            is_bluetooth_ = false;
                            connection_mode_detected_ = true;
                            std::cout << "[Gamepad] Detected USB connection mode" << std::endl;
                        } else if(std::abs(js_ev.value) > 100){
                            is_bluetooth_ = true;
                            connection_mode_detected_ = true;
                            std::cout << "[Gamepad] Detected Bluetooth connection mode" << std::endl;
                        }
                    }
                } else if(js_ev.type == JS_EVENT_BUTTON){
                    if(js_ev.number == 6 || js_ev.number == 7){
                        is_bluetooth_ = true;
                        connection_mode_detected_ = true;
                        std::cout << "[Gamepad] Detected Bluetooth connection mode" << std::endl;
                    } else if(js_ev.number == 2 || js_ev.number == 3){
                        is_bluetooth_ = false;
                        connection_mode_detected_ = true;
                        std::cout << "[Gamepad] Detected USB connection mode" << std::endl;
                    }
                }
            }
        }
    }

    ~XboxGamepadInterface(){
        if(js_fd_ >= 0) close(js_fd_);
    }

    virtual void Start(){
        if(js_fd_ < 0){
            std::cerr << "Cannot start gamepad thread: device not opened" << std::endl;
            return;
        }
        start_thread_flag_ = true;
        gp_thread_ = std::thread(std::bind(&XboxGamepadInterface::Run, this));
    }

    virtual void Stop(){
        start_thread_flag_ = false;
        if(gp_thread_.joinable()){
            gp_thread_.join();
        }
    }

    virtual UserCommand GetUserCommand() override {
        std::lock_guard<std::mutex> lock(mtx_);
        return usr_cmd_;
    }

    virtual void SetMotionStateFeedback(const MotionStateFeedback& msfb){
        std::lock_guard<std::mutex> lock(mtx_);
        msfb_ = msfb;
        usr_cmd_.target_mode = msfb.current_state;
    }

    void Run(){
        js_event js_ev;
        double forward_time_record = GetCurrentTimeStamp();
        double side_time_record = GetCurrentTimeStamp();
        double turning_time_record = GetCurrentTimeStamp();
        
        bool lb_prev = false;
        bool rb_prev = false;
        bool lt_prev = false;
        bool x_prev = false;

        std::cout << "Start Gamepad Listening" << std::endl;
        
        while (start_thread_flag_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            while(read(js_fd_, &js_ev, sizeof(js_event)) == sizeof(js_event)){
                std::lock_guard<std::mutex> lock(mtx_);
                
                switch(js_ev.type & ~JS_EVENT_INIT){
                    case JS_EVENT_AXIS:
                        if(is_bluetooth_){
                            switch(js_ev.number){
                                case 0: // Left stick X
                                    gp_state_.left_axis_x = ApplyDeadzone(js_ev.value / 32767.0f);
                                    break;
                                case 1: // Left stick Y
                                    gp_state_.left_axis_y = ApplyDeadzone(js_ev.value / 32767.0f);
                                    break;
                                case 2: // Right stick X
                                    gp_state_.right_axis_x = ApplyDeadzone(js_ev.value / 32767.0f);
                                    break;
                                case 3: // Right stick Y
                                    gp_state_.right_axis_y = ApplyDeadzone(js_ev.value / 32767.0f);
                                    break;
                                case 4: // RT (Right Trigger)
                                    break;
                                case 5: // LT (Left Trigger)
                                    gp_state_.lt_pressed = (js_ev.value > 10000);
                                    break;
                            }
                        } else {
                            switch(js_ev.number){
                                case 0: // Left stick X
                                    gp_state_.left_axis_x = ApplyDeadzone(js_ev.value / 32767.0f);
                                    break;
                                case 1: // Left stick Y
                                    gp_state_.left_axis_y = ApplyDeadzone(js_ev.value / 32767.0f);
                                    break;
                                case 2: // LT (Left Trigger)
                                    gp_state_.lt_pressed = (js_ev.value > 10000);
                                    break;
                                case 3: // Right stick Y
                                    gp_state_.right_axis_y = ApplyDeadzone(js_ev.value / 32767.0f);
                                    break;
                                case 4: // Right stick X
                                    gp_state_.right_axis_x = ApplyDeadzone(js_ev.value / 32767.0f);
                                    break;
                                case 5: // RT (Right Trigger)
                                    break;
                            }
                        }
                        break;
                        
                    case JS_EVENT_BUTTON:
                        if(is_bluetooth_){
                            switch(js_ev.number){
                                case 0: // A
                                    break;
                                case 1: // B
                                    break;
                                case 3: // X
                                    gp_state_.x_pressed = (js_ev.value != 0);
                                    break;
                                case 4: // Y
                                    break;
                                case 6: // LB (Left Bumper)
                                    gp_state_.lb_pressed = (js_ev.value != 0);
                                    break;
                                case 7: // RB (Right Bumper)
                                    gp_state_.rb_pressed = (js_ev.value != 0);
                                    break;
                                case 11: // Menu button
                                    break;
                                case 10: // View button
                                    break;
                            }
                        } else {
                            switch(js_ev.number){
                                case 0: // A
                                    break;
                                case 1: // B
                                    break;
                                case 2: // X
                                    gp_state_.x_pressed = (js_ev.value != 0);
                                    break;
                                case 3: // Y
                                    break;
                                case 4: // LB (Left Bumper)
                                    gp_state_.lb_pressed = (js_ev.value != 0);
                                    break;
                                case 5: // RB (Right Bumper)
                                    gp_state_.rb_pressed = (js_ev.value != 0);
                                    break;
                                case 6: // View button
                                    break;
                                case 7: // Menu button
                                    break;
                            }
                        }
                        break;
                }
            }
            
            std::lock_guard<std::mutex> lock(mtx_);
            
            switch(msfb_.current_state) {
                case RobotMotionState::WaitingForStand:
                    if(gp_state_.lb_pressed && !lb_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::StandingUp);
                        std::cout << "[Gamepad] LB pressed -> StandingUp" << std::endl;
                    }
                    break;
                    
                case RobotMotionState::StandingUp:
                    if(gp_state_.rb_pressed && !rb_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::RLControlMode);
                        std::cout << "[Gamepad] RB pressed -> RLControlMode" << std::endl;
                    }else if(gp_state_.x_pressed && !x_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::LieDown);
                        std::cout << "[Gamepad] X pressed -> LieDown" << std::endl;
                    }
                    break;
                    
                case RobotMotionState::RLControlMode: {
                    if(gp_state_.lt_pressed && !lt_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::JointDamping);
                        std::cout << "[Gamepad] LT pressed -> JointDamping" << std::endl;
                    }else if(gp_state_.x_pressed && !x_prev){
                        usr_cmd_.target_mode = int(RobotMotionState::LieDown);
                        std::cout << "[Gamepad] X pressed -> LieDown" << std::endl;
                    }
                    
                    double current_time = GetCurrentTimeStamp();
                    
                    float forward_input = -gp_state_.left_axis_y;
                    float side_input = gp_state_.left_axis_x;
                    float turning_input = -gp_state_.right_axis_x;
                    
                    if(std::abs(forward_input) > 0.01){
                        usr_cmd_.forward_vel_scale = forward_input;
                        forward_time_record = current_time;
                    }
                    
                    if(std::abs(side_input) > 0.01){
                        usr_cmd_.side_vel_scale = side_input;
                        side_time_record = current_time;
                    }
                    
                    if(std::abs(turning_input) > 0.01){
                        usr_cmd_.turnning_vel_scale = turning_input;
                        turning_time_record = current_time;
                    }
                    
                    if(current_time - forward_time_record > 300.) usr_cmd_.forward_vel_scale = 0;
                    if(current_time - side_time_record > 300.) usr_cmd_.side_vel_scale = 0;
                    if(current_time - turning_time_record > 300.) usr_cmd_.turnning_vel_scale = 0;

                    ClipNumber(usr_cmd_.forward_vel_scale, -1., 1.);
                    ClipNumber(usr_cmd_.side_vel_scale, -1., 1.);
                    ClipNumber(usr_cmd_.turnning_vel_scale, -1., 1.);
                    break;
                }
                    
                default:
                    break;
            }
            
            lb_prev = gp_state_.lb_pressed;
            rb_prev = gp_state_.rb_pressed;
            lt_prev = gp_state_.lt_pressed;
            x_prev = gp_state_.x_pressed;
        }
        
        std::cout << "Gamepad thread stopped" << std::endl;
    }
};
