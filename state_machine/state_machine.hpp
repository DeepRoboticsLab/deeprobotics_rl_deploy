/**
 * @file state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"
#include "idle_state.hpp"
#include "standup_state.hpp"
#include "joint_damping_state.hpp"
#include "liedown_state.hpp"

// #ifdef USE_ONNX
//     #include "rl_control_state_onnx.hpp"
// #else   
//     #include "rl_control_state.hpp"
// #endif

#include "rl_control_state_onnx.hpp"

#include "skydroid_gamepad_interface.hpp"
#include "retroid_gamepad_interface.hpp"
#include "keyboard_interface.hpp"
#include "xbox_gamepad_interface.hpp"
#include <fcntl.h>
#include <unistd.h>
#ifdef USE_RAISIM
    #include "simulation/jueying_raisim_simulation.hpp"
#endif
#ifdef USE_PYBULLET
    #include "simulation/simulation_interface.hpp"
#endif

#ifdef USE_MJCPP
    #include "simulation/mujoco_interface.hpp"
#endif

#include "hardware/hardware_interface.hpp"
#include "data_streaming.hpp"

class StateMachine{
private:
    std::shared_ptr<StateBase> current_controller_;
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> standup_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> liedown_controller_;

    StateName current_state_name_, next_state_name_;

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;

    std::shared_ptr<DataStreaming> ds_ptr_;

    void GetDataStreaming(){
        if(!ri_ptr_) return;
        VecXf pos = ri_ptr_->GetJointPosition();
        VecXf vel = ri_ptr_->GetJointVelocity();
        VecXf tau = ri_ptr_->GetJointTorque();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        MatXf jc = ri_ptr_->GetJointCommand();

        ds_ptr_->InsertInterfaceTime(ri_ptr_->GetInterfaceTimeStamp());
        ds_ptr_->InsertJointData("q", pos);
        ds_ptr_->InsertJointData("dq", vel);
        ds_ptr_->InsertJointData("tau", tau);
        ds_ptr_->InsertJointData("q_cmd", jc.col(1));
        ds_ptr_->InsertJointData("tau_ff", jc.col(4));

        ds_ptr_->InsertImuData("rpy", rpy);
        ds_ptr_->InsertImuData("acc", acc);
        ds_ptr_->InsertImuData("omg", omg);

        if(!uc_ptr_) return;
        auto cmd = uc_ptr_->GetUserCommand();
        ds_ptr_->InsertCommandData("target_mode", float(cmd.target_mode));

        ds_ptr_->InsertStateData("current_state", StateBase::msfb_.current_state);
       
        ds_ptr_->SendData();
    }

    std::shared_ptr<StateBase> GetNextStatePtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{
                return nullptr;
            }
            case StateName::kIdle:{
                return idle_controller_;
            }
            case StateName::kStandUp:{
                return standup_controller_;
            }
            case StateName::kRLControl:{
                return rl_controller_;
            }
            case StateName::kJointDamping:{
                return joint_damping_controller_;
            }
            case StateName::kLieDown:{
                return liedown_controller_;
            }
            default:{
                std::cerr << "error state name" << std::endl;
            }
        }
        return nullptr;
    }
public:
    StateMachine(RobotType robot_type){
        const std::string activation_key = "~/raisim/activation.raisim";
        std::string urdf_path = "";
        std::string mjcf_path = "";
        #ifdef BUILD_SIMULATION
            // 检测Xbox手柄是否存在
            std::string js_device = "/dev/input/js0";
            int js_fd = open(js_device.c_str(), O_RDONLY);
            if(js_fd >= 0){
                close(js_fd);
                std::cout << "Xbox gamepad detected, using XboxGamepadInterface" << std::endl;
                uc_ptr_ = std::make_shared<XboxGamepadInterface>(js_device);
            } else {
                std::cout << "No Xbox gamepad detected, using KeyboardInterface" << std::endl;
                uc_ptr_ = std::make_shared<KeyboardInterface>();
            }
        #else
            uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121);
        #endif
        // uc_ptr_ = std::make_shared<KeyboardInterface>();
        // uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121);
        if(robot_type == RobotType::Lite3){
            urdf_path = GetAbsPath()+"/../third_party/URDF_model/lite3_urdf/Lite3/urdf/Lite3.urdf";
            mjcf_path = GetAbsPath()+"third_party/URDF_model/Lite3/Lite3_mjcf/mjcf/Lite3.xml";
            #ifdef USE_RAISIM
                ri_ptr_ = std::make_shared<JueyingRaisimSimulation>(activation_key, urdf_path, "Lite3_sim");

            #elif defined(USE_MJCPP)
                ri_ptr_ = std::make_shared<MujocoInterface>("Lite3", mjcf_path);
                std::cout << "Using MujocoInterface CPP " << std::endl;
                std::cout << "mjcf_path: " << mjcf_path << std::endl;
            #elif defined(USE_PYBULLET)
                ri_ptr_ = std::make_shared<SimulationInterface>("Lite3");
            #else
                ri_ptr_ = std::make_shared<HardwareInterface>("Lite3");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_type);
        }else{
            std::cerr << "error" << std::endl;
        }

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        ds_ptr_ = std::make_shared<DataStreaming>(false, false);
        data_ptr->ds_ptr = ds_ptr_;

        idle_controller_ = std::make_shared<IdleState>(robot_type, "idle_state", data_ptr);
        standup_controller_ = std::make_shared<StandUpState>(robot_type, "standup_state", data_ptr);

        // 测试ONNX，后续需要改成参数控制
        // rl_controller_ = std::make_shared<RLControlState>(robot_type, "rl_control", data_ptr);
        // #ifdef USE_ONNX
        //     rl_controller_ = std::make_shared<RLControlStateONNX>(robot_type, "rl_control", data_ptr);
        // #else
        //     rl_controller_ = std::make_shared<RLControlState>(robot_type, "rl_control", data_ptr);
        // #endif
        rl_controller_ = std::make_shared<RLControlStateONNX>(robot_type, "rl_control", data_ptr);
        


        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_type, "joint_damping", data_ptr);
        liedown_controller_ = std::make_shared<LieDownState>(robot_type, "liedown_state", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;
   
        // std::cout << "Controller will be enabled in 3 seconds!!!" << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(3)); //for safety 

        ri_ptr_->Start();
        std::cout << "Robot interface started" << std::endl;
        uc_ptr_->Start();
        
        current_controller_->OnEnter();  
    }
    ~StateMachine(){}

    void Run(){
        int cnt = 0;
        static double time_record = 0;
        while(true){
            if(ri_ptr_->GetInterfaceTimeStamp()!= time_record){
                time_record = ri_ptr_->GetInterfaceTimeStamp();
                current_controller_ -> Run();
                
                if(current_controller_->LoseControlJudge()) next_state_name_ = StateName::kJointDamping;
                else next_state_name_ = current_controller_ -> GetNextStateName();
                
                if(next_state_name_ != current_state_name_){
                    current_controller_ -> OnExit();
                    std::cout << current_controller_ -> state_name_ << " ------------> ";
                    current_controller_ = GetNextStatePtr(next_state_name_);
                    std::cout << current_controller_ -> state_name_ << std::endl;
                    current_controller_ ->OnEnter();
                    current_state_name_ = next_state_name_; 
                }
                ++cnt;
                this->GetDataStreaming();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }

        uc_ptr_->Stop();
        ri_ptr_->Stop();
    }

};
