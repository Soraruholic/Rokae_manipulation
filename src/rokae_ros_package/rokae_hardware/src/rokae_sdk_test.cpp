#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <unistd.h>

#include <rokae/robot.h>
#include <rokae/data_types.h>
#include <rokae/motion_control_rt.h>


int main(int argc, char **argv){
    std::string local_ip="192.168.0.20";
    std::string robot_ip="192.168.0.160";
    std::shared_ptr<rokae::Cobot<7>> robot;
    std::shared_ptr<rokae::RtMotionControl<7>> rci;
    std::error_code ec;
    // std::vector<double> joint_position_state_;
    // joint_position_state_.resize(7, 0.0);
    // std::vector<double> joint_position_command_;
    // joint_position_command_.resize(7, 0.0);

    // Instantiation.
    try{
        robot = std::make_shared<rokae::Cobot<7>>(robot_ip, local_ip);
    }
    catch(const rokae::NetworkException& e){
        std::cout << e.what() << std::endl;
        return false;
    }

    // Connect to robot.
    try{
        robot->connectToRobot(ec);
        // ROS_INFO_STREAM("ec:"<< ec.value());
        if (ec.value() != 0) throw rokae::ExecutionException("Connection failed", "连接失败");
    }
    catch(const rokae::ExecutionException& e){
        std::cout<<"Robot connection failed:"<<e.what()<<std::endl;
        return false;
    }
    std::cout<<"连接完成"<<std::endl;


    // 失败 拖不动
    // robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    // robot->setOperateMode(rokae::OperateMode::manual, ec);
    // robot->setPowerState(false, ec); // 打开拖动之前，需要机械臂处于手动模式下电状态
    // // 笛卡尔空间，自由拖动
    // robot->enableDrag(rokae::DragParameter::cartesianSpace, rokae::DragParameter::freely, ec);
    // std::cout<<"打开拖动"<< ec<< "按回车继续"<<std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
    // while(getchar() != '\n');
    // robot->disableDrag(ec);
    // std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
    // return 0;

    // 失败 ec:-259
    // robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    // robot->setOperateMode(rokae::OperateMode::manual, ec); // 手动模式下jog
    // std::cout<< "准备Jog机器人, 需手动模式上电, 请确认已上电后按回车键"<<std::endl;
    // while(getchar() != '\n');
    // // 对于有外接使能开关的情况，需要按住开关手动上电
    // robot->setPowerState(true, ec);
    // std::cout<< "-- 开始Jog机器人-- \n世界坐标系下, 沿Z+方向运动50mm, 速率50%，等待机器人停止运动后按回车继续"<<std::endl;
    // robot->startJog(rokae::JogOpt::world, 0.5, 50, 2, true, ec);
    // std::cout<<"ec:"<<ec<<std::endl;//-259
    // while(getchar() != '\n');
    // std::cout<< "轴空间，6轴负向连续转动，速率5%，按回车停止Jog"<<std::endl;
    // robot->startJog(rokae::JogOpt::jointSpace, 0.05, 5000, 5, false, ec);
    // std::cout<<"ec:"<<ec<<std::endl;
    // while(getchar() != '\n'); // 按回车停止
    // robot->stop(ec); // jog结束必须调用stop()停止
    // return 0;


    // 失败 不动
    // robot->setOperateMode(rokae::OperateMode::automatic, ec);
    // robot->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
    // robot->setPowerState(true, ec);
    // try {
    //     auto rtCon = robot->getRtMotionController().lock();

    //     // 可选: 设置要接收数据
    //     rtCon->startReceiveRobotState({rokae::RtSupportedFields::jointPos_m});
    //     bool init = true;
    //     double time = 0;

    //     std::array<double, 7> jntPos{};
    //     std::array<double, 7> q_home = {0, 0, 0, 0, 0, 0,0};

    //     // 从当前位置MoveJ运动到拖拽位姿
    //     rtCon->MoveJ(0.1, robot->jointPos(ec), q_home);

    //     std::function<rokae::JointPosition()> callback = [&, rtCon](){
    //         if(init) {
    //             jntPos = robot->jointPos(ec);
    //             init = false;
    //         }
    //         time += 0.001;
    //         double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / 2.5 * time));
    //         rokae::JointPosition cmd = {{jntPos[0] + delta_angle, jntPos[1] + delta_angle,
    //                                 jntPos[2] - delta_angle,
    //                                 jntPos[3] + delta_angle, jntPos[4] - delta_angle,
    //                                 jntPos[5] + delta_angle}};
    //         std::cout<<"move to";
    //         for(int i=0;i<7;i++){
    //             std::cout<<cmd.joints[i]<<",";
    //         }
    //         std::cout<<std::endl;
    //         if(time > 60) {
    //             cmd.setFinished(); // 60秒后结束
    //         }
    //         return cmd;
    //     };

    //     rtCon->startMove(rokae::RtControllerMode::jointPosition);
    //     // 开始轴空间位置控制
    //     rtCon->setControlLoop(callback);

    //     // 阻塞loop
    //     rtCon->startLoop(true);
    //     std::cout<< "控制结束"<<std::endl;

    // } catch (const std::exception &e) {
    //     std::cout<<"Exception - "<<e.what()<<std::endl;
    // }
    // return 0;


    // Enable the robot.
    try{
        robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        robot->setOperateMode(rokae::OperateMode::automatic, ec);
        robot->setPowerState(true, ec);
        // robot->setRtNetworkTolerance(50, ec);
        std::cout<<"ec:"<< ec.value()<<std::endl;
        if (ec.value() != 0) throw rokae::ExecutionException("Enable failed", "上电失败");
        
    }
    catch(const rokae::ExecutionException& e){
        std::cout<<"Could not enable robot:"<<e.what()<<std::endl;
        return false;
    }
    std::cout<<"上电完成"<<std::endl;

    // Get joint position
    auto robot_joint_postion = robot->jointPos(ec);
    std::cout<<"======================================="<<std::endl;
    for(std::size_t i = 0; i < 7; i++){
        std::cout<<robot_joint_postion[i]<<std::endl;
    }

    // Set realtime control.
    try
    {
        robot->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        rci = robot->getRtMotionController().lock();

        rci->startReceiveRobotState({rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m, 
                                    rokae::RtSupportedFields::tau_m, rokae::RtSupportedFields::tauExt_inBase,
                                    rokae::RtSupportedFields::tauExt_inStiff});
    }
    catch(const rokae::ExecutionException& e)
    {
        std::cout<<"Could not set realtime control mode:"<<e.what()<<std::endl;
        return false;
    }

    std::cout<<"rt is ready!"<<std::endl;           

    for(int i = 0; i < 10; i++){
        sleep(1);
        // update robot state
        rci->updateRobotState();

        std::array<double, 7> robot_joint_postion;
        std::array<double, 7> robot_joint_velocity;
        std::array<double, 7> robot_joint_torque;  
        // fixed size
        std::array<double, 6> robot_ext_force_base; 
        std::array<double, 6> robot_ext_force_stiff; 

        // Get joint position, velocity and torque.
        int pos_flag = rci->getStateData("q_m",  robot_joint_postion);
        int vel_flag = rci->getStateData(rokae::RtSupportedFields::jointVel_m,  robot_joint_velocity);
        int tor_flag = rci->getStateData("tau_m",  robot_joint_torque);
        int ext_base = rci->getStateData(rokae::RtSupportedFields::tauExt_inBase,  robot_ext_force_base);
        int ext_stiff = rci->getStateData(rokae::RtSupportedFields::tauExt_inStiff,  robot_ext_force_stiff);

        // std::cout<<"pos:";
        // for(int i=0;i<7;i++){
        //     std::cout<<robot_joint_postion[i]<<",";
        // }
        // std::cout<<std::endl;
        // std::cout<<"vel:";
        // for(int i=0;i<7;i++){
        //     std::cout<<robot_joint_velocity[i]<<",";
        // }
        // std::cout<<std::endl;
        std::cout<<"tau:";
        for(int i=0;i<6;i++){
            std::cout<<robot_ext_force_base[i]<<",";
        }
        std::cout<<std::endl;
    }
    return 0;
}
