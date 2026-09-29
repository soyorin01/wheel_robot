/*
 * ROS2 IMU数据发布节点
 * 该节点创建一个定时器，以50Hz频率发布模拟的IMU数据
 * 消息类型：sensor_msgs/msg/Imu
 * 话题名称：/imu/data
 * 坐标系：imu_link
 * 新增：串口任务执行线程
 * 新增：机器人运动控制订阅
 * 新增：关节状态订阅
 * 新增：里程计数据发布
 */

// ROS2核心库
#include <rclcpp/rclcpp.hpp>
// IMU消息类型
#include <sensor_msgs/msg/imu.hpp>
// 运动控制消息类型
#include <geometry_msgs/msg/twist.hpp>
// 关节状态消息类型
#include <sensor_msgs/msg/joint_state.hpp>
// 里程计消息类型
#include <nav_msgs/msg/odometry.hpp>
// 数学函数库（sin, cos等）
#include <cmath>
// 时间处理库
#include <chrono>
// 线程支持
#include <thread>
// 互斥锁
#include <mutex>
// 包含关节轨迹消息的头文件，用于控制机器人关节运动
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <algorithm> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <iostream>
#include <vector>
#include <deque>
#include <cstring>
#include <string>
#include <stdexcept>
#include <termios.h>  // 串口波特率定义
#include <limits>
#include <unistd.h>   // 新增：用于close()函数
#include <fcntl.h>    // 新增：用于文件控制操作
#include <errno.h>    // 新增：用于错误处理
#include "robot_functions.h"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "motor_temperature_pkg/msg/motor_temperatures.hpp"

// Match the lower controller's Model_Selection=0 default height so entering
// ROS mode does not cause a sudden leg-height step.
const double Default_Height = 0.25f;
const double LEG_MIN_LOW = 0.14f;   //腿最低
const double LEG_MAX_HIGH = 0.36f;   //腿最高
const double  Pitch_limits  = 35.0f; 
const double  Roll_limits  = 15.0f; 
const float  Timeout_threshold  = 1.0f; //订阅话题未更新超时
const float  Cmd_Vel_Timeout_Threshold = 0.30f; // /cmd_vel 超时后自动停车
const float  Default_Motor_Temperature_Warning_C = 60.5f;
const float  Default_Jump_Min_Height = 0.18f;
const float  Default_Jump_Max_Height = 0.30f;
const float  ROS_JUMP_MIN_HEIGHT_LOW = 0.16f;
const float  ROS_JUMP_MIN_HEIGHT_HIGH = 0.24f;
const float  ROS_JUMP_MAX_HEIGHT_LOW = 0.24f;
const float  ROS_JUMP_MAX_HEIGHT_HIGH = 0.34f;
const float  ROS_JUMP_PROFILE_TIMEOUT = 1.0f;
constexpr uint16_t ROS_SET_JUMP = 0x0001;
constexpr uint16_t ROS_SET_JUMP_PROFILE = 0x0002;
constexpr uint16_t ROS_SET_SLIDE_MASK = 0xFF00;
constexpr float ROS_SLIDE_LIMIT = 0.08f;

uint16_t encode_slide_into_set(uint16_t set, float slide_m)
{
  const float clipped = std::clamp(slide_m, -ROS_SLIDE_LIMIT, ROS_SLIDE_LIMIT);
  const int encoded = std::clamp(
      static_cast<int>(std::lround(clipped / ROS_SLIDE_LIMIT * 127.0f)),
      -127,
      127);
  const auto raw = static_cast<uint8_t>(static_cast<int8_t>(encoded));
  return static_cast<uint16_t>(
      (set & static_cast<uint16_t>(~ROS_SET_SLIDE_MASK)) |
      (static_cast<uint16_t>(raw) << 8));
}

typedef struct {
    rclcpp::Time current_time;
    rclcpp::Time  last_time;
    float dt;
  } Time_dt_t;



// 使用时间字面量（如20ms）
using namespace std::chrono_literals;

//腿轮类，继承自rclcpp::Node
class Legwheel : public rclcpp::Node
{
public:
  // 构造函数
  Legwheel()
  : Node("wl_base_node"),  // 节点名称
    cmd_updated_vel_(false),         // 控制指令更新标志（先声明）
    cmd_posture_updated_sign_(false),
    Robot_Mode_(0),
    cmd_joint_updated_sign_(false),
    stop_serial_task_(false),    // 初始化停止标志（后声明）
    serial_fd_(-1),              // 初始化串口文件描述符
    serial_selected_(false),      // 串口选择标志
    last_odom_time_(this->get_clock()->now())  // 初始化里程计时间
  {
    // ros2 run without parameters keeps the original interactive selector.
    // A launch file can pass serial_port to start unattended.
    const std::string configured_port =
      this->declare_parameter<std::string>("serial_port", "");

    if (configured_port.empty())
    {
      selectSerialPort();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "正在打开参数指定的串口: %s",
                  configured_port.c_str());
      serial_fd_ = openSerialPort(configured_port, B230400);
      if (serial_fd_ != -1)
      {
        serial_selected_ = true;
        RCLCPP_INFO(this->get_logger(), "成功打开串口 %s，波特率 230400",
                    configured_port.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "无法打开参数指定的串口: %s",
                     configured_port.c_str());
      }
    }
    
    // 如果串口选择成功，初始化ROS2组件
    if (serial_selected_) 
    {
      initROSComponents();
    } 
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "串口选择失败，节点无法正常启动");
      throw std::runtime_error("底盘串口打开失败");
    }
  }

  // 析构函数
  ~Legwheel()
  {
    // ===================== 安全停止串口线程 =====================
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      stop_serial_task_ = true;
    }
    
    if (serial_thread_.joinable()) {
      serial_thread_.join();
      RCLCPP_INFO(this->get_logger(), "Serial task thread stopped");
    }

    send_stop_command_frames();
    
    // 关闭串口
    if (serial_fd_ != -1) {
      close(serial_fd_);
      serial_fd_ = -1;
      RCLCPP_INFO(this->get_logger(), "串口已关闭");
    }
  }

private:
  // 串口选择函数
  void selectSerialPort()
  {
    RCLCPP_INFO(this->get_logger(), "开始串口选择...");
    
    std::vector<std::string> ports = listSerialPorts();
    int selectedPortIndex = -1;
    char choice;
    bool quitProgram = false;
    bool restartSelection = true;

    // 主选择循环：处理串口选择
    while (!quitProgram && restartSelection) 
    {
        restartSelection = false;
        
        // 处理无可用串口情况
        if (ports.empty()) 
        {
            std::cout << "\n未找到可用串口!" << std::endl;
            std::cout << "按 [R] 重新扫描或 [Q] 退出: ";
            std::cin >> choice;
            choice = std::toupper(choice);
            
            if (choice == 'R') {
                ports = listSerialPorts();
                restartSelection = true;
                continue;
            } else if (choice == 'Q') {
                quitProgram = true;
                break;
            }
        }
        
        // 显示菜单并获取用户选择
        std::cout << "\n可用串口列表:" << std::endl;
        for (size_t i = 0; i < ports.size(); i++) {
            std::cout << "[" << i+1 << "] " << ports[i] << std::endl;
        }
        std::cout << "[R] 重新扫描" << std::endl;
        std::cout << "[Q] 退出" << std::endl;
        std::cout << "请选择串口: ";
        std::cin >> choice;
        choice = std::toupper(choice);
        
        // 处理用户选择
        if (choice == 'Q') {
            quitProgram = true;
        } 
        else if (choice == 'R') {
            ports = listSerialPorts();
            restartSelection = true;
        } 
        else if (std::isdigit(choice)) {
            // 将字符转换为数字索引
            int index = choice - '1';
            
            // 验证索引是否有效
            if (index >= 0 && static_cast<size_t>(index) < ports.size()) {
                selectedPortIndex = index;
                std::cout << "\n正在打开: " << ports[selectedPortIndex] << "..." << std::endl;
                
                // CH340K on the controller Type-C port supports up to 230400 baud.
                serial_fd_ = openSerialPort(ports[selectedPortIndex], B230400);
                
                if (serial_fd_ != -1) {
                    std::cout << "成功打开串口: " << ports[selectedPortIndex] << std::endl;
                    std::cout << "波特率: 230400" << std::endl;
                    serial_selected_ = true;
                    restartSelection = false; // 成功打开，退出选择循环
                } else {
                    std::cout << "打开串口失败，请重试或选择其他串口" << std::endl;
                    restartSelection = true; // 重新显示选择菜单
                }
            } else {
                std::cout << "无效选择! 请重新输入。" << std::endl;
                restartSelection = true;
            }
        } 
        else 
        {
            std::cout << "无效选择! 请重新输入。" << std::endl;
            restartSelection = true;
        }
        
        // 清除输入缓冲区
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    
    if (quitProgram) 
    {
        RCLCPP_INFO(this->get_logger(), "用户选择退出程序");
        // 这里可以添加适当的退出逻辑
    }
  }
  
  // 初始化ROS2组件
  void initROSComponents()
  {
    // Start from a fully defined, stationary command. The original code left
    // these POD structures uninitialized until callbacks/timeouts ran, which
    // could put indeterminate values on the wire during node startup.
    get_ComWheelLegged = {};
    get_ComWheelLegged.linear_velocity = 0.0f;
    get_ComWheelLegged.angle_velocity = 0.0f;
    get_ComWheelLegged.leg_length = Default_Height;
    get_ComWheelLegged.pitch_angle = 0.0f;
    get_ComWheelLegged.roll_angle = 0.0f;
    get_ComWheelLegged.jump_min_height = Default_Jump_Min_Height;
    get_ComWheelLegged.jump_max_height = Default_Jump_Max_Height;
    ComWheelLegged = get_ComWheelLegged;

    // 创建发布者，话题名为/imu/data，队列大小为10
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    
    // ===================== 新增：里程计发布者 =====================
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    
    // 创建发布者，发布到"sbus_channels"话题，队列大小为10
    sbus_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/sbus_channels", 10);    //遥控器10通道原始数据
    diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/robot_diagnostics", 10);
    pid_tune_telemetry_publisher_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/pid_tuning/telemetry", 10);
    motor_temperature_warning_threshold_ =
      this->declare_parameter<double>(
        "motor_temperature_warning_threshold_c",
        Default_Motor_Temperature_Warning_C);
    motor_temperature_publisher_ =
      this->create_publisher<motor_temperature_pkg::msg::MotorTemperatures>(
        "/motor_temperatures", 10);


    // 初始化关节名称
    g_joint_names = {
        "left_front_joint_link",
        "left_back_joint_link",
        "right_front_joint_link",
        "right_back_joint_link",
        "left_wheel_joint_link",
        "right_wheel_joint_link"
    };


    // 验证关节数量
    if (g_joint_names.size() != MotorID::TOTAL_COUNT) {
        RCLCPP_INFO(this->get_logger(), "关节名称数量与电机总数不匹配！");
    }
    // 创建发布者
    g_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/robot_joint_states", 10);


    // ===================== 新增：TF变换广播器 =====================
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);//


    // 创建订阅者，订阅"joint_commands"话题     /cmd_robot_joint
    cmd_joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/cmd_robot_joint",10,std::bind(&Legwheel::joint_control_callback, this, std::placeholders::_1)//订阅关节控制
    );


    // ===================== 新增：运动控制订阅 =====================
    // 创建订阅者，话题名为/cmd_vel，队列大小为10
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&Legwheel::cmd_vel_callback, this, std::placeholders::_1));//控制机器人 X线速度（米每秒）  Z角速度（弧度每秒）
    
    // ===================== 新增：关节状态订阅 =====================
    // 创建订阅者，话题名为/cmd_posture，队列大小为10
    body_posture_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/cmd_posture", 10, std::bind(&Legwheel::joint_state_callback, this, std::placeholders::_1));//控制机器人俯仰（度） 横滚（度） 高度(米)

    // One-shot actions carried in ComWheelLegged_t::set.
    cmd_jump_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/cmd_jump", 10, std::bind(&Legwheel::cmd_jump_callback, this, std::placeholders::_1));
    cmd_jump_profile_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/cmd_jump_profile", 10,
        std::bind(&Legwheel::cmd_jump_profile_callback, this, std::placeholders::_1));
    pid_tune_command_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/pid_tuning/command", 10,
        std::bind(&Legwheel::pid_tune_command_callback, this, std::placeholders::_1));
    
    // 创建定时器
    timer_ = this->create_wall_timer(
      20ms, std::bind(&Legwheel::timer_callback, this));//定时发布IMU 和里程计数据

    timer2_ = this->create_wall_timer(
      10ms,std::bind(&Legwheel::timer2_callback, this)//给下位机定时发送数据 定时发布IMU 和里程计数据
    );
      

    
    // ===================== 串口任务初始化 =====================
    // 启动串口任务线程
    serial_thread_ = std::thread(&Legwheel::serial_task, this);
    
    // 初始化关节数据
    joint_positions_ = {Default_Height, 0.0, 0.0};
    joint_velocities_ = {0.0, 0.0, 0.0};
    joint_efforts_ = {0.0, 0.0, 0.0};
    cmd_posture_updated_sign_ = false;



    cmd_joint_updated_sign_ = false;
    
    //初始里程计信息
    odom.angle_speed = 0;
    odom.linear_speed = 0;
    odom.angle = 0;  
    odom.x = 0;
    odom.y = 0;
    cmd_updated_vel_ = false;

    COD_vel_dt.current_time = this->get_clock()->now(); 
    COD_vel_dt.last_time = COD_vel_dt.current_time;

    COD_posture_dt.current_time = this->get_clock()->now();
    COD_posture_dt.last_time = COD_posture_dt.current_time;
    
    TxdMitData_init();

    // 输出启动信息
    RCLCPP_INFO(this->get_logger(), "IMU发布话题: /imu/data");
    RCLCPP_INFO(this->get_logger(), "里程计发布话题: /odom");
    RCLCPP_INFO(this->get_logger(), "遥控器发布话题: /sbus_channels");
    RCLCPP_INFO(this->get_logger(), "机器人自检话题: /robot_diagnostics");
    RCLCPP_INFO(this->get_logger(), "PID调参遥测话题: /pid_tuning/telemetry");
    RCLCPP_INFO(this->get_logger(), "电机温度发布话题: /motor_temperatures");
    RCLCPP_INFO(this->get_logger(), "机器人关节状态发布话题: /robot_joint_states");


    RCLCPP_INFO(this->get_logger(), "移动底盘速度控制订阅话题：/cmd_vel");
    RCLCPP_INFO(this->get_logger(), "移动底盘姿态控制订阅话题：/cmd_posture");
    RCLCPP_INFO(this->get_logger(), "跳跃高度配置订阅话题：/cmd_jump_profile");
    RCLCPP_INFO(this->get_logger(), "机器人关节控制订阅话题：/cmd_robot_joint");
    RCLCPP_INFO(this->get_logger(), "PID调参命令话题：/pid_tuning/command");
    
  }


  void TXD_MobileChassis(void)//发送移动底盘控制数据
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    const size_t dataSize = sizeof(ComWheelLegged_t); 
    
    get_ComWheelLegged.header1 = 123;         // 帧头1 (123)
    get_ComWheelLegged.header2 = 45;          // 帧头2 (45)
    get_ComWheelLegged.length = dataSize;     // 结构体长度

    ComWheelLegged.header1 = 123;             // 帧头1 (123)
    ComWheelLegged.header2 = 45;              // 帧头2 (45)
    ComWheelLegged.length = dataSize;         // 结构体长度

    if ((get_ComWheelLegged.set & ROS_SET_JUMP_PROFILE) != 0 &&
        jump_profile_last_update_.nanoseconds() > 0 &&
        (this->now() - jump_profile_last_update_).seconds() >
          ROS_JUMP_PROFILE_TIMEOUT)
    {
      get_ComWheelLegged.set &= static_cast<uint16_t>(~ROS_SET_JUMP_PROFILE);
      get_ComWheelLegged.jump_min_height = Default_Jump_Min_Height;
      get_ComWheelLegged.jump_max_height = Default_Jump_Max_Height;
      cmd_jump_profile_updated_sign_ = true;
    }

    if((cmd_posture_updated_sign_ == true)&&(cmd_updated_vel_ == true))
    {
      ComWheelLegged = get_ComWheelLegged;
      get_ComWheelLegged.count++;
      cmd_posture_updated_sign_ = false;
      cmd_updated_vel_ = false;

    }
    else if((cmd_posture_updated_sign_ == true)&&(cmd_updated_vel_ == false))
    {

      ComWheelLegged.leg_length = get_ComWheelLegged.leg_length;
      ComWheelLegged.roll_angle = get_ComWheelLegged.roll_angle;
      ComWheelLegged.pitch_angle = get_ComWheelLegged.pitch_angle; 
      ComWheelLegged.count = get_ComWheelLegged.count; 
      get_ComWheelLegged.count++;
      cmd_posture_updated_sign_ = false;

    }
    else if((cmd_posture_updated_sign_ == false)&&(cmd_updated_vel_ == true))
    {
      ComWheelLegged.linear_velocity = get_ComWheelLegged.linear_velocity;
      ComWheelLegged.angle_velocity = get_ComWheelLegged.angle_velocity;
      ComWheelLegged.count = get_ComWheelLegged.count; 
      get_ComWheelLegged.count++;
      cmd_updated_vel_ = false;
    }

    // Bit 0 is a one-shot jump request.  Copying it independently from the
    // velocity/posture update flags also lets a standalone /cmd_jump publisher
    // reach the lower controller.
    if (ComWheelLegged.set != get_ComWheelLegged.set)
    {
      ComWheelLegged.set = get_ComWheelLegged.set;
      ComWheelLegged.count = get_ComWheelLegged.count;
      get_ComWheelLegged.count++;
    }

    if (cmd_jump_profile_updated_sign_)
    {
      ComWheelLegged.jump_min_height = get_ComWheelLegged.jump_min_height;
      ComWheelLegged.jump_max_height = get_ComWheelLegged.jump_max_height;
      ComWheelLegged.count = get_ComWheelLegged.count;
      get_ComWheelLegged.count++;
      cmd_jump_profile_updated_sign_ = false;
    }

    // 计算实际时间间隔
    COD_vel_dt.current_time = this->get_clock()->now();
    COD_vel_dt.dt = (COD_vel_dt.current_time - COD_vel_dt.last_time).seconds();
    

    COD_posture_dt.current_time = this->get_clock()->now();
    COD_posture_dt.dt = (COD_posture_dt.current_time - COD_posture_dt.last_time).seconds();

      
    if((COD_vel_dt.dt>Cmd_Vel_Timeout_Threshold)&&(Cmd_Vel_Timeout_Threshold!=0)) 
    {
      ComWheelLegged.linear_velocity = 0;
      ComWheelLegged.angle_velocity = 0;
      get_ComWheelLegged.linear_velocity = 0;
      get_ComWheelLegged.angle_velocity = 0;
      //std::cout << " 速度更新超时\n";
    }
  
    
    if((COD_posture_dt.dt>Timeout_threshold)&&(Timeout_threshold!=0)) 
    { 
      ComWheelLegged.leg_length = Default_Height;
      ComWheelLegged.roll_angle = 0;
      ComWheelLegged.pitch_angle = 0;
      ComWheelLegged.set &= static_cast<uint16_t>(~ROS_SET_SLIDE_MASK);
      get_ComWheelLegged.set &= static_cast<uint16_t>(~ROS_SET_SLIDE_MASK);
      //std::cout << " 姿态更新超时\n";
    }
    
    
    if(get_ComWheelLegged.count==255)
      get_ComWheelLegged.count = 0; 




    uint8_t dataArray[dataSize];  // 用于存储转换后的字节数据

    size_t copied = TXD_structToArr(ComWheelLegged, dataArray, dataSize);//将结构体数据复制到字节数组
    if (copied > 0) 
    {
      ComWheelLegged.crc = crc16(dataArray, dataSize-2);
      
      copied = TXD_structToArr(ComWheelLegged, dataArray, dataSize);//将结构体数据复制到字节数组

      if (copied > 0) 
      {
        ssize_t bytesWritten = write(serial_fd_, dataArray, dataSize);

        // 检查写入结果
        if (bytesWritten < 0) {
            // 处理写入错误
            perror("写入串口失败");
        } else if (bytesWritten < dataSize) {
            // 只写入了部分数据，可能需要重试
            printf("警告：只写入了 %zd/%zu 字节\n", bytesWritten, dataSize);
        } else {
            // 成功写入所有数据
            //printf("成功写入 %zd 字节\n", bytesWritten);
/*
            std::cout << std::fixed << std::setprecision(5);
            std::cout << "joint:\n";
            std::cout << "  leg_length: " << ComWheelLegged.leg_length << " m\n";
            std::cout << "  roll_angle: " << ComWheelLegged.roll_angle << " rad\n";
            std::cout << "  pitch_angle: " << ComWheelLegged.pitch_angle << " rad\n";
        
*/

        } 

      }
      else
      {
        perror("结构体转数组失败！");

      }
    } 
    else 
    {
      perror("结构体转数组失败！");

    }

  }

  void TXD_PidTuneCommand()
  {
    PidTuneCommand_t command{};
    {
      std::lock_guard<std::mutex> lock(pid_tune_mutex_);
      if (pid_tune_command_queue_.empty())
        return;
      command = pid_tune_command_queue_.front();
      pid_tune_command_queue_.pop_front();
    }

    command.header1 = 0x7B;
    command.header2 = 0x2F;
    command.length = sizeof(PidTuneCommand_t);
    command.protocol_version = 1;
    command.crc = 0;
    uint8_t data[sizeof(PidTuneCommand_t)];
    std::memcpy(data, &command, sizeof(data));
    command.crc = crc16(data, sizeof(data) - 2);
    std::memcpy(data, &command, sizeof(data));

    const ssize_t bytes_written = write(serial_fd_, data, sizeof(data));
    if (bytes_written != static_cast<ssize_t>(sizeof(data))) {
      RCLCPP_ERROR(this->get_logger(),
        "PID调参命令串口写入不完整: %zd/%zu",
        bytes_written, sizeof(data));
    }
  }

  
  void TXD_JointData(void)//发送移动底盘控制数据
  {
    const size_t dataSize = sizeof(TX_MIT_Data_t); 

    TX_MIT_Data.header1 = 123;             // 帧头1 (123)
    TX_MIT_Data.header2 = 45;              // 帧头2 (45)
    TX_MIT_Data.length = dataSize;         // 结构体长度

    TX_MIT_Data.count++;
    
    if(TX_MIT_Data.count==255)
      TX_MIT_Data.count = 0; 

    uint8_t dataArray[dataSize];  // 用于存储转换后的字节数据

    size_t copied = TXD_joint_structToArr(TX_MIT_Data, dataArray, dataSize);//将结构体数据复制到字节数组
    if (copied > 0) 
    {
      TX_MIT_Data.crc = crc16(dataArray, dataSize-2);
      
      copied = TXD_joint_structToArr(TX_MIT_Data, dataArray, dataSize);//将结构体数据复制到字节数组

      if (copied > 0) 
      {
        ssize_t bytesWritten = write(serial_fd_, dataArray, dataSize);

        // 检查写入结果
        if (bytesWritten < 0) {
            // 处理写入错误
            perror("写入串口失败");
        } else if (bytesWritten < dataSize) {
            // 只写入了部分数据，可能需要重试
            printf("警告：只写入了 %zd/%zu 字节\n", bytesWritten, dataSize);
        } else {
            // 成功写入所有数据
            //printf("成功写入 %zd 字节\n", bytesWritten);
        } 

      }
      else
      {
        perror("结构体转数组失败！");

      }
    } 
    else 
    {
      perror("结构体转数组失败！");

    }
    

  }




  void publish_IMU_Data(void)
  {
    // 创建IMU消息对象
    auto imu_msg = sensor_msgs::msg::Imu();

    //printMilemeterData(ROS_body.milemeter);
    //printImuData(ROS_body.ImuData);

    // ===================== 设置消息头 =====================
    imu_msg.header.stamp = this->now();    // 当前时间戳
    imu_msg.header.frame_id = "imu_link";  // 坐标系名称
    
    // ===================== 方向数据（四元数） =====================
    // 模拟随时间旋转的角度（每帧增加0.05弧度）

    // 设置四元数（绕Z轴旋转）
    // 注意：实际应用中应从真实传感器获取这些数据
    imu_msg.orientation.x = ROS_body.ImuData.q0;
    imu_msg.orientation.y = ROS_body.ImuData.q1;
    imu_msg.orientation.z = ROS_body.ImuData.q2;  // Z分量
    imu_msg.orientation.w = ROS_body.ImuData.q3;  // W分量
    
    // ===================== 角速度数据 =====================
    // 模拟角速度值（单位：弧度/秒）
    imu_msg.angular_velocity.x = ROS_body.ImuData.gyro[0];  // X轴角速度
    imu_msg.angular_velocity.y = ROS_body.ImuData.gyro[1];  // Y轴角速度
    imu_msg.angular_velocity.z = ROS_body.ImuData.gyro[2];              // Z轴角速度（恒定）
    
    
    // ===================== 线加速度数据 =====================
    // 模拟加速度值（单位：米/秒²）
    // 注意：实际IMU测量值包含重力加速度
    imu_msg.linear_acceleration.x = ROS_body.ImuData.accel[0];   // X轴加速度
    imu_msg.linear_acceleration.y = ROS_body.ImuData.accel[1];   // Y轴加速度
    imu_msg.linear_acceleration.z = ROS_body.ImuData.accel[2];  // Z轴加速度（重力）
    
  
    // 发布IMU消息
    imu_publisher_->publish(imu_msg);


  }

  // 定时器回调函数，每次触发时发布IMU数据
  void timer_callback()
  {
    if(Serial_update_flag==1)
    {
      //printMilemeterData(ROS_body.milemeter);
      publish_odom();     // 发布里程计信息
      publish_IMU_Data();
      publish_sbus_data();
      publish_JointState();
      publish_MotorTemperatures();
      Serial_update_flag = 0;
    }
   
  }


  void timer2_callback()
  {
    if(Robot_Mode_==0)
    {
      TXD_MobileChassis();
      //printMITFeedbackData(ROS_body);
    }
    else
    {
      if(cmd_joint_updated_sign_==true)
      {
        TXD_JointData();  
        //printMitTxdData(TX_MIT_Data);
        cmd_joint_updated_sign_ = false;
      }

    }
    TXD_PidTuneCommand();
  }



  // 回调函数处理接收到的关节控制消息（
  void joint_control_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
  {
      // 检查消息数据的一致性（保留原有逻辑）
      if (msg->name.size() != msg->position.size() || 
          msg->name.size() != msg->velocity.size() ||
          msg->name.size() != msg->effort.size())
      {
          RCLCPP_WARN(this->get_logger(), "不匹配的关节状态数据长度");
          return;
      }
      else
      {

        if(cmd_joint_updated_sign_ == false)
        {
          for (size_t i = 0; i < msg->name.size(); ++i)
          {
              const std::string& joint_name = msg->name[i];  // 关节名称
              TX_MIT_Data.MIT_Command_Data[i].position = radians_to_degrees(msg->position[i]);            // 关节位置（度）
              TX_MIT_Data.MIT_Command_Data[i].velocity = radians_to_degrees(msg->velocity[i]);            // 关节速度（度/秒）
              TX_MIT_Data.MIT_Command_Data[i].effort = msg->effort[i];                                    // 关节力/力矩（牛·米）
  
          }

          cmd_joint_updated_sign_ = true;

        }

      }

  }





  
  // ===================== 新增：运动控制回调函数 =====================
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 获取互斥锁保护控制指令数据
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    // 存储接收到的控制指令
    current_cmd_ = *msg;
    COD_vel_dt.last_time = this->now();
    get_ComWheelLegged.linear_velocity = msg->linear.x;
    get_ComWheelLegged.angle_velocity = msg->angular.z;
    cmd_updated_vel_ = true;
/*
    // 打印接收到的控制指令
    RCLCPP_INFO(this->get_logger(), 
                "Received control command: "
                "Linear: x=%.2f, y=%.2f, z=%.2f | "
                "Angular: x=%.2f, y=%.2f, z=%.2f",
                msg->linear.x, msg->linear.y, msg->linear.z,
                msg->angular.x, msg->angular.y, msg->angular.z);
*/
        
        
  }

  void send_stop_command_frames()
  {
    if (serial_fd_ == -1) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      get_ComWheelLegged.linear_velocity = 0.0f;
      get_ComWheelLegged.angle_velocity = 0.0f;
      ComWheelLegged.linear_velocity = 0.0f;
      ComWheelLegged.angle_velocity = 0.0f;
      COD_vel_dt.last_time = this->now();
      cmd_updated_vel_ = true;
    }

    for (int index = 0; index < 5; ++index) {
      TXD_MobileChassis();
      std::this_thread::sleep_for(20ms);
    }
  }

  void cmd_jump_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
      get_ComWheelLegged.set |= ROS_SET_JUMP;
    else
      get_ComWheelLegged.set &= static_cast<uint16_t>(~ROS_SET_JUMP);
  }

  void cmd_jump_profile_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(this->get_logger(),
        "/cmd_jump_profile 需要至少2个数: [jump_min_height, jump_max_height]");
      return;
    }

    if (msg->data[0] <= 0.0f || msg->data[1] <= 0.0f) {
      get_ComWheelLegged.jump_min_height = Default_Jump_Min_Height;
      get_ComWheelLegged.jump_max_height = Default_Jump_Max_Height;
      get_ComWheelLegged.set &= static_cast<uint16_t>(~ROS_SET_JUMP_PROFILE);
      cmd_jump_profile_updated_sign_ = true;
      return;
    }

    const float min_height = std::clamp(
      msg->data[0], ROS_JUMP_MIN_HEIGHT_LOW, ROS_JUMP_MIN_HEIGHT_HIGH);
    const float max_height = std::clamp(
      msg->data[1], ROS_JUMP_MAX_HEIGHT_LOW, ROS_JUMP_MAX_HEIGHT_HIGH);
    if (max_height <= min_height + 0.04f) {
      RCLCPP_WARN(this->get_logger(),
        "拒绝跳跃高度配置: min=%.3f max=%.3f，两者间隔太小",
        min_height, max_height);
      return;
    }

    get_ComWheelLegged.jump_min_height = min_height;
    get_ComWheelLegged.jump_max_height = max_height;
    get_ComWheelLegged.set |= ROS_SET_JUMP_PROFILE;
    jump_profile_last_update_ = this->now();
    cmd_jump_profile_updated_sign_ = true;
  }

  void pid_tune_command_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // data = [action, parameter_id, value, sequence]
    if (msg->data.size() != 4) {
      RCLCPP_WARN(this->get_logger(),
        "PID调参命令长度错误，应为4，实际为%zu", msg->data.size());
      return;
    }

    const int action = static_cast<int>(std::lround(msg->data[0]));
    const int parameter_id = static_cast<int>(std::lround(msg->data[1]));
    const int sequence = static_cast<int>(std::lround(msg->data[3]));
    if (action < PID_TUNE_ACTION_BEGIN || action > PID_TUNE_ACTION_GET ||
        parameter_id < PID_TUNE_PARAM_NONE ||
        parameter_id > PID_TUNE_PARAM_HEADING_HOLD_ENABLE ||
        sequence < 0 || sequence > 65535 ||
        !std::isfinite(msg->data[2])) {
      RCLCPP_WARN(this->get_logger(), "拒绝格式或范围无效的PID调参命令");
      return;
    }

    PidTuneCommand_t command{};
    command.action = static_cast<uint8_t>(action);
    command.parameter_id = static_cast<uint8_t>(parameter_id);
    command.value = msg->data[2];
    command.sequence = static_cast<uint16_t>(sequence);

    std::lock_guard<std::mutex> lock(pid_tune_mutex_);
    constexpr size_t kMaxQueuedCommands = 64;
    if (pid_tune_command_queue_.size() >= kMaxQueuedCommands) {
      RCLCPP_WARN(this->get_logger(), "PID调参命令队列已满，拒绝新命令");
      return;
    }
    pid_tune_command_queue_.push_back(command);
  }
  
  // ===================== 新增：关节状态回调函数 =====================
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 获取互斥锁保护关节状态数据
    std::lock_guard<std::mutex> lock(joint_mutex_);
    
    // 假设消息包含至少3个关节的数据
    if (msg->name.size() >= 3 && msg->position.size() >= 3) 
    {
      // 查找三个关节的索引
      int joint_height_idx = -1, joint_roll_idx = -1, joint_pitching_idx = -1;
      int joint_slide_idx = -1;
      
      for (size_t i = 0; i < msg->name.size(); i++) 
      {
        if (msg->name[i] == "joint_height") joint_height_idx = i;
        if (msg->name[i] == "joint_roll") joint_roll_idx = i;
        if (msg->name[i] == "joint_pitching") joint_pitching_idx = i;
        if (msg->name[i] == "joint_slide") joint_slide_idx = i;
      }
      
      // 更新关节数据
      if (joint_height_idx != -1 && joint_roll_idx != -1 && joint_pitching_idx != -1) 
      {
        joint_positions_[0] = msg->position[joint_height_idx];
        joint_positions_[1] = msg->position[joint_roll_idx];
        joint_positions_[2] = msg->position[joint_pitching_idx];
        COD_posture_dt.last_time = COD_posture_dt.current_time; 
        
        if(cmd_posture_updated_sign_ == false)
        {
          get_ComWheelLegged.leg_length = std::clamp(joint_positions_[0], LEG_MIN_LOW, LEG_MAX_HIGH);
          get_ComWheelLegged.roll_angle = std::clamp(joint_positions_[1], -Roll_limits, Roll_limits);
          get_ComWheelLegged.pitch_angle = std::clamp(joint_positions_[2], -Pitch_limits, Pitch_limits);
          float slide = 0.0f;
          if (joint_slide_idx != -1 &&
              joint_slide_idx < static_cast<int>(msg->position.size()))
          {
            slide = static_cast<float>(msg->position[joint_slide_idx]);
          }
          get_ComWheelLegged.set = encode_slide_into_set(
              get_ComWheelLegged.set, slide);
          cmd_posture_updated_sign_ = true;
        }

        // 如果有速度和力数据，也更新它们
        if (msg->velocity.size() >= 3) 
        {
          joint_velocities_[0] = msg->velocity[joint_height_idx];
          joint_velocities_[1] = msg->velocity[joint_roll_idx];
          joint_velocities_[2] = msg->velocity[joint_pitching_idx];
        }
        
        if (msg->effort.size() >= 3) 
        {
          joint_efforts_[0] = msg->effort[joint_height_idx];
          joint_efforts_[1] = msg->effort[joint_roll_idx];
          joint_efforts_[2] = msg->effort[joint_pitching_idx];
        }
      }
    } else if (msg->position.size() >= 3) {
      // 如果没有关节名称但有至少3个位置数据，假设前三个数据对应三个关节
      joint_positions_[0] = msg->position[0];
      joint_positions_[1] = msg->position[1];
      joint_positions_[2] = msg->position[2];
      
      if (msg->velocity.size() >= 3) {
        joint_velocities_[0] = msg->velocity[0];
        joint_velocities_[1] = msg->velocity[1];
        joint_velocities_[2] = msg->velocity[2];
      }
      
      if (msg->effort.size() >= 3) {
        joint_efforts_[0] = msg->effort[0];
        joint_efforts_[1] = msg->effort[1];
        joint_efforts_[2] = msg->effort[2];
      }
    }
  }
  
  // ===================== 串口任务函数 =====================
  void serial_task()
  {
    // 主通信循环：持续接收并解析串口数据
    while (rclcpp::ok()) 
    {
        // 检查停止标志
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (stop_serial_task_) break;
        }
        
        
        // 读取并解析串口数据
        if (serial_fd_ != -1) 
        {
            try {
                SERIAL_RXT(serial_fd_);  // 读取并解析串口数据
                if(Diagnostic_update_flag == 1)
                {
                  publish_diagnostics();
                  Diagnostic_update_flag = 0;
                }
                if(PidTuneTelemetry_update_flag == 1)
                {
                  publish_pid_tune_telemetry();
                  PidTuneTelemetry_update_flag = 0;
                }
                if(Serial_update_flag == 1)
                {
                  // 计算实际时间间隔
                  rclcpp::Time current_time = this->get_clock()->now();
                  double dt = (current_time - last_odom_time_).seconds();
                  last_odom_time_ = current_time;
                  
                  update_odom(dt);    // 更新里程计数据
                  //printMilemeterData(ROS_body.milemeter);
                  //printImuData(ROS_body.ImuData);

                  if(ROS_body.Status.status.model==0)
                  {
                    Robot_Mode_ = 0;
                  }
                  else 
                  {
                    Robot_Mode_ = 1;
                  }
                  
                }
            } 
            catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Serial communication error: %s", e.what());
                std::this_thread::sleep_for(1s);  // 异常后延时1秒
            }
        } else {
            // 如果没有串口，等待一段时间再重试
            std::this_thread::sleep_for(1s);
        }
        
        // 短暂睡眠以避免CPU占用过高
        std::this_thread::sleep_for(10ms);
    }

  }


  void publish_sbus_data()
  {
    auto msg = std_msgs::msg::Int16MultiArray();
    
    // 确保数组大小为10
    msg.data.resize(10);
    
    // 将ros_body中的SBUS通道数据复制到消息中
    for (int i = 0; i < 10; ++i) {
        msg.data[i] = ROS_body.SBUS_Channels_Data[i];
    }
    
    // 发布消息
    sbus_publisher_->publish(msg);
    
    // 打印调试信息（仅在DEBUG级别可见）
    // 打印全部10通道数据，便于完整调试
    //RCLCPP_INFO(this->get_logger(), 
    //            "Publishing SBUS data: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
    //            msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
    //            msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9]);
  }

  void publish_pid_tune_telemetry()
  {
    const PidTuneTelemetry_t data = PidTuneTelemetry;
    std_msgs::msg::Float32MultiArray message;
    // Keep this order synchronized with wheel_robot/scripts/vofa_pid_tuner.py.
    message.data = {
      static_cast<float>(data.protocol_version),
      static_cast<float>(data.session_active),
      static_cast<float>(data.last_status),
      static_cast<float>(data.last_parameter_id),
      static_cast<float>(data.last_sequence),
      static_cast<float>(data.safety_flags),
      static_cast<float>(data.uptime_ms),
      data.target_speed_mps,
      data.measured_speed_mps,
      data.session_position_m,
      data.target_pitch_deg,
      data.measured_pitch_deg,
      data.pitch_rate_rad_s,
      data.speed_error_mps,
      data.speed_integral,
      data.speed_output,
      data.target_leg_x_m,
      data.angle_error_deg,
      data.angle_output_nm,
      data.target_yaw_rate_rad_s,
      data.measured_yaw_rate_rad_s,
      data.yaw_error_rad_s,
      data.yaw_output_nm,
      data.yaw_hold_error_deg,
      data.left_wheel_torque_nm,
      data.right_wheel_torque_nm,
      data.speed_kp,
      data.speed_ki,
      data.speed_kd,
      data.applied_speed_kp,
      data.applied_speed_ki,
      data.yaw_kp,
      data.yaw_ki,
      data.yaw_kd,
      data.applied_yaw_ki,
      data.heading_hold_kp,
      data.configured_angle_kp,
      data.applied_angle_kp,
      data.angle_kd,
    };
    pid_tune_telemetry_publisher_->publish(message);
  }

  static std::string boot_stage_text(uint8_t stage)
  {
    switch (stage) {
      case 0: return "ESP32启动";
      case 1: return "电机/CAN初始化";
      case 2: return "IMU初始化";
      case 3: return "遥控器初始化";
      case 4: return "等待遥控器复位";
      case 5: return "初始化完成";
      case 100: return "CAN初始化失败";
      case 101: return "电机停止失败";
      case 102: return "电机零点设置失败";
      case 103: return "电机方向设置失败";
      case 104: return "电机模式设置失败";
      case 105: return "电机使能失败";
      case 106: return "IMU初始化失败";
      default: return "未知启动阶段";
    }
  }

  static diagnostic_msgs::msg::KeyValue key_value(
    const std::string &key, const std::string &value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
  }

  void publish_diagnostics()
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const RobotDiagnostic_t data = RobotDiagnostic;
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = this->now();

    DiagnosticStatus overall;
    overall.name = "wheel_robot/self_test";
    overall.hardware_id = "esp32s3_wheel_leg_controller";
    overall.level = data.overall_state == 2 ? DiagnosticStatus::OK :
                    data.overall_state == 3 ? DiagnosticStatus::ERROR : DiagnosticStatus::WARN;
    overall.message = boot_stage_text(data.boot_stage);
    overall.values.push_back(key_value("protocol_version", std::to_string(data.protocol_version)));
    overall.values.push_back(key_value("boot_stage", std::to_string(data.boot_stage)));
    overall.values.push_back(key_value("fault_bits", std::to_string(data.fault_bits)));
    overall.values.push_back(key_value("uptime_ms", std::to_string(data.uptime_ms)));
    overall.values.push_back(key_value("robot_mode", std::to_string(data.robot_mode)));
    array.status.push_back(overall);

    DiagnosticStatus remote;
    remote.name = "wheel_robot/remote";
    remote.hardware_id = overall.hardware_id;
    if (data.sbus_frame_age_ms >= 200 || data.sbus_failsafe != 0) {
      remote.level = DiagnosticStatus::ERROR;
      remote.message = "遥控器未连接或信号丢失";
    } else if (data.boot_stage == 4 && data.remote_reset_mask != 0x3F) {
      remote.level = DiagnosticStatus::WARN;
      remote.message = "遥控器已连接，但通道未复位";
    } else {
      remote.level = DiagnosticStatus::OK;
      remote.message = "遥控器信号正常";
    }
    remote.values.push_back(key_value("frame_period_ms", std::to_string(data.sbus_frame_period_ms)));
    remote.values.push_back(key_value("frame_age_ms", std::to_string(data.sbus_frame_age_ms)));
    remote.values.push_back(key_value("failsafe", std::to_string(data.sbus_failsafe)));
    remote.values.push_back(key_value("reset_mask", std::to_string(data.remote_reset_mask)));
    for (int i = 0; i < 10; ++i)
      remote.values.push_back(key_value("channel_" + std::to_string(i + 1),
                                        std::to_string(data.sbus_channels[i])));
    array.status.push_back(remote);

    DiagnosticStatus battery;
    battery.name = "wheel_robot/battery";
    battery.hardware_id = overall.hardware_id;
    battery.level = (data.fault_bits & DIAG_FAULT_BATTERY_LOW) ?
                    DiagnosticStatus::ERROR : DiagnosticStatus::OK;
    battery.message = battery.level == DiagnosticStatus::OK ? "电池电压正常" : "电池电压过低";
    battery.values.push_back(key_value("voltage_v",
      std::to_string(static_cast<double>(data.battery_mv) / 1000.0)));
    array.status.push_back(battery);

    DiagnosticStatus can_imu;
    can_imu.name = "wheel_robot/can_imu";
    can_imu.hardware_id = overall.hardware_id;
    const bool can_fault = data.fault_bits & DIAG_FAULT_CAN_INIT;
    const bool imu_fault = data.fault_bits & DIAG_FAULT_IMU;
    can_imu.level = (can_fault || imu_fault) ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
    can_imu.message = can_fault ? "CAN初始化失败" :
                      imu_fault ? "IMU初始化失败" : "CAN与IMU无初始化故障";
    can_imu.values.push_back(key_value(
      "yaw_hold_active", data.yaw_hold_active ? "1" : "0"));
    can_imu.values.push_back(key_value(
      "yaw_bias_rad_s",
      std::to_string(static_cast<double>(data.yaw_bias_millirad_s) / 1000.0)));
    can_imu.values.push_back(key_value(
      "corrected_yaw_rate_rad_s",
      std::to_string(static_cast<double>(data.corrected_yaw_rate_millirad_s) / 1000.0)));
    can_imu.values.push_back(key_value(
      "yaw_hold_error_deg",
      std::to_string(static_cast<double>(data.yaw_hold_error_centideg) / 100.0)));
    array.status.push_back(can_imu);

    for (int i = 0; i < 6; ++i) {
      const uint8_t bit = static_cast<uint8_t>(1U << i);
      DiagnosticStatus motor;
      motor.name = "wheel_robot/motor_" + std::to_string(i + 1);
      motor.hardware_id = "can_motor_" + std::to_string(i + 1);
      const bool seen = data.motor_seen_mask & bit;
      const bool fresh = data.motor_fresh_mask & bit;
      const bool enabled = data.motor_enabled_mask & bit;
      if (data.motor_errors[i] != 0 || (data.boot_stage == 5 && !fresh)) {
        motor.level = DiagnosticStatus::ERROR;
        motor.message = data.motor_errors[i] != 0 ? "电机报错" : "电机CAN回复超时";
      } else if (!seen) {
        motor.level = DiagnosticStatus::WARN;
        motor.message = "尚未收到电机回复";
      } else {
        motor.level = DiagnosticStatus::OK;
        motor.message = "电机通信正常";
      }
      motor.values.push_back(key_value("seen", seen ? "1" : "0"));
      motor.values.push_back(key_value("fresh", fresh ? "1" : "0"));
      motor.values.push_back(key_value("enabled", enabled ? "1" : "0"));
      motor.values.push_back(key_value("error_code", std::to_string(data.motor_errors[i])));
      array.status.push_back(motor);
    }

    diagnostics_publisher_->publish(array);
  }




  // ===================== 发布里程计数据函数 =====================
  void publish_odom()
  {
    // 检查里程计数据是否有显著变化
    static double last_x = 0, last_y = 0, last_angle = 0;
    const double pos_threshold = 0.001;  // 1mm
    const double angle_threshold = 0.001; // 约0.057度
    
    double dx = std::abs(odom.x - last_x);
    double dy = std::abs(odom.y - last_y);
    double dangle = std::abs(odom.angle - last_angle);
    
    // 如果变化太小，跳过发布
    //if (dx < pos_threshold && dy < pos_threshold && dangle < angle_threshold) {
    //  return;
    //}
    
    // 更新上次值
    last_x = odom.x;
    last_y = odom.y;
    last_angle = odom.angle;
    
    // 创建里程计消息
    auto odom_msg = nav_msgs::msg::Odometry();
    
    // 设置消息头
    odom_msg.header.stamp = this->now(); // 或者使用硬件时间戳
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // 设置位置
    odom_msg.pose.pose.position.x = odom.x;
    odom_msg.pose.pose.position.y = odom.y;
    odom_msg.pose.pose.position.z = 0.0;
    
    // 将欧拉角转换为四元数
    tf2::Quaternion quat;
    quat.setRPY(0, 0, odom.angle);
    odom_msg.pose.pose.orientation.x = quat.x();
    odom_msg.pose.pose.orientation.y = quat.y();
    odom_msg.pose.pose.orientation.z = quat.z();
    odom_msg.pose.pose.orientation.w = quat.w();
    
    // 设置速度
    odom_msg.twist.twist.linear.x = (double)odom.linear_speed;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.z = (double)odom.angle_speed;
    
    // 优化后的协方差设置
    odom_msg.pose.covariance[0] = 0.05;   // x 方差
    odom_msg.pose.covariance[7] = 0.05;   // y 方差
    odom_msg.pose.covariance[14] = 0.1;   // z 方差
    odom_msg.pose.covariance[21] = 0.1;   // roll 方差
    odom_msg.pose.covariance[28] = 0.1;   // pitch 方差
    odom_msg.pose.covariance[35] = 0.03;  // yaw 方差

    // 速度协方差
    odom_msg.twist.covariance[0] = 0.1;   // vx 方差
    odom_msg.twist.covariance[7] = 0.1;   // vy 方差
    odom_msg.twist.covariance[14] = 0.1;  // vz 方差
    odom_msg.twist.covariance[21] = 0.1;  // vroll 方差
    odom_msg.twist.covariance[28] = 0.1;  // vpitch 方差
    odom_msg.twist.covariance[35] = 0.05; // vyaw 方差
    
    // 发布里程计消息
    odom_publisher_->publish(odom_msg);

    //RCLCPP_INFO(this->get_logger(), "odom_msg sx: %.2f, px: %.2f, px1: %.2f, angle: %.2f", 
    //            odom_msg.twist.twist.linear.x, odom_msg.pose.pose.position.x,  odom.x, odom.angle);

    

    // 发布TF变换
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom_msg.header.stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    // 位置信息（平移）
    transform.transform.translation.x = odom.x;
    transform.transform.translation.y = odom.y;
    transform.transform.translation.z = 0.0;
    
    // 姿态信息（旋转）   
    transform.transform.rotation = odom_msg.pose.pose.orientation;
    
    // 发送变换
    tf_broadcaster_->sendTransform(transform);
  }

  // Publish joint status
  void publish_JointState()
  {
      auto msg = sensor_msgs::msg::JointState();      
      // 设置消息头
      msg.header.stamp = this->now(); // 或者使用硬件时间戳
      msg.header.frame_id = "base_link";

      // 关节名称
      msg.name = g_joint_names;

      // 位置数据（弧度）
      msg.position.resize(MotorID::TOTAL_COUNT);
      for (int i = 0; i < MotorID::TOTAL_COUNT; ++i)
      {
        msg.position[i] = degrees_to_radians(ROS_body.MIT_Feedback_Data[i].position);
      }
      // 速度数据（弧度/秒）
      msg.velocity.resize(MotorID::TOTAL_COUNT);
      for (int i = 0; i < MotorID::TOTAL_COUNT; ++i)
      {
        msg.velocity[i] = degrees_to_radians(ROS_body.MIT_Feedback_Data[i].velocity);
      }
      // 扭矩数据（N·m）
      msg.effort.resize(MotorID::TOTAL_COUNT);
      for (int i = 0; i < MotorID::TOTAL_COUNT; ++i)
      {
        msg.effort[i] = ROS_body.MIT_Feedback_Data[i].torque;
      }
      // 发布消息
      g_publisher->publish(msg);
  }

  void publish_MotorTemperatures()
  {
      auto msg = motor_temperature_pkg::msg::MotorTemperatures();
      msg.header.stamp = this->now();
      msg.header.frame_id = "base_link";
      msg.name = g_joint_names;
      msg.temperature_c.resize(MotorID::TOTAL_COUNT);
      msg.warning_threshold_c =
        static_cast<float>(motor_temperature_warning_threshold_);
      msg.overheated = false;

      for (int i = 0; i < MotorID::TOTAL_COUNT; ++i)
      {
        const float temperature = ROS_body.MIT_Feedback_Data[i].Temp;
        msg.temperature_c[i] = temperature;
        if (temperature > msg.warning_threshold_c)
        {
          msg.overheated = true;
          msg.overheated_index.push_back(static_cast<uint8_t>(i));
        }
      }

      motor_temperature_publisher_->publish(msg);
  }

  


    
    // 修改update_odom函数，使用double类型的时间间隔
  void update_odom(double dt)
  {
    odom.angle_speed = ROS_body.milemeter.AngularVelocity;
    odom.linear_speed = ROS_body.milemeter.Speed;

    // 角速度积分
    odom.angle += odom.angle_speed * dt;  // 
    TransAngleInPI(odom.angle, odom.angle);

    // 小车前进的距离
    double dx = odom.linear_speed * dt;

    odom.x += dx * cos(odom.angle);
    odom.y += dx * sin(odom.angle);
    
    /*
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "里程计数据:\n";
    std::cout << "  左轮速度: " << ROS_body.milemeter.LeftWheelSpeed << " m/s\n";
    std::cout << "  右轮速度: " << ROS_body.milemeter.RightWheelSpeed << " m/s\n";
    std::cout << "  前进速度: " << odom.linear_speed << " m/s\n";
    std::cout << "  角速度: " << odom.angle_speed << " rad/s\n";
    std::cout << "  angle: " << odom.angle << " rad\n";
    std::cout << "  odom.x: " << odom.x << " m\n";
    std::cout << "  odom.y: " << odom.y << " m\n";
    std::cout << "  dt: " << dt << " s";  
    */
  }


  void TransAngleInPI(float angle,float& out_angle)
  {
    if(angle > M_PI)
    {
      out_angle -= 2*M_PI;
    }
    else if(angle < -M_PI)
    {
      out_angle += 2*M_PI;
    }
      
  }

  
  // ===================== 成员变量 =====================
  rclcpp::TimerBase::SharedPtr timer_;        // 定时器
  rclcpp::TimerBase::SharedPtr timer2_;        // 定时器
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr sbus_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    pid_tune_telemetry_publisher_;
  rclcpp::Publisher<motor_temperature_pkg::msg::MotorTemperatures>::SharedPtr
    motor_temperature_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;  // IMU发布者
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;  // 里程计发布者


  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr g_publisher;
  std::vector<std::string> g_joint_names;


  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // TF变换广播器
  
 
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_joint_subscriber_;
  bool cmd_joint_updated_sign_;                       // 指令更新标志（先声明）

  // ===== 新增：运动控制相关成员 =====
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; // 运动控制订阅
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_jump_sub_; // 一次性跳跃请求
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
    cmd_jump_profile_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
    pid_tune_command_sub_;
  geometry_msgs::msg::Twist current_cmd_;      // 当前控制指令
  std::mutex cmd_mutex_;                       // 控制指令互斥锁
  bool cmd_updated_vel_;                       // 指令更新标志（先声明）
  bool cmd_jump_profile_updated_sign_{false};
  rclcpp::Time jump_profile_last_update_{0, 0, RCL_ROS_TIME};
  
  // ===== 新增：关节状态相关成员 =====
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr body_posture_sub_; //身体身体姿式订阅
  std::vector<double> joint_positions_;        // 关节位置
  std::vector<double> joint_velocities_;       // 关节速度
  std::vector<double> joint_efforts_;          // 关节力/力矩
  std::mutex joint_mutex_;                     // 关节数据互斥锁
  bool cmd_posture_updated_sign_;                       // 指令更新标志（先声明）
  
  // ===== 串口任务相关成员 =====
  std::thread serial_thread_;          // 串口任务线程
  std::mutex serial_mutex_;            // 保护串口资源的互斥锁
  bool stop_serial_task_;              // 线程停止标志（后声明）
  int serial_fd_;                      // 串口文件描述符
  bool serial_selected_;               // 串口选择成功标志
  std::mutex pid_tune_mutex_;
  std::deque<PidTuneCommand_t> pid_tune_command_queue_;
  double motor_temperature_warning_threshold_;
  
  odom_t odom;    //里程计信息
  rclcpp::Time last_odom_time_;  // 存储上一次里程计更新时间

  Time_dt_t COD_vel_dt;
  Time_dt_t COD_posture_dt;

  ComWheelLegged_t ComWheelLegged;
  ComWheelLegged_t get_ComWheelLegged;

  int Robot_Mode_;//0:控制移动底盘  1：控制电机

};

// 主函数
int main(int argc, char * argv[])
{

  // 初始化ROS2
  rclcpp::init(argc, argv);
  
  // 创建节点并开始循环
  auto node = std::make_shared<Legwheel>();
  
  // 只有当串口选择成功后才开始spin
  rclcpp::spin(node);
  
  // 关闭ROS2
  rclcpp::shutdown();
  return 0;
}
