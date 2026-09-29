#ifndef ROBOT_STRUCTS_H
#define ROBOT_STRUCTS_H

#include <cstdint> // 用于标准整数类型

/**
 * @brief 轮式机器人电机ID枚举
 * 包含左前、左后、右前、右后电机及左轮、右轮电机（适配不同底盘结构）
 */
enum MotorID : uint8_t  
{
    LEFT_FRONT = 0,    // 身体左前电机
    LEFT_REAR = 1,     // 身体左后电机
    RIGHT_FRONT = 2,   // 身体右前电机
    RIGHT_REAR = 3,    // 身体右后电机
    LEFT_WHEEL = 4,    // 左轮电机
    RIGHT_WHEEL = 5,   // 右轮电机

    TOTAL_COUNT        // 电机总数（用于数组大小定义、遍历等）
};



// 里程计数据结构
typedef struct {
    double  x;     
    double  y;    
    float angle;             
    float linear_speed;  // 前进速度 (m/s) 
    float angle_speed;   // 角速度 (rad/s) 
} odom_t;



struct RobotStatus {
    // 状态码联合体（你的定义）
    union {
        uint16_t status_code;  // 32位状态码整体
        struct {
            uint16_t switched_on : 1;           // 机器开启状态(1=开启，0=关闭)
            uint16_t model : 1;                 // 机器工作模式(1=控制电机模式，0=控制移动底盘模式)
            uint16_t PADDING : 14;              // 保留位（必须为0）
        };
    } status;
};




// ==================== 新增结构体定义 ====================
#pragma pack(push, 1)  // 确保1字节对齐，避免填充字节


// ROS通信数据结构体
typedef struct {
    uint8_t header1;          // 帧头1 (123)
    uint8_t header2;          // 帧头2 (45)
    uint16_t length;          // 结构体长度
    float linear_velocity;
    float angle_velocity;
    float leg_length;
    float pitch_angle;
    float roll_angle;
    float jump_min_height;
    float jump_max_height;
    uint16_t set;             // 设置
    uint8_t count;
    uint16_t crc;             // CRC16校验
    
} ComWheelLegged_t;


// 里程计数据结构
typedef struct {
    float LeftWheelSpeed;     // 左轮速度 (m/s)
    float RightWheelSpeed;    // 右轮速度 (m/s)
    float Speed;              // 前进速度 (m/s)
    float AngularVelocity;    // 角速度 (rad/s)
} milemeter_t;

// IMU传感器数据结构
typedef struct {
    float accel[3];           // 加速度计数据 [x, y, z] (m/s²)
    float gyro[3];            // 陀螺仪数据 [x, y, z] (rad/s)
    float q0 = 1.0f;          // 四元数分量
    float q1 = 0.0f;          // 四元数分量
    float q2 = 0.0f;          // 四元数分量
    float q3 = 0.0f;          // 四元数分量
} ImuData_t;

// 电机控制参数
typedef struct {
    float position; // 位置指令
    float velocity; // 速度指令
    float kp;           // 比例增益
    float kd;           // 微分增益
    float effort;    // 前馈转矩
} MIT_Command_Data_t;


typedef struct {
    uint8_t header1;          // 帧头1 (123)
    uint8_t header2;          // 帧头2 (45)
    uint16_t length;          // 数据包长度
    MIT_Command_Data_t MIT_Command_Data[6];
    uint16_t count;            // 计数
    uint16_t crc;             // CRC16校验    
} TX_MIT_Data_t;


typedef struct {
    float position; // 电机位置信息
    float velocity; // 电机速度信息
    float torque;   // 力矩
    float Temp;           //回传温度
} MIT_Feedback_Data_t;


// ROS通信数据结构体
typedef struct {
    uint8_t header1;          // 帧头1 (123)
    uint8_t header2;          // 帧头2 (45)
    uint16_t length;          // 数据包长度
    milemeter_t milemeter;    // 里程计数据
    ImuData_t ImuData;        // IMU数据
    MIT_Feedback_Data_t MIT_Feedback_Data[6];//电机反馈
    int16_t SBUS_Channels_Data[10];
    RobotStatus Status;          // 状态标志
    uint16_t count;            // 计数
    uint16_t crc;             // CRC16校验
} ROS_body_t;


// ROS通信数据结构体
typedef struct {
    uint8_t header1;          // 帧头1 (123)
    uint8_t header2;          // 帧头2 (45)
    uint16_t length;          // 数据包长度
    MIT_Feedback_Data_t MIT_Feedback_Data[6];//电机反馈
    int16_t SBUS_Channels_Data[10];
    uint16_t count;            // 计数
    uint16_t crc;             // CRC16校验
} ROS_body_t1;

// Structured diagnostics sent by ESP32 over the same Type-C serial link.
typedef struct {
    uint8_t header1;              // 0x7B
    uint8_t header2;              // 0x2E
    uint16_t length;
    uint8_t protocol_version;
    uint8_t boot_stage;
    uint8_t overall_state;        // 0=starting, 1=waiting, 2=ready, 3=fault
    uint8_t robot_mode;
    uint32_t fault_bits;
    uint32_t uptime_ms;
    uint16_t battery_mv;
    uint16_t sbus_frame_period_ms;
    uint16_t sbus_frame_age_ms;
    int16_t sbus_channels[10];
    uint8_t sbus_failsafe;
    uint8_t remote_reset_mask;
    uint8_t motor_seen_mask;
    uint8_t motor_fresh_mask;
    uint8_t motor_enabled_mask;
    uint8_t yaw_hold_active;
    int16_t yaw_bias_millirad_s;
    int16_t corrected_yaw_rate_millirad_s;
    int16_t yaw_hold_error_centideg;
    uint32_t motor_errors[6];
    uint16_t count;
    uint16_t crc;
} RobotDiagnostic_t;

static_assert(sizeof(RobotDiagnostic_t) == 82,
              "RobotDiagnostic_t protocol size must be 82 bytes");

typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint16_t length;
    uint8_t protocol_version;
    uint8_t action;
    uint8_t parameter_id;
    uint8_t reserved;
    float value;
    uint16_t sequence;
    uint16_t crc;
} PidTuneCommand_t;

static_assert(sizeof(PidTuneCommand_t) == 16,
              "PidTuneCommand_t protocol size must be 16 bytes");

typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint16_t length;
    uint8_t protocol_version;
    uint8_t session_active;
    uint8_t last_status;
    uint8_t last_parameter_id;
    uint16_t last_sequence;
    uint16_t safety_flags;
    uint32_t uptime_ms;
    float target_speed_mps;
    float measured_speed_mps;
    float session_position_m;
    float target_pitch_deg;
    float measured_pitch_deg;
    float pitch_rate_rad_s;
    float speed_error_mps;
    float speed_integral;
    float speed_output;
    float target_leg_x_m;
    float angle_error_deg;
    float angle_output_nm;
    float target_yaw_rate_rad_s;
    float measured_yaw_rate_rad_s;
    float yaw_error_rad_s;
    float yaw_output_nm;
    float yaw_hold_error_deg;
    float left_wheel_torque_nm;
    float right_wheel_torque_nm;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float applied_speed_kp;
    float applied_speed_ki;
    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float applied_yaw_ki;
    float heading_hold_kp;
    float configured_angle_kp;
    float applied_angle_kp;
    float angle_kd;
    uint16_t count;
    uint16_t crc;
} PidTuneTelemetry_t;

static_assert(sizeof(PidTuneTelemetry_t) == 148,
              "PidTuneTelemetry_t protocol size must be 148 bytes");


#pragma pack(pop)  // 恢复默认对齐方式

enum PidTuneAction : uint8_t {
    PID_TUNE_ACTION_BEGIN = 1,
    PID_TUNE_ACTION_SET = 2,
    PID_TUNE_ACTION_REVERT = 3,
    PID_TUNE_ACTION_RESTORE_BOOT = 4,
    PID_TUNE_ACTION_ACCEPT_RAM = 5,
    PID_TUNE_ACTION_GET = 6,
};

enum PidTuneParameter : uint8_t {
    PID_TUNE_PARAM_NONE = 0,
    PID_TUNE_PARAM_SPEED_KP = 1,
    PID_TUNE_PARAM_SPEED_KI = 2,
    PID_TUNE_PARAM_SPEED_KD = 3,
    PID_TUNE_PARAM_YAW_KP = 4,
    PID_TUNE_PARAM_YAW_KI = 5,
    PID_TUNE_PARAM_YAW_KD = 6,
    PID_TUNE_PARAM_HEADING_HOLD_KP = 7,
    PID_TUNE_PARAM_HEADING_HOLD_ENABLE = 8,
};

enum PidTuneStatus : uint8_t {
    PID_TUNE_STATUS_OK = 0,
    PID_TUNE_STATUS_NO_SESSION = 1,
    PID_TUNE_STATUS_UNSAFE_STATE = 2,
    PID_TUNE_STATUS_BAD_PARAMETER = 3,
    PID_TUNE_STATUS_OUT_OF_RANGE = 4,
    PID_TUNE_STATUS_BAD_ACTION = 5,
    PID_TUNE_STATUS_BAD_VERSION = 6,
    PID_TUNE_STATUS_AUTO_REVERTED = 7,
    PID_TUNE_STATUS_TARGET_PENDING = 8,
};

enum PidTuneSafetyFlags : uint16_t {
    PID_TUNE_SAFE_BOOT_READY = 1U << 0,
    PID_TUNE_SAFE_CHASSIS_MODE = 1U << 1,
    PID_TUNE_SAFE_ZERO_COMMAND = 1U << 2,
    PID_TUNE_SAFE_NOT_JUMPING = 1U << 3,
    PID_TUNE_SAFE_PITCH = 1U << 4,
    PID_TUNE_SAFE_REMOTE = 1U << 5,
};

enum DiagnosticFaultBits : uint32_t {
    DIAG_FAULT_CAN_INIT = 1UL << 0,
    DIAG_FAULT_MOTOR_STOP = 1UL << 1,
    DIAG_FAULT_MOTOR_HOME = 1UL << 2,
    DIAG_FAULT_MOTOR_DIRECTION = 1UL << 3,
    DIAG_FAULT_MOTOR_MODE = 1UL << 4,
    DIAG_FAULT_MOTOR_ENABLE = 1UL << 5,
    DIAG_FAULT_IMU = 1UL << 6,
    DIAG_FAULT_SBUS = 1UL << 7,
    DIAG_FAULT_REMOTE_NOT_RESET = 1UL << 8,
    DIAG_FAULT_BATTERY_LOW = 1UL << 9,
    DIAG_FAULT_MOTOR_OFFLINE = 1UL << 10,
    DIAG_FAULT_MOTOR_ERROR = 1UL << 11,
};



#endif // ROBOT_STRUCTS_H
