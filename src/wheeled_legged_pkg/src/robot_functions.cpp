#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>          // 确保包含此头文件用于close()
#include <fcntl.h>
#include <termios.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <chrono>            // 确保包含此头文件
#include <thread>            // 确保包含此头文件
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include "robot_functions.h"




// 全局变量定义
ROS_body_t ROS_body;
ROS_body_t1 ROS_body1;
RobotDiagnostic_t RobotDiagnostic;
PidTuneTelemetry_t PidTuneTelemetry;

TX_MIT_Data_t TX_MIT_Data;//

int Serial_update_flag = 0;
int Serial_update_flag1 = 0;
int Diagnostic_update_flag = 0;
int PidTuneTelemetry_update_flag = 0;

unsigned long RX_error = 0;
unsigned int RX_error_hz = 0;


void TxdMitData_init(void)
{
    const size_t dataSize = sizeof(TX_MIT_Data_t);
    TX_MIT_Data.header1 = 123;
    TX_MIT_Data.header1 = 45;
    TX_MIT_Data.length = dataSize;

    for (int i = 0; i < 4; i++)
    {
        TX_MIT_Data.MIT_Command_Data[i].kp = Motor_KP;
        TX_MIT_Data.MIT_Command_Data[i].kd = Motor_KD;
        TX_MIT_Data.MIT_Command_Data[i].position = Joint_Start_Angle;
        TX_MIT_Data.MIT_Command_Data[i].velocity = 0;
        TX_MIT_Data.MIT_Command_Data[i].effort = 0;
    }
    TX_MIT_Data.MIT_Command_Data[MotorID::LEFT_WHEEL].kp = 0;
    TX_MIT_Data.MIT_Command_Data[MotorID::LEFT_WHEEL].kd = 0;
    TX_MIT_Data.MIT_Command_Data[MotorID::LEFT_WHEEL].effort = 0;

    TX_MIT_Data.MIT_Command_Data[MotorID::RIGHT_WHEEL].kp = 0;
    TX_MIT_Data.MIT_Command_Data[MotorID::RIGHT_WHEEL].kd = 0;
    TX_MIT_Data.MIT_Command_Data[MotorID::RIGHT_WHEEL].effort = 0;

}


/**
 * @brief 将角度转换为弧度
 * @param degrees 角度值（单位：度）
 * @return 对应的弧度值（单位：弧度）
 * @note 转换公式：弧度 = 角度 × π / 180
 */float degrees_to_radians(float degrees) 

{
    return degrees * M_PI / 180.0;
}


/**
 * @brief 将弧度转换为角度
 * @param radians 弧度值（单位：弧度）
 * @return 对应的角度值（单位：度）
 * @note 转换公式：角度 = 弧度 × 180 / π
 */
float radians_to_degrees(float radians) 
{
    return radians * 180.0 / M_PI;
}




/**
 * @brief CRC16校验函数 (Modbus协议常用)
 */
uint16_t crc16(const uint8_t* data, uint32_t length) 
{
    uint16_t crc = 0xFFFF;  // 初始值
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];       // 与当前字节异或
        
        // 按位处理
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) { // 最低位为1
                crc >>= 1;
                crc ^= 0xA001;  // 多项式反转值 (0x8005)
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}




/**
 * @brief 将结构体数据复制到字节数组
 */
size_t structToArr(const ROS_body_t& structData, uint8_t* array, size_t arraySize) {
    const size_t structSize = sizeof(ROS_body_t);
    
    // 安全检查：数组不为空且大小足够
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 内存拷贝（替代循环，效率更高）
    memcpy(array, &structData, structSize);
    return structSize;
}

/**
 * @brief 将结构体数据复制到字节数组
 */
size_t TXD_structToArr(const ComWheelLegged_t& structData, uint8_t* array, size_t arraySize) {
   
    const size_t structSize = sizeof(ComWheelLegged_t);
    
    // 安全检查：数组不为空且大小足够
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 内存拷贝（替代循环，效率更高）
    memcpy(array, &structData, structSize);
    return structSize;
}


size_t TXD_joint_structToArr(const TX_MIT_Data_t& structData, uint8_t* array, size_t arraySize) {
   
    const size_t structSize = sizeof(TX_MIT_Data_t);
    
    // 安全检查：数组不为空且大小足够
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 内存拷贝（替代循环，效率更高）
    memcpy(array, &structData, structSize);
    return structSize;
}



/**
 * @brief 将字节数组数据复制到结构体
 */
size_t arrToStruct(const uint8_t* array, size_t arraySize, ROS_body_t& structData) {
    const size_t structSize = sizeof(ROS_body_t);
    
    // 安全检查：数组不为空且大小足够
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 内存拷贝
    memcpy(&structData, array, structSize);
    return structSize;
}

size_t arrToStruct1(const uint8_t* array, size_t arraySize, ROS_body_t1& structData) {
    const size_t structSize = sizeof(ROS_body_t1);
    
    // 安全检查：数组不为空且大小足够
    if (array == nullptr || arraySize < structSize) {
        return 0;
    }
    
    // 内存拷贝
    memcpy(&structData, array, structSize);
    return structSize;
}

void printRobotStatus(const ROS_body_t& ros_body)
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "机器人状态:\n";

    std::cout << "    开关机: " << ros_body.Status.status.switched_on << "\n";
    std::cout << "    机器工作模式: " << ros_body.Status.status.model << "\n";


}


void printMITFeedbackData(const ROS_body_t& ros_body)
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "电机反馈数据:\n";
    
    for (int i = 0; i < 6; ++i)
    {
        std::cout << "  电机 " << i+1 << ":\n";
        std::cout << "    位置: " << ros_body.MIT_Feedback_Data[i].position << "\n";
        std::cout << "    速度: " << ros_body.MIT_Feedback_Data[i].velocity << "\n";
        std::cout << "    力矩: " << ros_body.MIT_Feedback_Data[i].torque << "\n";
        std::cout << "    温度: " << ros_body.MIT_Feedback_Data[i].Temp << "\n";
    }
}


void printMitTxdData(const TX_MIT_Data_t& TX_Data)
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "电机电机反馈数据:\n";
    
    for (int i = 0; i < 6; ++i)
    {
        std::cout << "  电机 " << i+1 << ":\n";
        std::cout << "    位置: " << TX_Data.MIT_Command_Data[i].position << "\n";
        std::cout << "    速度: " << TX_Data.MIT_Command_Data[i].velocity << "\n";
        std::cout << "    力矩: " << TX_Data.MIT_Command_Data[i].effort << "\n";
        std::cout << "    kp: " << TX_Data.MIT_Command_Data[i].kp << "\n";
        std::cout << "    kd: " << TX_Data.MIT_Command_Data[i].kd << "\n";
    }
}



void printSbusData(const ROS_body_t& ros_body)
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "遥控器通道数据:\n";
    
    for (int i = 0; i < 10; ++i)
    {
        std::cout << "  通道 " << i+1 << ":";
        std::cout << ros_body.SBUS_Channels_Data[i] << "\n";
    }
}



/**
 * @brief 打印里程计数据
 */
void printMilemeterData(const milemeter_t& milemeter) 
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "里程计数据:\n";
    std::cout << "  左轮速度: " << milemeter.LeftWheelSpeed << " m/s\n";
    std::cout << "  右轮速度: " << milemeter.RightWheelSpeed << " m/s\n";
    std::cout << "  前进速度: " << milemeter.Speed << " m/s\n";
    std::cout << "  角速度: " << milemeter.AngularVelocity << " rad/s\n";
}

/**
 * @brief 打印IMU传感器数据
 */
void printImuData(const ImuData_t& imu) 
{
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "IMU数据:\n";
    std::cout << "  加速度: [" 
              << imu.accel[0] << ", " 
              << imu.accel[1] << ", " 
              << imu.accel[2] << "] m/s²\n";
    std::cout << "  陀螺仪: [" 
              << imu.gyro[0] << ", " 
              << imu.gyro[1] << ", " 
              << imu.gyro[2] << "] rad/s\n";
    std::cout << "  四元数分量: [" 
              << imu.q0 << ", " 
              << imu.q1 << ", " 
              << imu.q2 << ", " 
              << imu.q3 << "] \n";   
    
    // 计算欧拉角（从四元数转换）
    float roll = atan2(2*(imu.q0*imu.q1 + imu.q2*imu.q3), 1 - 2*(imu.q1*imu.q1 + imu.q2*imu.q2));
    float pitch = asin(2*(imu.q0*imu.q2 - imu.q3*imu.q1));
    float yaw = atan2(2*(imu.q0*imu.q3 + imu.q1*imu.q2), 1 - 2*(imu.q2*imu.q2 + imu.q3*imu.q3));
    
    std::cout << "  姿态 (欧拉角):\n";
    std::cout << "  横滚(Roll): " << roll * 180/M_PI << "°\n";
    std::cout << "  俯仰(Pitch): " << pitch * 180/M_PI << "°\n";
    std::cout << "  偏航(Yaw): " << yaw * 180/M_PI << "°\n";
}

/**
 * @brief 枚举系统中可用的串口设备
 */
std::vector<std::string> listSerialPorts() {
    std::vector<std::string> ports;
    
    DIR* dir = opendir("/dev");
    if (!dir) return ports;

    dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        
        // 识别常见的串口设备名格式
        if (name.substr(0, 6) == "ttyUSB" || 
            name.substr(0, 6) == "ttyACM" ||
            name.substr(0, 4) == "ttyS"  ||
            name.substr(0, 6) == "ttyAMA") {
            ports.push_back("/dev/" + name);
        }
    }
    closedir(dir);
    return ports;
}

/**
 * @brief 打开并配置串口
 */
int openSerialPort(const std::string &port, speed_t baudRate) {
    // 打开串口设备（读写模式，不成为控制终端）
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("无法打开串口");
        return -1;
    }

    // 获取当前串口配置
    struct termios tty;
    if (tcgetattr(fd, &tty) == -1) {
        perror("获取串口属性失败");
        close(fd);
        return -1;
    }

    // 设置波特率
    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    // 配置数据格式：8位数据位，无校验位，1个停止位
    tty.c_cflag &= ~PARENB;    // 无校验位
    tty.c_cflag &= ~CSTOPB;    // 1个停止位
    tty.c_cflag &= ~CSIZE;     // 清除数据位设置
    tty.c_cflag |= CS8;        // 8位数据位

    // 禁用硬件流控，启用接收
    tty.c_cflag &= ~CRTSCTS;   // 禁用硬件流控
    tty.c_cflag |= (CLOCAL | CREAD);  // 启用接收，设置本地模式

    // 禁用规范模式和回显
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 禁用软件流控和特殊字符处理
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | 
                     INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;     // 禁用输出处理
    tty.c_oflag &= ~ONLCR;     // 禁用换行转换

    // 设置超时：1秒超时，最小读取0字节
    tty.c_cc[VTIME] = 10;      // 10*100ms = 1秒
    tty.c_cc[VMIN] = 0;

    // 应用配置
    if (tcsetattr(fd, TCSANOW, &tty) == -1) {
        perror("设置串口属性失败");
        close(fd);
        return -1;
    }

    return fd;
}

/**
 * @brief 十六进制字符串转字节数组
 */
std::vector<uint8_t> hexSend(const std::string &hexStr) {
    std::vector<uint8_t> bytes;
    
    for (size_t i = 0; i < hexStr.length(); i += 2) {
        std::string byteString = hexStr.substr(i, 2);
        uint8_t byte = (uint8_t)strtol(byteString.c_str(), nullptr, 16);
        bytes.push_back(byte);
    }
    return bytes;
}

/**
 * @brief 显示串口选择菜单
 */
void displaySerialMenu(const std::vector<std::string>& ports) {
    std::cout << "\n===== 串口选择菜单 =====" << std::endl;
    for (size_t i = 0; i < ports.size(); ++i) {
        std::cout << "  [" << i + 1 << "] " << ports[i] << std::endl;
    }
    std::cout << "  [R] 重新扫描串口" << std::endl;
    std::cout << "  [Q] 退出程序" << std::endl;
    std::cout << "=========================" << std::endl;
    std::cout << "请选择要使用的串口 (1-" << ports.size() << "): ";
}

/**
 * @brief 获取当前时间戳（毫秒）
 */
unsigned long millis() {
    using namespace std::chrono;
    static const auto start = steady_clock::now();
    return duration_cast<milliseconds>(steady_clock::now() - start).count();
}


void RxtErrorHz(void)
{
    static unsigned long ms = millis();

    unsigned long tt = millis() - ms;
    if(tt>1000)
    {
        ms = millis();   
        RX_error_hz = RX_error;
        RX_error = 0;
    }
}



/**
 * @brief 读取并解析串口数据
 */
void SERIAL_RXT(int serialFd) 
{
    constexpr size_t kMaxFrameSize = 256;
    static uint8_t dataArray[kMaxFrameSize] = {};
    static size_t count = 0;
    static uint16_t frameLength = 0;

    auto resetParser = [&]() {
        count = 0;
        frameLength = 0;
    };

    // 检查串口是否有数据可读
    int bytesAvailable;
    ioctl(serialFd, FIONREAD, &bytesAvailable);
    RxtErrorHz();
    while (bytesAvailable > 0) 
    {
        uint8_t dat;
        ssize_t bytesRead = read(serialFd, &dat, 1);
        if (bytesRead != 1) {
            perror("读取串口数据失败");
            break;
        }
        bytesAvailable--;

        if (count == 0) {
            if (dat == 0x7B) dataArray[count++] = dat;
            continue;
        }

        if (count == 1) {
            if (dat == 0x2D || dat == 0x2E || dat == 0x30) {
                dataArray[count++] = dat;
            } else if (dat == 0x7B) {
                dataArray[0] = dat;
            } else {
                resetParser();
            }
            continue;
        }

        if (count >= kMaxFrameSize) {
            resetParser();
            continue;
        }
        dataArray[count++] = dat;

        if (count == 4) {
            frameLength = static_cast<uint16_t>(dataArray[2]) |
                          (static_cast<uint16_t>(dataArray[3]) << 8);
            const bool knownLength =
                (dataArray[1] == 0x2D &&
                 (frameLength == sizeof(ROS_body_t) || frameLength == sizeof(ROS_body_t1))) ||
                (dataArray[1] == 0x2E && frameLength == sizeof(RobotDiagnostic_t)) ||
                (dataArray[1] == 0x30 && frameLength == sizeof(PidTuneTelemetry_t));
            if (!knownLength || frameLength < 6 || frameLength > kMaxFrameSize) {
                // Silently resynchronise. A process may attach halfway through
                // a frame; this is normal and not a hardware length fault.
                resetParser();
                continue;
            }
        }

        if (frameLength == 0 || count < frameLength) continue;

        uint16_t receivedCrc = 0;
        std::memcpy(&receivedCrc, dataArray + frameLength - 2, sizeof(receivedCrc));
        const uint16_t calculatedCrc = crc16(dataArray, frameLength - 2);
        if (calculatedCrc != receivedCrc) {
            RX_error++;
            resetParser();
            continue;
        }

        if (dataArray[1] == 0x2D && frameLength == sizeof(ROS_body_t)) {
            if (Serial_update_flag == 0) {
                std::memcpy(&ROS_body, dataArray, sizeof(ROS_body));
                Serial_update_flag = 1;
            }
        } else if (dataArray[1] == 0x2D && frameLength == sizeof(ROS_body_t1)) {
            if (Serial_update_flag == 0) {
                std::memcpy(&ROS_body1, dataArray, sizeof(ROS_body1));
                Serial_update_flag = 1;
            }
        } else if (dataArray[1] == 0x2E && frameLength == sizeof(RobotDiagnostic_t)) {
            std::memcpy(&RobotDiagnostic, dataArray, sizeof(RobotDiagnostic));
            Diagnostic_update_flag = 1;
        } else if (dataArray[1] == 0x30 && frameLength == sizeof(PidTuneTelemetry_t)) {
            std::memcpy(&PidTuneTelemetry, dataArray, sizeof(PidTuneTelemetry));
            PidTuneTelemetry_update_flag = 1;
        }

        resetParser();
    }
}




