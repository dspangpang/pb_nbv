#include "../include/turntable_ros_controller/TurntableRosControl.h"

// add for reset usb
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>

using namespace itas109;

CSerialPort _sp;
int _countRead = 0;
double _current_angle = 0;
double _current_target_angle = 0;
int _wait_time_ms = 50;
bool listen_flag = true;
unsigned int _currnet_location_state = 0x00;
unsigned long _current_pulse = 0;
int _angle_overflow_cnt = 0;
int _max_overflow_cnt = 20;
std::mutex _mtx;


// joint state publisher
ros::Publisher _turntable_jointstate_pub;

// 超时flag
bool _timeout_flag = false;
// 执行到位flag
bool _execute_flag = false;
// 角度溢出flag
bool _angle_overflow_flag = false;



// 重置usb设备
void reset_usb_device()
{
    libusb_context *ctx = nullptr;
    libusb_device_handle *handle = nullptr;
    int r;

    r = libusb_init(&ctx);
    if (r < 0)
    {
        std::cerr << "Failed to initialize libusb: " << libusb_error_name(r) << std::endl;
        return;
    }

    // 打开设备，假设设备的供应商ID和产品ID已知
    handle = libusb_open_device_with_vid_pid(ctx, 0x0403, 0x6001);
    if (!handle)
    {
        std::cerr << "Failed to open USB device" << std::endl;
        libusb_exit(ctx);
        return;
    }

    // 重置设备
    r = libusb_reset_device(handle);
    if (r < 0)
    {
        std::cerr << "Failed to reset USB device: " << libusb_error_name(r) << std::endl;
    }

    libusb_close(handle);
    libusb_exit(ctx);

    ROS_INFO("reset_usb_device success !");
}

void computeCRC16(const char *data, int len, char *crc)
{
    unsigned short crc16 = 0xFFFF;
    for (int i = 0; i < len; ++i)
    {
        crc16 ^= (unsigned char)data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc16 & 0x0001)
            {
                crc16 = (crc16 >> 1) ^ 0xA001;
            }
            else
            {
                crc16 = crc16 >> 1;
            }
        }
    }

    // 低字节在前，高字节在后
    crc[0] = (crc16 & 0xFF);
    crc[1] = (crc16 >> 8);
}

bool checkCRC16(const char *data, int len)
{
    char *crc = new char[2];
    char *input_data = new char[len-2];

    bool result = false;
    for (size_t i = 0; i < len-2; i++)
    {
        input_data[i] = data[i];
    }
    
    computeCRC16(input_data, len-2, crc);

    if(crc[0] == data[len-2] && crc[1] == data[len-1]){
        result = true;
    }

    return result;
}

// 定义一个倒计时函数
void countdown(int seconds)
{
    ROS_INFO("Countdown: %d", seconds);
    while (seconds >= 0)
    {
        sleep(1);
        seconds--;
        if (_execute_flag)
        {
            return;
        }
    }
    _mtx.lock();
    _timeout_flag = true;
    ROS_ERROR("Timeout");
    _mtx.unlock();
}

double get_current_angle(){
    double angle;
    _mtx.lock();
    angle = _current_angle;
    _mtx.unlock();
    return angle;
}

void set_current_angle(double angle){
    _current_angle = angle;
}

void set_current_target_angle(double angle){
    _mtx.lock();
    _current_target_angle = angle;
    _mtx.unlock();
}

double get_current_target_angle(){
    double angle;
    angle = _current_target_angle;
    return angle;
}

bool get_listen_flag(){
    bool flag;
    _mtx.lock();
    flag = listen_flag;
    _mtx.unlock();
    return flag;
}

void set_listen_flag(bool flag){
    _mtx.lock();
    listen_flag = flag;
    _mtx.unlock();
}

// 设置
std::string char2hexstr(const char *str, int len)
{
    static const char hexTable[17] = "0123456789ABCDEF";

    std::string result;
    for (int i = 0; i < len; ++i)
    {
        result += "0x";
        result += hexTable[(unsigned char)str[i] / 16];
        result += hexTable[(unsigned char)str[i] % 16];
        result += " ";
    }
    return result;
}

class MyListener : public CSerialPortListener
{
public:
    MyListener(CSerialPort *sp)
        : p_sp(sp)
    {
        current_data_.clear();
    };

    ~MyListener() = default;

    void onReadEvent(const char *portName, unsigned int readBufferLen)
    {
        if (readBufferLen > 0)
        {
            char *data = new char[readBufferLen];            
            if (data)
            {
                // read
                int recLen = p_sp->readData(data, readBufferLen);
                
                for (size_t i = 0; i < recLen; i++)
                {
                    current_data_.push_back(data[i]);
                }

                if(current_data_.size() >= 8)
                {   
                    // 如果数据的开头不是0x01, 则从头开始找0x01
                    while(current_data_[0] != 0x01){
                        current_data_.erase(current_data_.begin());
                    }
                    // 创建一个新的向量，并将current_data_的内容复制到新向量中
                    std::vector<u_char> new_vector(current_data_.begin(), current_data_.end());
                    // 使用新向量替换current_data_
                    current_data_.swap(new_vector);
                    
                    // 如果currnet_data_已经有8个字节, 判断一下currnet_data_的第二位是否是0x03
                    if(current_data_.size() >= 8)
                    { 
                        //判断一下currnet_data_的第二位是否是0x03
                        if (current_data_[1] == 0x03)
                        {
                            // 如果是0x03, 则说明是位置查询指令的返回值
                            // 即这一帧还没有接收完毕 需要9个字节才接受完毕
                            // 判断一下是否接受了9个字节
                            // 如果没有接受完整，则继续接受
                            if (current_data_.size() >= 9){
                                // 从currnet_data_中解析出角度 此时一帧数据已经接收完毕
                                // 解析currnet_data_
                                // Convert tmp_data to a hex string
                                unsigned char *tmp_data = new unsigned char[9];
                                std::stringstream ss;
                                for (int i = 0; i < 9; ++i)
                                {
                                    ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)current_data_[i] << " ";
                                    tmp_data[i] = current_data_[i];

                                }
                                // 对数据进行CRC校验
                                std::string data_hex = ss.str();
                                // printf("%s - Count: %d, Length: %d, Hex: %s\n", portName, ++_countRead, recLen, data_hex.c_str());
                                if(checkCRC16((char *)tmp_data, 9)){
                                
                                    // 读取当前位置
                                    // 低位
                                    if (_currnet_location_state == 0x00)
                                    {
                                        _current_pulse = (tmp_data[3] << 8) + tmp_data[4];
                                        _currnet_location_state = 0x01;
                                    }
                                    else if (_currnet_location_state == 0x01)
                                    {
                                        _current_pulse += (tmp_data[3] << 24) + (tmp_data[4] << 16);
                                        _currnet_location_state = 0x02;
                                    }

                                    if(_currnet_location_state == 0x02){
                                        
                                        _currnet_location_state = 0x00;
                                        // 实际模型的角度为 -180~180
                                        double current_angle = _current_pulse / 500.0 - 180.0;
                                        if(current_angle >= -180.0 && current_angle <= 180.0){
                                            set_current_angle(current_angle);
                                        }else{
                                            // 如果角度超出范围10度，则将角度设置为-180~180之间
                                            if (abs(current_angle) < 185.0)
                                            {
                                                if(current_angle > 180.0){
                                                    set_current_angle(180.0);
                                                }else if(current_angle < -180.0){
                                                    set_current_angle(-180.0);
                                                }
                                            }else{
                                                std::cout << "Current Wrong Pulse: " << _current_pulse << std::endl;
                                                std::cout << "Current Wrong Angle: " << current_angle << std::endl;
                                                if (_angle_overflow_cnt < _max_overflow_cnt)
                                                {
                                                    _angle_overflow_cnt++;
                                                }else{
                                                    _angle_overflow_cnt = 0;
                                                    _mtx.lock();
                                                    _angle_overflow_flag = true;
                                                    _mtx.unlock();
                                                }
                                            }

                                        }
                                    }

                                    // 清空currnet_data_的前七位,并把后第八位放在第一位
                                    std::vector<u_char>::iterator it;
                                    for(it = current_data_.begin(); it != current_data_.end() && it != current_data_.begin()+9; ){
                                        it = current_data_.erase(it);
                                    }
                                    // 创建一个新的向量，并将current_data_的内容复制到新向量中
                                    std::vector<u_char> new_vector(current_data_.begin(), current_data_.end());
                                    // 使用新向量替换current_data_
                                    current_data_.swap(new_vector);
                                    
                                    delete tmp_data;
                                }else{
                                    // 如果CRC校验失败，则清空currnet_data_的前七位,并把后第八位放在第一位
                                    ROS_WARN("CRC16 Check Failed, size 9, get a wrong data!");
                                    std::vector<u_char>::iterator it;
                                    for(it = current_data_.begin(); it != current_data_.end() && it != current_data_.begin()+9; ){
                                        it = current_data_.erase(it);
                                    }
                                    // 创建一个新的向量，并将current_data_的内容复制到新向量中
                                    std::vector<u_char> new_vector(current_data_.begin(), current_data_.end());
                                    // 使用新向量替换current_data_
                                    current_data_.swap(new_vector);

                                    delete tmp_data;
                                }
                            }
                            
                        }
                        else{
                            // 如果不是0x03, 则说明是其他指令的返回值
                            // 此时一帧数据已经接收完毕
                            std::stringstream ss;
                            unsigned char *tmp_data = new unsigned char[8];
                            for (int i = 0; i < 8; ++i)
                            {
                                ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)current_data_[i] << " ";
                                tmp_data[i] = current_data_[i];
                            }
                            // 对数据进行CRC校验
                            std::string data_hex = ss.str();
                            // printf("%s - Count: %d, Length: %d, Hex: %s\n", portName, ++_countRead, recLen, data_hex.c_str());
                            if(checkCRC16((char *)tmp_data, 8)){

                                // 清空currnet_data_的前8位,并把后第八位放在第一位
                                std::vector<u_char>::iterator it;
                                for(it = current_data_.begin(); it != current_data_.end() && it != current_data_.begin()+8; ){
                                    it = current_data_.erase(it);
                                }
                                // 创建一个新的向量，并将current_data_的内容复制到新向量中
                                std::vector<u_char> new_vector(current_data_.begin(), current_data_.end());
                                // 使用新向量替换current_data_
                                current_data_.swap(new_vector);
                            }else{
                                // 如果CRC校验失败，则清空currnet_data_的前8位,并把后第八位放在第一位
                                ROS_WARN("CRC16 Check Failed, size 8, get a wrong data!");
                                std::vector<u_char>::iterator it;
                                for(it = current_data_.begin(); it != current_data_.end() && it != current_data_.begin()+8; ){
                                    it = current_data_.erase(it);
                                }
                                // 创建一个新的向量，并将current_data_的内容复制到新向量中
                                std::vector<u_char> new_vector(current_data_.begin(), current_data_.end());
                                // 使用新向量替换current_data_
                                current_data_.swap(new_vector);

                                delete tmp_data;
                            }

                        }
                    }
                }
            }

            delete[] data;
            data = NULL;
        }
    };

private:
    CSerialPort *p_sp;

    // 返回的完整指令一个9或者8个字节，但是可能会分多次接收，所以需要缓存数据
    std::vector<u_char> current_data_;
};

/*r/min*/
void sendTargetSpeed(unsigned int speed){
    // 设置目标速度
    char hex[6];
    hex[0] = 0x01;
    hex[1] = 0x06;
    hex[2] = 0x00;
    hex[3] = 0x36;

    hex[4] = (speed >> 8) & 0xFF;
    hex[5] = speed & 0xFF;

    char hex_with_crc[8];
    memcpy(hex_with_crc, hex, 6);
    char crc[2];
    computeCRC16(hex, 6, crc);
    hex_with_crc[6] = crc[0];
    hex_with_crc[7] = crc[1];
    
    _sp.writeData(hex_with_crc, sizeof(hex_with_crc));
    usleep(1000 * _wait_time_ms); // sleep  等待数据回应

    // std::stringstream ss;
    // for (int i = 0; i < 13; ++i)
    // {
    //     ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)hex_with_crc[i] << " ";

    // }
    // std::string data_hex = ss.str();

    // printf("Hex: %s\n", data_hex.c_str());

}

void sendInitSig(){

    // 设置细分脉冲为 1000
    char hex[8];
    hex[0] = 0x01;
    hex[1] = 0x06;
    hex[2] = 0x00;
    hex[3] = 0x23;
    hex[4] = 0x03;
    hex[5] = 0xe8;
    hex[6] = 0x78;
    hex[7] = 0xbe;

    _sp.writeData(hex, sizeof(hex));
    usleep(1000 * _wait_time_ms); // sleep 等待数据回应


    sendTargetSpeed(400);

}

void sendStartSig(){
    // 启动圆盘
    char hex[8];
    hex[0] = 0x01;
    hex[1] = 0x06;
    hex[2] = 0x00;
    hex[3] = 0x4e;
    hex[4] = 0x00;
    hex[5] = 0x01;
    hex[6] = 0x28;
    hex[7] = 0x1d;
    
    _sp.writeData(hex, sizeof(hex));
    usleep(1000 * _wait_time_ms);
}

void sendTargetSig(double angle){
    // 设置目标角度
    char hex[11];
    hex[0] = 0x01;
    hex[1] = 0x10;
    hex[2] = 0x00;
    hex[3] = 0x37;
    hex[4] = 0x00;
    hex[5] = 0x02;
    hex[6] = 0x04;

    // 判断当前角度与目标角度的差值
    double current_angle = get_current_angle();
    double diff = angle - current_angle;
    
    // 根据diff 计算目标脉冲数 1000脉冲对应2度
    long target_pulse = (long)(diff * 500.0);
    std::cout << "target pulse: " << target_pulse << std::endl;
    // 把target_pulse转换为32位整数
    hex[7] = (target_pulse >> 24) & 0xFF;
    hex[8] = (target_pulse >> 16) & 0xFF;
    hex[9] = (target_pulse >> 8) & 0xFF;
    hex[10] = target_pulse & 0xFF;

    char hex_with_crc[13];
    memcpy(hex_with_crc, hex, 11);
    char crc[2];
    computeCRC16(hex, 11, crc);
    hex_with_crc[11] = crc[0];
    hex_with_crc[12] = crc[1];
    
    _sp.writeData(hex_with_crc, sizeof(hex_with_crc));
    usleep(1000 * _wait_time_ms); // sleep  等待数据回应

    // std::stringstream ss;
    // for (int i = 0; i < 13; ++i)
    // {
    //     ss << std::hex << std::setw(2) << std::setfill('0') << (int)(unsigned char)hex_with_crc[i] << " ";

    // }
    // std::string data_hex = ss.str();

    // printf("Hex: %s\n", data_hex.c_str());

}

// 发送停止指令
void sendStopSig(){
    // 停止圆盘
    char hex[8];
    hex[0] = 0x01;
    hex[1] = 0x06;
    hex[2] = 0x00;
    hex[3] = 0x4e;
    hex[4] = 0x00;
    hex[5] = 0x20;
    hex[6] = 0xe8;
    hex[7] = 0x05;
    
    _sp.writeData(hex, sizeof(hex));
    usleep(1000 * _wait_time_ms); // sleep  等待数据回应
}

void sendLocationQuerySig(){
    // 查询当前位置

    // l
    char hex[8];
    hex[0] = 0x01;
    hex[1] = 0x03;
    hex[2] = 0x00;
    hex[3] = 0x09;
    hex[4] = 0x00;
    hex[5] = 0x02;
    hex[6] = 0x14;
    hex[7] = 0x09;

    _sp.writeData(hex, sizeof(hex));
    usleep(1000 * _wait_time_ms); // sleep 100ms 等待数据回应
    
    // h
    hex[0] = 0x01;
    hex[1] = 0x03;
    hex[2] = 0x00;
    hex[3] = 0x08;
    hex[4] = 0x00;
    hex[5] = 0x02;
    hex[6] = 0x45;
    hex[7] = 0xc9;
    
    _sp.writeData(hex, sizeof(hex));
    usleep(1000 * _wait_time_ms); // sleep 100ms 等待数据回应

}

// 新建线程用于实时发送位置查询指令
void sendLocationQuerySigThread(){

    while (ros::ok())
    {
        if(get_listen_flag()){
            sendLocationQuerySig();   
        }

        usleep(1000*_wait_time_ms);
    }
    
}

// 新建线程发布 joint state数据
void publishJointStateThread(){
    ros::Rate r(50);
    while (ros::ok())
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"turntable_support_joint"};
        joint_state.position = {get_current_angle() / 180.0 * M_PI};
        _turntable_jointstate_pub.publish(joint_state);
        r.sleep();
    }
}

TurntableRosControl::TurntableRosControl(const ros::NodeHandle &nh, const std::string& action_name) :
        as_(nh, action_name, [this](auto && PH1)
        { executeCB(std::forward<decltype(PH1)>(PH1)); },
            false), action_name_(action_name)
{
    this->nh_ = nh;

    // 初始化CSerialPort
    printf("Version: %s\n\n", _sp.getVersion());

    MyListener listener(&_sp);

    std::vector<SerialPortInfo> m_availablePortsList = CSerialPortInfo::availablePortInfos();

    printf("AvailableFriendlyPorts:\n");

    int availablePortCount = (int)m_availablePortsList.size();

    for (int i = 1; i <= availablePortCount; ++i)
    {
        SerialPortInfo serialPortInfo = m_availablePortsList[i - 1];
        printf("%d - %s %s %s\n", i, serialPortInfo.portName, serialPortInfo.description, serialPortInfo.hardwareId);
    }

    if (m_availablePortsList.size() == 0)
    {
        printf("No valid port\n");
    }
    else
    {
        std::cout << std::endl;

        // 遍历所有serialPortInfo.description，找到包含“FTDI”的串口
        int input = 0;
        for (int i = 0; i < m_availablePortsList.size(); ++i)
        {
            if (strstr(m_availablePortsList[i].description, "FTDI"))
            {
                printf("Found FTDI Port: %s\n", m_availablePortsList[i].portName);
                input = i;
                break;
            }
        }

        const char *portName = m_availablePortsList[input].portName;
        printf("Port Name: %s\n", portName);

        _sp.init(portName,              // windows:COM1 Linux:/dev/ttyS0
                itas109::BaudRate9600, // baudrate
                itas109::ParityNone,   // parity
                itas109::DataBits8,    // data bit
                itas109::StopOne,      // stop bit
                itas109::FlowNone,     // flow
                4096                   // read buffer size
        );
        _sp.setReadIntervalTimeout(0); // read interval timeout 0ms

        _sp.open();
        printf("Open %s %s\n", portName, _sp.isOpen() ? "Success" : "Failed");
        printf("Code: %d, Message: %s\n", _sp.getLastError(), _sp.getLastErrorMsg());

        // connect for read
        _sp.connectReadEvent(&listener);
    }

    sendInitSig();
    // 设置运动规划的时间容限为4倍规划时间
    ros::param::set("move_group/trajectory_execution/allowed_execution_duration_scaling", 8.0);
    _turntable_jointstate_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 50);

    // 新建一个线程用于实时发送位置查询指令
    std::thread t1(sendLocationQuerySigThread);
    t1.detach();

    std::thread t2(publishJointStateThread);
    t2.detach();
    // 初始化发布器
    sleep(1); // 等待as_初始化完成

    as_.start();

    ros::spin();
    
}

TurntableRosControl::~TurntableRosControl() = default;


void TurntableRosControl::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {

    // helper variables
    ros::Rate r(10);
    bool success = true;

    // set controller type, joints and constraints
    // ...

    // publish info to the console for the user
    ROS_INFO("%s: Executing goal for %ld points", action_name_.c_str(), goal->trajectory.points.size());
 
    int size = goal->trajectory.points.back().positions.size();
    // 弧度转角度
    double rad = goal->trajectory.points.back().positions[0];// 只有一个关节可移动
    double angle = rad * 180 / M_PI;
    std::cout << "target angle: "<< angle << std::endl;
    set_current_target_angle(angle);
    // 发送运动指令
    set_listen_flag(false);
    usleep(1000 * _wait_time_ms);
    sendTargetSig(angle);
    sendStartSig();
    set_listen_flag(true);

    double cur = get_current_angle();
    feedback_.header.stamp = ros::Time::now();
    feedback_.joint_names = {"turntable_support_joint"};
    feedback_.actual.positions = {cur / 180.0 * M_PI};
    as_.publishFeedback(feedback_);

    // 新建一个线程用于倒计时
    _timeout_flag = false;
    _execute_flag = false;
    // 速度为0.1609秒/度 所以设置超时时间为目标角度与当前角度的差值乘以0.17
    int seconds = (int)(abs(angle - cur) * 0.17);
    std::thread t1(countdown, seconds);
    t1.detach();

    // 每5次循环发布一次cout
    int cnt = 0;
    while((cur - angle > 0.002) || (cur - angle < -0.002)){
        r.sleep();
        cnt ++;
        cur = get_current_angle();
        feedback_.header.stamp = ros::Time::now();
        feedback_.joint_names = {"turntable_support_joint"};
        feedback_.actual.positions = {cur / 180.0 * M_PI};
        as_.publishFeedback(feedback_);
        if (cnt % 5 == 0)
        {
            std::cout << "get_current_angle:" << cur << "\r\n" << "target angle: " << angle << std::endl;
        }
        if (_timeout_flag || _angle_overflow_flag)
        {
            sendStopSig();
            break;
        }
        
    }
    // 结束后再发布一次确认位置
    if (!_angle_overflow_flag)
    {
        cur = get_current_angle();
        feedback_.header.stamp = ros::Time::now();
        feedback_.joint_names = {"turntable_support_joint"};
        feedback_.actual.positions = {cur / 180.0 * M_PI};
        as_.publishFeedback(feedback_);
        std::cout << "get_current_angle:" << cur << "\r\n" << "target angle: " << angle << std::endl;
    }

    // 返回结果
    result_.error_code = result_.SUCCESSFUL;
    if (_timeout_flag || _angle_overflow_flag)
    {
        result_.error_code = result_.INVALID_JOINTS;
        as_.setAborted(result_);
    }else
    {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
        _angle_overflow_cnt = 0;
    }
    
    // set the action state to succeeded子主题
    _mtx.lock();
    _execute_flag = true;
    _mtx.unlock();
    if (_timeout_flag || _angle_overflow_flag)
    {
        ROS_INFO("%s: Failed", action_name_.c_str());
        reset_usb_device();
        if (_timeout_flag)
        {
            ROS_ERROR("Timeout, reset usb device!");
        }
        if (_angle_overflow_flag)
        {
            ROS_ERROR("Angle Overflow, Restart dirver!");
        }
        exit(-1);
    }
}
