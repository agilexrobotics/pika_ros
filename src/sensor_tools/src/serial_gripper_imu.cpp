//sensor_tools.cpp
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <data_msgs/Gripper.h>
#include <data_msgs/CaptureService.h>
#include "jsoncpp/json/json.h"

#include <boost/asio.hpp>
#include <thread>
#include <cmath>
#include <iomanip>

#include <data_msgs/CaptureStatus.h>
#include <data_msgs/TeleopStatus.h>
#include <data_msgs/LocalizationStatus.h>
#include <data_msgs/ArmControlStatus.h>

#include <limits.h>

enum Send_Flag{
	DISABLE = 10,
	ENABLE = 11,
	SET_ZERO = 12,
	VELOCITY_CTRL = 13,
	EFFORT_CTRL = 15,
	POSITION_CTRL_MIT = 22,
	POSITION_CTRL_POS_VEL = 23,
	LIGHT_CTRL = 50,
	VIBRATE_CTRL = 51
};

enum Color{
	COLOR_WHITE = 0,
	COLOR_RED = 1,
	COLOR_GREEN = 2,
	COLOR_BLUE = 3,
	COLOR_YELLOW = 4,
	COLOR_SIZE = 5
};

enum Vibrate{
	VIBRATE_NONE = 0,
	VIBRATE_ONE = 1,
	VIBRATE_SIZE = 2
};

bool find_json(std::string &msg, int &start, int &end){
	std::vector<int> stack;
	int count = 0;
	for(int i=0; i<msg.size(); i++){
		char ch = (char)msg[i];
		if(ch == '{'){
			stack.push_back(i);
		}else if(ch == '}'){
			if(!stack.empty()){
				int index = stack.back();
				stack.pop_back();
				if(stack.empty() || (index > 0 && msg[index-1]!=':')){
					start = index;
					end = i;
					return true;
				}
			}
		}
	}
	return false;
}

class RosOperator{
	public:
	bool isGripper = false;
	std::string msg;
	std::string serialPort;
	boost::asio::io_context io_context;
	std::unique_ptr<boost::asio::serial_port> serial;
	std::mutex serialMtx;

	std::string jointName;

	std::string ctrlMode;
	
  	ros::Publisher pubImu;
	ros::Publisher pubGripper;
	ros::Subscriber subGripper;
	ros::Subscriber subJointStateCtrl;
	ros::Subscriber subJointStateInfo;
	ros::Publisher pubArmJointStateWithGripper;
	ros::Publisher pubGripperJointState;

	ros::Subscriber subDataCaptureStatus;
	ros::Subscriber subTeleopStatus;
	ros::Subscriber subLocalizationStatus;
	ros::Subscriber subArmControlStatus;

	ros::ServiceClient client;
	ros::ServiceClient client_arm_teleop;

	std::mutex receiveDataMtx;
	float effort;
	float velocity;

	float angle;
	float distance;
	float motorCurrent;
	float voltage;
	float driver_temp;
	float motor_temp;
	float bus_current;
	std::string status;
	bool enable;

	int command;

	float lastCommandAngle;

	float motorCurrentLimit;
	float motorCurrentRedundancy;
	float ctrlRate;
	float ctrlFreq;

	std::vector<bool> colorStatus;
	std::mutex colorStatusMtx;
	std::vector<bool> vibrateStatus;
	std::mutex vibrateStatusMtx;

	std::thread *receivingThread;
	std::thread *statusSendingThread;

	bool mitMode;

	ros::NodeHandle *nh;

	template<typename T>
	std::vector<uint8_t> createBinaryCommand(uint8_t cmd, std::vector<T> values = std::vector<T>{0.0f}, bool bigEndian = false) {
		std::vector<uint8_t> binaryCmd;
		binaryCmd.push_back(cmd);
		if(bigEndian) {
			for(int i=0; i<values.size(); i++){
				T value = values.at(i);
				union {
					T f;
					uint32_t u;
				} converter;
				converter.f = value;
				uint32_t pad = converter.u;
				uint8_t bytes[4];
				bytes[0] = (pad >> 24) & 0xFF;
				bytes[1] = (pad >> 16) & 0xFF;
				bytes[2] = (pad >> 8) & 0xFF;
				bytes[3] = pad & 0xFF;
				for(int i = 0; i < 4; i++) {
					binaryCmd.push_back(bytes[i]);
				}
			}
		} else {
			for(int i=0; i<values.size(); i++){
				T value = values.at(i);
				const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
				for(int i = 0; i < 4; i++) {
					binaryCmd.push_back(bytes[i]);
				}
			}
		}
		binaryCmd.push_back('\r');
		binaryCmd.push_back('\n');
		return binaryCmd;
	}

    RosOperator(){
		nh = new ros::NodeHandle("~");
		nh->param<std::string>("serial_port", serialPort, "/dev/ttyUSB0");
		nh->param<float>("motor_current_limit", motorCurrentLimit, 1000);
		nh->param<float>("motor_current_redundancy", motorCurrentRedundancy, 500);
		nh->param<float>("ctrl_rate", ctrlRate, 50);
		nh->param<bool>("mit_mode", mitMode, true);
		ctrlFreq = 1.0/ctrlRate;
        char resolvedPath[PATH_MAX];
		if(serialPort == "/dev/ttyUSB60" || serialPort == "/dev/ttyUSB61")
			isGripper = true;
        int ret = readlink(serialPort.c_str(), resolvedPath, sizeof(resolvedPath));
		if(ret >= 0){
			serialPort = "/dev/" + std::string(resolvedPath);
		}
		nh->param<std::string>("joint_name", jointName, "center_joint");
		nh->param<std::string>("ctrl_mode", ctrlMode, "collection");

		// std::cout<<serialPort<<std::endl;

		for(int i=0; i<COLOR_SIZE; i++){
			colorStatus.push_back(false);
		}
		colorStatus[COLOR_WHITE] = true;

		for(int i=0; i<VIBRATE_SIZE; i++){
			vibrateStatus.push_back(false);
		}
		vibrateStatus[VIBRATE_NONE] = true;

		effort = -1;
		velocity = -1;
		
		angle = 0;
		distance = 0;
		motorCurrent = 0;
		voltage = 0;
		driver_temp = 0;
		motor_temp = 0;
		bus_current = 0;
		status = "";
		enable = true;
		command = -1;

		lastCommandAngle = -1;
		
		// 初始化线程指针
		receivingThread = nullptr;
		statusSendingThread = nullptr;
		
		// 初始化serial指针
		serial = nullptr;
    }

	~RosOperator(){
		// 等待并清理线程
		if(receivingThread && receivingThread->joinable()){
			receivingThread->join();
			delete receivingThread;
			receivingThread = nullptr;
		}
		if(statusSendingThread && statusSendingThread->joinable()){
			statusSendingThread->join();
			delete statusSendingThread;
			statusSendingThread = nullptr;
		}
		// 关闭串口
		if(serial && serial->is_open()){
			serial->close();
		}
		serial.reset();
	}

	bool initSerial(){
		try {
			serial = std::make_unique<boost::asio::serial_port>(io_context, serialPort);
			serial->set_option(boost::asio::serial_port_base::baud_rate(460800));
			serial->set_option(boost::asio::serial_port_base::character_size(8));
			serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
			serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
			serial->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
			
			// ros::NodeHandle nh("~");
			pubImu = nh->advertise<sensor_msgs::Imu>("/imu/data", 1);
			pubGripper = nh->advertise<data_msgs::Gripper>("/gripper/data", 1);
			subGripper = nh->subscribe<data_msgs::Gripper>("/gripper/ctrl", 1, &RosOperator::gripperCtrlHandler, this, ros::TransportHints().tcpNoDelay());

			pubGripperJointState = nh->advertise<sensor_msgs::JointState>("/gripper/joint_state", 1);
			subJointStateCtrl = nh->subscribe<sensor_msgs::JointState>("/gripper/joint_state_ctrl", 1, &RosOperator::jointStateCtrlHandler, this, ros::TransportHints().tcpNoDelay());

			subJointStateInfo = nh->subscribe<sensor_msgs::JointState>("/joint_state_info", 1, &RosOperator::jointStateInfoHandler, this, ros::TransportHints().tcpNoDelay());
			pubArmJointStateWithGripper = nh->advertise<sensor_msgs::JointState>("/joint_state_gripper", 1);

			subDataCaptureStatus = nh->subscribe<data_msgs::CaptureStatus>("/data_capture_status", 1, &RosOperator::dataCaptureStatusHandler, this, ros::TransportHints().tcpNoDelay());
			subTeleopStatus = nh->subscribe<data_msgs::TeleopStatus>("/teleop_status", 1, &RosOperator::teleopStatusHandler, this, ros::TransportHints().tcpNoDelay());
			subLocalizationStatus = nh->subscribe<data_msgs::LocalizationStatus>("/localization_status", 1, &RosOperator::localizationStatusHandler, this, ros::TransportHints().tcpNoDelay());
			subArmControlStatus = nh->subscribe<data_msgs::ArmControlStatus>("/arm_control_status", 1, &RosOperator::armControlStatusHandler, this, ros::TransportHints().tcpNoDelay());

			client = nh->serviceClient<data_msgs::CaptureService>("/data_tools_dataCapture/capture_service");
			client_arm_teleop = nh->serviceClient<std_srvs::Trigger>("/teleop_trigger");

			receivingThread = new std::thread(&RosOperator::receiving, this);
			if(!isGripper)
				statusSendingThread = new std::thread(&RosOperator::statusSending, this);
			motorCurrentLimit = motorCurrentLimit/1000;
			std::vector<uint8_t> command = createBinaryCommand<float>(EFFORT_CTRL, std::vector<float>{static_cast<float>(motorCurrentLimit)});
			std::lock_guard<std::mutex> serialLock(serialMtx);
			if(serial && serial->is_open()){
				boost::asio::write(*serial, boost::asio::buffer(command));
			}
			return true;
		} catch (boost::system::system_error& e) {
			ROS_ERROR("Failed to open serial port: %s", e.what());
			return false;
		}
	}

	void statusSending(){
		ros::Rate rate1(50);
		ros::Rate rate2(100);
		int lastColorStatus = -1;
		double lastColorStatusTime = -1;
		while(ros::ok()){
			// 处理颜色状态
			{
				std::lock_guard<std::mutex> colorLock(colorStatusMtx);
				if(colorStatus[COLOR_BLUE]){
					if(lastColorStatus == COLOR_BLUE){
						if(ros::Time::now().toSec() - lastColorStatusTime > 1){
							colorStatus[COLOR_BLUE] = false;
						}
					}
				}
				int nowColorStatus;
				if(colorStatus[COLOR_BLUE])
					nowColorStatus = COLOR_BLUE;
				else if(colorStatus[COLOR_RED])
					nowColorStatus = COLOR_RED;
				else if(colorStatus[COLOR_YELLOW])
					nowColorStatus = COLOR_YELLOW;
				else if(colorStatus[COLOR_GREEN])
					nowColorStatus = COLOR_GREEN;
				else
					nowColorStatus = COLOR_WHITE;
					
				if(nowColorStatus != lastColorStatus){
					lastColorStatusTime = ros::Time::now().toSec();
					lastColorStatus = nowColorStatus;
				}
				std::vector<uint8_t> command = createBinaryCommand<int>(LIGHT_CTRL, std::vector<int>{static_cast<int>(nowColorStatus)}, true);
				std::lock_guard<std::mutex> serialLock(serialMtx);
				if(serial && serial->is_open()){
					boost::asio::write(*serial, boost::asio::buffer(command));
				}
			}
			rate2.sleep();
			// 处理振动状态
			{
				int nowVibrateStatus = VIBRATE_NONE;
				{
					std::lock_guard<std::mutex> vibrateLock(vibrateStatusMtx);
					if(vibrateStatus[VIBRATE_ONE]){
						nowVibrateStatus = VIBRATE_ONE;
						vibrateStatus[VIBRATE_ONE] = false;
					}
				}
				
				if(nowVibrateStatus != VIBRATE_NONE){
					std::vector<uint8_t> command = createBinaryCommand<int>(VIBRATE_CTRL, std::vector<int>{static_cast<int>(nowVibrateStatus)}, true);
					std::lock_guard<std::mutex> serialLock(serialMtx);
					if(serial && serial->is_open()){
						boost::asio::write(*serial, boost::asio::buffer(command));
					}
				}
			}
			rate1.sleep();
		}
	}

	void dataCaptureStatusHandler(const data_msgs::CaptureStatus::ConstPtr& msg){
		std::lock_guard<std::mutex> lock(colorStatusMtx);
		if(msg->fail){
			colorStatus[COLOR_YELLOW] = true;
		}else if(!msg->quit){
			colorStatus[COLOR_GREEN] = true;
		}else{
			colorStatus[COLOR_GREEN] = false;
			colorStatus[COLOR_YELLOW] = false;
		}
	}

	void teleopStatusHandler(const data_msgs::TeleopStatus::ConstPtr& msg){
		std::lock_guard<std::mutex> lock(colorStatusMtx);
		if(msg->fail){
			colorStatus[COLOR_YELLOW] = true;
		}else if(!msg->quit){
			colorStatus[COLOR_GREEN] = true;
		}else{
			colorStatus[COLOR_GREEN] = false;
			colorStatus[COLOR_YELLOW] = false;
		}
	}

	void localizationStatusHandler(const data_msgs::LocalizationStatus::ConstPtr& msg){
		std::lock_guard<std::mutex> lock(colorStatusMtx);
		if(msg->accurate){
			colorStatus[COLOR_RED] = false;
		}else{
			colorStatus[COLOR_RED] = true;
		}
	}

	void armControlStatusHandler(const data_msgs::ArmControlStatus::ConstPtr& msg){
		if(msg->over_limit){
			std::lock_guard<std::mutex> lock(vibrateStatusMtx);
			vibrateStatus[VIBRATE_ONE] = true;
		}
	}

	void jointStateCtrlHandler(const sensor_msgs::JointState::ConstPtr& msg){
		static double jointStateCtrlTime = -1;
		if(msg->header.stamp.toSec() - jointStateCtrlTime < ctrlFreq)
			return;
		jointStateCtrlTime = msg->header.stamp.toSec();
		receiveDataMtx.lock();
		bool enable = this->enable;
		float velocity = this->velocity;
		float effort = this->effort;
		receiveDataMtx.unlock();
		if(!enable){
			std::vector<uint8_t> command = createBinaryCommand<float>(ENABLE, std::vector<float>{0.0f});
			std::lock_guard<std::mutex> lock(serialMtx);
			if(serial && serial->is_open()){
				boost::asio::write(*serial, boost::asio::buffer(command));
			}
		}
		if(msg->effort.size() > 0 && msg->effort.back() != 0 && effort != msg->effort.back()){
			effort = msg->effort.back();
			this->effort = effort;
			std::vector<uint8_t> command = createBinaryCommand<float>(EFFORT_CTRL, std::vector<float>{effort});
			std::lock_guard<std::mutex> lock(serialMtx);
			if(serial && serial->is_open()){
				boost::asio::write(*serial, boost::asio::buffer(command));
			}
		}
		if(msg->velocity.size() > 0 && msg->velocity.back() != 0 && velocity != msg->velocity.back()){
			velocity = msg->velocity.back();
			this->velocity = velocity;
			std::vector<uint8_t> command = createBinaryCommand<float>(VELOCITY_CTRL, std::vector<float>{velocity, velocity});
			std::lock_guard<std::mutex> lock(serialMtx);
			if(serial && serial->is_open()){
				boost::asio::write(*serial, boost::asio::buffer(command));
			}
		}
		float distance = msg->position.back();
		distance = distance > 0.098?0.098:distance;
		distance = distance < 0?0:distance;
		float angle = getAngle((distance/2+getDistance(0)));
		angle = angle > 1.67?1.67:angle;
		angle = angle < 0?0:angle;
		receiveDataMtx.lock();
		float motorCurrent = this->motorCurrent;
		float motorAngle = this->angle;
		receiveDataMtx.unlock();
		// if(fabs(motorCurrent) > motorCurrentLimit && motorCurrentLimit > 0){
		// 	if(motorCurrent < 0 && angle < motorAngle)
		// 		return;
		// 	if(motorCurrent > 0 && angle > motorAngle)
		// 		return;
		// }
		std::vector<uint8_t> command = createBinaryCommand<float>(mitMode?POSITION_CTRL_MIT:POSITION_CTRL_POS_VEL, std::vector<float>{angle});
		std::lock_guard<std::mutex> lock(serialMtx);
		if(serial && serial->is_open()){
			boost::asio::write(*serial, boost::asio::buffer(command));
			lastCommandAngle = angle;
		}
	}

	void jointStateInfoHandler(const sensor_msgs::JointState::ConstPtr& msg){
		sensor_msgs::JointState jointState = *msg;
		if(jointState.position.size() < 7) {
			jointState.position.resize(7);
		}
		receiveDataMtx.lock();
		jointState.position[6] = distance;
		receiveDataMtx.unlock();
		pubArmJointStateWithGripper.publish(jointState);
	}

    void gripperCtrlHandler(const data_msgs::Gripper::ConstPtr& msg){
		static double gripperCtrlTime = -1;
		if(msg->header.stamp.toSec() - gripperCtrlTime < ctrlFreq)
			return;
		gripperCtrlTime = msg->header.stamp.toSec();
		receiveDataMtx.lock();
		bool enable = this->enable;
		float velocity = this->velocity;
		float effort = this->effort;
		receiveDataMtx.unlock();
		if(msg->enable != enable){
			if(msg->enable){
				std::vector<uint8_t> command = createBinaryCommand<float>(ENABLE, std::vector<float>{0.0f});
				std::lock_guard<std::mutex> lock(serialMtx);
				if(serial && serial->is_open()){
					boost::asio::write(*serial, boost::asio::buffer(command));
				}
			}else{
				std::vector<uint8_t> command = createBinaryCommand<float>(DISABLE, std::vector<float>{0.0f});
				std::lock_guard<std::mutex> lock(serialMtx);
				if(serial && serial->is_open()){
					boost::asio::write(*serial, boost::asio::buffer(command));
				}
			}
		}else if(msg->set_zero){
			std::vector<uint8_t> command = createBinaryCommand<float>(SET_ZERO, std::vector<float>{0.0f});
			std::lock_guard<std::mutex> lock(serialMtx);
			if(serial && serial->is_open()){
				boost::asio::write(*serial, boost::asio::buffer(command));
			}
		}else{
			if(msg->effort != 0 && msg->effort != effort){
				std::vector<uint8_t> command = createBinaryCommand<float>(EFFORT_CTRL, std::vector<float>{msg->effort});
				effort = msg->effort;
				this->effort = effort;
				std::lock_guard<std::mutex> lock(serialMtx);
				if(serial && serial->is_open()){
					boost::asio::write(*serial, boost::asio::buffer(command));
				}
			}
			if(msg->velocity != 0 && msg->velocity != velocity){
				std::vector<uint8_t> command = createBinaryCommand<float>(VELOCITY_CTRL, std::vector<float>{msg->velocity, msg->velocity});
				velocity = msg->velocity;
				this->velocity = velocity;
				std::lock_guard<std::mutex> lock(serialMtx);
				if(serial && serial->is_open()){
					boost::asio::write(*serial, boost::asio::buffer(command));
				}
			}
			float angle = msg->angle;
			float distance = msg->distance;
			if (distance != 0){
				distance = distance > 0.098?0.098:distance;
				distance = distance < 0?0:distance;
				angle = getAngle((distance/2+getDistance(0)));
			}
			angle = angle > 1.67?1.67:angle;
			angle = angle < 0?0:angle;
			receiveDataMtx.lock();
			float motorCurrent = this->motorCurrent;
			float motorAngle = this->angle;
			receiveDataMtx.unlock();
			// if(fabs(motorCurrent) > motorCurrentLimit && motorCurrentLimit > 0){
			// 	if(motorCurrent < 0 && angle < motorAngle)
			// 		return;
			// 	if(motorCurrent > 0 && angle > motorAngle)
			// 		return;
			// }
			std::vector<uint8_t> command = createBinaryCommand<float>(mitMode?POSITION_CTRL_MIT:POSITION_CTRL_POS_VEL, std::vector<float>{angle});
			std::lock_guard<std::mutex> lock(serialMtx);
			if(serial && serial->is_open()){
				boost::asio::write(*serial, boost::asio::buffer(command));
				lastCommandAngle = angle;
			}
		}
    }

	std::string stringToHex(const std::string& input) {
		std::stringstream ss;
		ss << std::hex << std::setfill('0');
		for (unsigned char c : input) {
			ss << std::hex << std::setw(2) << static_cast<int>(c);
		}
		return ss.str();
	}

	int c2i(char ch)  
	{  
			// 如果是数字，则用数字的ASCII码减去48, 如果ch = '2' ,则 '2' - 48 = 2  
			if(isdigit(ch))  
					return ch - 48;  
	
			// 如果是字母，但不是A~F,a~f则返回  
			if( ch < 'A' || (ch > 'F' && ch < 'a') || ch > 'z' )  
					return -1;  
	
			// 如果是大写字母，则用数字的ASCII码减去55, 如果ch = 'A' ,则 'A' - 55 = 10  
			// 如果是小写字母，则用数字的ASCII码减去87, 如果ch = 'a' ,则 'a' - 87 = 10  
			if(isalpha(ch))  
					return isupper(ch) ? ch - 55 : ch - 87;  
	
			return -1;  
	}  

	int hex2dec(const char *hex)  
	{  
			int len;  
			int num = 0;  
			int temp;  
			int bits;  
			int i;  
			
			// 此例中 hex = "1de" 长度为3, hex是main函数传递的  
			len = strlen(hex);  
	
			for (i=0, temp=0; i<len; i++, temp=0)  
			{  
					// 第一次：i=0, *(hex + i) = *(hex + 0) = '1', 即temp = 1  
					// 第二次：i=1, *(hex + i) = *(hex + 1) = 'd', 即temp = 13  
					// 第三次：i=2, *(hex + i) = *(hex + 2) = 'd', 即temp = 14  
					temp = c2i( *(hex + i) );  
					// 总共3位，一个16进制位用 4 bit保存  
					// 第一次：'1'为最高位，所以temp左移 (len - i -1) * 4 = 2 * 4 = 8 位  
					// 第二次：'d'为次高位，所以temp左移 (len - i -1) * 4 = 1 * 4 = 4 位  
					// 第三次：'e'为最低位，所以temp左移 (len - i -1) * 4 = 0 * 4 = 0 位  
					bits = (len - i - 1) * 4;  
					temp = temp << bits;  
	
					// 此处也可以用 num += temp;进行累加  
					num = num | temp;  
			}  
	
			// 返回结果  
			return num;  
	}  

	double getDistance(double angle){
		angle = (180.0-43.99)/180.0*M_PI-angle;
		double height = 0.0325*sin(angle);
		double width_d = 0.0325*cos(angle);
		double width = sqrtf32((0.058*0.058)-(height-0.01456)*(height-0.01456)) + width_d;
		return width;
	}

    // 反向求解函数，输入 width，输出 angle，使用二分法
    double getAngle(double targetWidth, double tol = 1e-6, int maxIterations = 1000) {
        // 定义角度的搜索范围
        double left = 0.0;
        double right = M_PI; // 假设角度在0到90度之间

        for (int i = 0; i < maxIterations; ++i) {
            double mid = (left + right) / 2;
            double currentWidth = getDistance(mid);

            if (std::abs(currentWidth - targetWidth) < tol) {
                return mid;
            }

            if (currentWidth < targetWidth) {
                left = mid;
            } else {
                right = mid;
            }
        }
        
        // 如果没有找到精确解，返回最后的中间值
        return (left + right) / 2;
    }

	void receiving(){
		ros::Rate rate(100);
		while(ros::ok()) {
			rate.sleep();
			std::string data;
			{
				std::lock_guard<std::mutex> lock(serialMtx);
				if (serial && serial->is_open()) {
					try {
						std::vector<char> buffer(2048);
						boost::system::error_code ec;
						size_t bytes_read = serial->read_some(boost::asio::buffer(buffer), ec);
						
						if (!ec && bytes_read > 0) {
							data = std::string(buffer.begin(), buffer.begin() + bytes_read);
						}
					} catch (const boost::system::system_error& e) {
						continue;
					}
				}
			}
			
			if (data.size() > 0) {
				if(msg.size() > 1000)
					msg.clear();
				msg += data;
				int start = -1;
				int end = -1;
				// if(isGripper)
				// 	std::cout<<"msg:"<<data<<std::endl;
				while(find_json(msg, start, end)){
					// if(isGripper)
					// std::cout<<"------------------------------------find_json"<<std::endl;
					std::string str = msg.substr(start, end + 1 - start);
					if(end + 1 < msg.size())
						msg = msg.substr(end + 1);
					else
						msg.clear();
					ros::Time time = ros::Time().now();
					Json::Reader jsonReader;
					Json::Value root;
					try{
						jsonReader.parse(str, root);
						if(root.isMember("AS5047")){
							data_msgs::Gripper gripper;
							gripper.header.stamp = time;
							gripper.enable = true;
							Json::Value AS5047Value = root["AS5047"];
							if(!AS5047Value.isMember("error")){
								gripper.angle = AS5047Value["rad"].asDouble();
								gripper.error = false;
								if(gripper.angle < 0){
									gripper.error = true;
									gripper.angle = 0;
								}else if(gripper.angle > 1.67){
									gripper.error = true;
									gripper.angle = 1.67;
								}
								double dist1 = getDistance(gripper.angle);
								double dist0 = getDistance(0);
								gripper.distance = 2*(dist1 - dist0);//AS5047Value["distance"].asDouble();
								receiveDataMtx.lock();
								this->angle = gripper.angle;
								this->distance = gripper.distance;
								receiveDataMtx.unlock();
								pubGripper.publish(gripper);

								sensor_msgs::JointState jointState;
								jointState.header.stamp = time;
								jointState.name.resize(1);
								jointState.position.resize(1);
								jointState.name[0] = jointName;
								jointState.position[0] = gripper.distance;  // 0.77 - gripper.angle / 1.67 * (0.77 + 0.10);
								pubGripperJointState.publish(jointState);
							}
						}
						if(root.isMember("IMU")){
							sensor_msgs::Imu imu;
							imu.header.stamp = time;
							Json::Value IMUValue = root["IMU"];
							imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(IMUValue["roll"].asDouble(), IMUValue["pitch"].asDouble(), IMUValue["yaw"].asDouble());
							imu.angular_velocity.x = IMUValue["gyr"][0].asDouble();
							imu.angular_velocity.y = IMUValue["gyr"][1].asDouble();
							imu.angular_velocity.z = IMUValue["gyr"][2].asDouble();
							imu.linear_acceleration.x = IMUValue["acc"][0].asDouble();
							imu.linear_acceleration.y = IMUValue["acc"][1].asDouble();
							imu.linear_acceleration.z = IMUValue["acc"][2].asDouble();
							pubImu.publish(imu);
						}
						if(root.isMember("motor")){
							data_msgs::Gripper gripper;
							gripper.header.stamp = time;
							Json::Value motorValue = root["motor"];
							gripper.angle = motorValue["Position"].asDouble();
							gripper.error = false;
							if(gripper.angle < 0){
								gripper.error = true;
								gripper.angle = 0;
							}else if(gripper.angle > 1.67){
								gripper.error = true;
								gripper.angle = 1.67;
							}
							double dist1 = getDistance(gripper.angle);
							double dist0 = getDistance(0);
							gripper.distance = 2*(dist1 - dist0);//AS5047Value["distance"].asDouble();
							gripper.effort = motorValue["Current"].asDouble();
							gripper.velocity = motorValue["Speed"].asDouble();
							gripper.enable = enable;
							gripper.voltage = voltage;
							gripper.driver_temp = driver_temp;
							gripper.motor_temp = motor_temp;
							gripper.bus_current = bus_current;
							gripper.status = status;
							pubGripper.publish(gripper);
							receiveDataMtx.lock();
							this->angle = gripper.angle;
							this->distance = gripper.distance;
							this->motorCurrent = gripper.effort;
							receiveDataMtx.unlock();

							sensor_msgs::JointState jointState;
							jointState.header.stamp = time;
							jointState.name.resize(1);
							jointState.position.resize(1);
							jointState.name[0] = jointName;
							jointState.position[0] = gripper.distance;  // 0.77 - gripper.angle / 1.67 * (0.77 + 0.10);
							pubGripperJointState.publish(jointState);

							// if(fabs(gripper.effort) > motorCurrentLimit + motorCurrentRedundancy && motorCurrentLimit > 0){
							// 	float step = 0;
							// 	if(gripper.effort < 0)
							// 		step = 0.01;
							// 	if(gripper.effort > 0)
							// 		step = -0.01;
							// 	std::vector<uint8_t> command = createBinaryCommand<float>(mitMode?POSITION_CTRL_MIT:POSITION_CTRL_POS_VEL, std::vector<float>{gripper.angle+step});
							// 	std::lock_guard<std::mutex> lock(serialMtx);
							// 	if(!((step > 0 && lastCommandAngle > gripper.angle+step) || (step < 0 && lastCommandAngle < gripper.angle+step))){
							// 		if(serial && serial->is_open()){
							// 			boost::asio::write(*serial, boost::asio::buffer(command));
							// 			lastCommandAngle = angle;
							// 		}
							// 	}
							// }
						}
						if(root.isMember("motorstatus")){
							Json::Value motorstatusValue = root["motorstatus"];
							receiveDataMtx.lock();
							voltage = motorstatusValue["Voltage"].asDouble();
							driver_temp = motorstatusValue["DriverTemp"].asDouble();
							motor_temp = motorstatusValue["MotorTemp"].asDouble();
							bus_current = motorstatusValue["BusCurrent"].asDouble();
							status = motorstatusValue["Status"].asString();
							receiveDataMtx.unlock();
							if(motorstatusValue["Status"].asString().size() == 4){
								int status = hex2dec(motorstatusValue["Status"].asString().substr(2, 2).c_str());
								if(0b01000000 & status){
									receiveDataMtx.lock();
									this->enable = true;
									receiveDataMtx.unlock();
								}else{
									receiveDataMtx.lock();
									this->enable = false;
									receiveDataMtx.unlock();
								}
							}
						}
						if(root.isMember("Command")){
							if(command == -1)
								command = root["Command"].asInt();
							if(root["Command"].asInt() != command){
								{
									std::lock_guard<std::mutex> lock(colorStatusMtx);
									colorStatus[COLOR_BLUE] = true;
								}
								command = root["Command"].asInt();
								std_srvs::Trigger teleop_srv;
								client_arm_teleop.call(teleop_srv);
								data_msgs::CaptureService srv;
								srv.request.dataset_dir = "";
								srv.request.episode_index = -1;
								srv.request.instructions = "[null]";
								srv.request.start = true;
								srv.request.end = true;
								client.call(srv);
							}
						}
					}catch(Json::LogicError e){
						continue;
					}
					start = -1;
					end = -1;
				}
			}
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sensor_tools");
	RosOperator rosOperator;
	if(rosOperator.initSerial()){
		ROS_INFO_STREAM("serial started");
		ros::spin();
	}else{
		ROS_INFO_STREAM("serial error");
	}
	return 0;
}