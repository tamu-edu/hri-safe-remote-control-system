/*
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * ROS Includes
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int32.hpp"

/**
 * System Includes
 */
#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <sys/resource.h>

/**
 * Includes
 */
#include "VscProcess.h"
#include "JoystickHandler.h"
#include "VehicleInterface.h"
#include "VehicleMessages.h"

using namespace hri_safety_sense;

VscProcess::VscProcess() : Node("vsc_process_node"), myEStopState(0)
{
    std::string serialPort = this->declare_parameter("serial", "/dev/ttyACM0");
    RCLCPP_INFO(this->get_logger(), "Serial Port updated to:  %s", serialPort.c_str());

    int serialSpeed = this->declare_parameter("serial_speed", 115200);
    RCLCPP_INFO(this->get_logger(), "Serial Port Speed updated to:  %i", serialSpeed);

    // Open VSC Interface
    vscInterface = vsc_initialize(serialPort.c_str(), serialSpeed);
    if (vscInterface == NULL) {
        RCLCPP_FATAL(this->get_logger(), "Cannot open serial port! (%s, %i)", serialPort.c_str(), serialSpeed);
    } else {
        RCLCPP_INFO(this->get_logger(), "Connected to VSC on %s : %i", serialPort.c_str(), serialSpeed);
    }

    // Attempt to Set priority
    bool set_priority = this->declare_parameter("set_priority", false);
    RCLCPP_INFO(this->get_logger(), "Set priority updated to:  %i", set_priority);

    // EStop callback
	estopServ = this->create_service<srv::EmergencyStop>("safety/service/send_emergency_stop", 
		[this](const std::shared_ptr<srv::EmergencyStop::Request> request,
				std::shared_ptr<srv::EmergencyStop::Response> response) 
	{
        this->EmergencyStop(request, response);
    }
	);
	
    // KeyValue callbacks
    keyValueServ = this->create_service<srv::KeyValue>("safety/service/key_value", 
    [this](const std::shared_ptr<srv::KeyValue::Request> request,
           std::shared_ptr<srv::KeyValue::Response> response) 
    {
        this->KeyValue(request, response);
    }
	);

     keyStringServ = this->create_service<srv::KeyString>("safety/service/key_string", 
    [this](const std::shared_ptr<srv::KeyString::Request> request,
           std::shared_ptr<srv::KeyString::Response> response) 
    {
        this->KeyString(request, response);
    }
	);

    // Publish Emergency Stop Status
    estopPub = this->create_publisher<std_msgs::msg::UInt32>("safety/emergency_stop", 10);

    // Main Loop Timer Callback
    mainLoopTimer = this->create_wall_timer(std::chrono::milliseconds(1000 / VSC_INTERFACE_RATE), std::bind(&VscProcess::processOneLoop, this));

    // Init last time to now
    lastDataRx = this->now();

    // Clear all error counters
    memset(&errorCounts, 0, sizeof(errorCounts));
}

VscProcess::~VscProcess()
{
    // Destroy vscInterface
    vsc_cleanup(vscInterface);
    if(joystickHandler) delete joystickHandler;
    RCLCPP_INFO(this->get_logger(), "Interface Cleaned Up");
}

void VscProcess::init() {
    joystickHandler = new JoystickHandler(this->shared_from_this());
    RCLCPP_INFO(this->get_logger(), "Joystick Handler Initialized");
}

void VscProcess::EmergencyStop(const std::shared_ptr<srv::EmergencyStop::Request> req, std::shared_ptr<srv::EmergencyStop::Response> /*res*/)
{
    myEStopState = (uint32_t)req->emergency_stop;

    RCLCPP_WARN(this->get_logger(), "VscProcess::EmergencyStop: to 0x%x", myEStopState);

}

void VscProcess::KeyValue(const std::shared_ptr<srv::KeyValue::Request> req, std::shared_ptr<srv::KeyValue::Response> /*res*/)
{
    // Send heartbeat message to vehicle in every state
    vsc_send_user_feedback(vscInterface, req->key, req->value);

    RCLCPP_INFO(this->get_logger(), "VscProcess::KeyValue: 0x%x, 0x%x", req->key, req->value);
}

void VscProcess::KeyString(const std::shared_ptr<srv::KeyString::Request> req, std::shared_ptr<srv::KeyString::Response> /*res*/)
{
    // Send heartbeat message to vehicle in every state
    vsc_send_user_feedback_string(vscInterface, req->key, req->value.c_str());

    RCLCPP_INFO(this->get_logger(), "VscProcess::KeyValue: 0x%x, %s", req->key, req->value.c_str());
}

void VscProcess::processOneLoop()
{
	// Send heartbeat message to vehicle in every state
	vsc_send_heartbeat(vscInterface, myEStopState);

	// Check for new data from vehicle in every state
	readFromVehicle();
}

int VscProcess::handleHeartbeatMsg(VscMsgType& recvMsg)
{
    int retVal = 0;

    if(recvMsg.msg.length == sizeof(HeartbeatMsgType)) {
        RCLCPP_DEBUG(this->get_logger(), "Received Heartbeat from VSC");

        HeartbeatMsgType *msgPtr = (HeartbeatMsgType*)recvMsg.msg.data;

        // Publish E-STOP Values
        std_msgs::msg::UInt32 estopValue;
        estopValue.data = msgPtr->EStopStatus;
        estopPub->publish(estopValue);

        if(msgPtr->EStopStatus > 0) {
            RCLCPP_DEBUG(this->get_logger(), "Received ESTOP from the vehicle!!! 0x%x",msgPtr->EStopStatus);
        }

    } else {
        RCLCPP_WARN(this->get_logger(), "RECEIVED HEARTBEAT WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
                (unsigned int)sizeof(HeartbeatMsgType), recvMsg.msg.length);
        retVal = 1;
    }

    return retVal;
}

void VscProcess::readFromVehicle()
{
    VscMsgType recvMsg;

    /* Read all messages */
    while (vsc_read_next_msg(vscInterface, &recvMsg) > 0) {
        /* Read next Vsc Message */
        switch (recvMsg.msg.msgType) {
        case MSG_VSC_HEARTBEAT:
            if(handleHeartbeatMsg(recvMsg) == 0) {
                lastDataRx = this->now();
            }

            break;
        case MSG_VSC_JOYSTICK:
            if(joystickHandler->handleNewMsg(recvMsg) == 0) {
                lastDataRx = this->now();
            }

            break;

        case MSG_VSC_NMEA_STRING:
//			handleGpsMsg(&recvMsg);

            break;
        case MSG_USER_FEEDBACK:
//			handleFeedbackMsg(&recvMsg);

            break;
        default:
            errorCounts.invalidRxMsgCount++;
            RCLCPP_ERROR(this->get_logger(), "Receive Error.  Invalid MsgType (0x%02X)",recvMsg.msg.msgType);
            break;
        }
    }

    // Log warning when no data is received
    rclcpp::Duration noDataDuration = this->now() - lastDataRx;
	if(noDataDuration > rclcpp::Duration(0, 250000000)) { // 0.25 seconds = 0 seconds + 250000000 nanoseconds
		RCLCPP_WARN(this->get_logger(), "No Data Received in %f.%09li seconds", noDataDuration.seconds(), noDataDuration.nanoseconds());
	}
}

