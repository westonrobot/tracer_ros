#include "tracer_sdk/tracer_base.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

namespace
{
// source: https://github.com/rxdu/stopwatch
struct StopWatch
{
    using Clock = std::chrono::high_resolution_clock;
    using time_point = typename Clock::time_point;
    using duration = typename Clock::duration;

    StopWatch() { tic_point = Clock::now(); };

    time_point tic_point;

    void tic()//start timing
    {
        tic_point = Clock::now();
    };

    double toc()//stop timing and return time
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count() / 1000000.0;
    };

    // for different precisions
    double stoc()
    {
        return std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - tic_point).count();
    };

    double mtoc()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();
    };

    double utoc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();
    };

    double ntoc()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - tic_point).count();
    };

    // you have to call tic() before calling this function
    void sleep_until_ms(int64_t period_ms)
    {
        int64_t duration = period_ms - std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(duration));
    };

    void sleep_until_us(int64_t period_us)
    {
        int64_t duration = period_us - std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(duration));
    };
};
} // namespace

namespace wescore
{
TracerBase::~TracerBase()
{
    if (serial_connected_)
        serial_if_->close();

    if (cmd_thread_.joinable())
        cmd_thread_.join();
}

void TracerBase::Connect(std::string dev_name, int32_t baud_rate)
{
    if (baud_rate == 0)
    {
        ConfigureCANBus(dev_name);
    }
    else
    {
        ConfigureSerial(dev_name, baud_rate);

        if (!serial_connected_)
            std::cerr << "ERROR: Failed to connect to serial port" << std::endl;
    }
}

void TracerBase::Disconnect()
{
    if (serial_connected_)
    {
        if (serial_if_->is_open())
            serial_if_->close();
    }
}

void TracerBase::ConfigureCANBus(const std::string &can_if_name)
{
    can_if_ = std::make_shared<ASyncCAN>(can_if_name);

    can_if_->set_receive_callback(std::bind(&TracerBase::ParseCANFrame, this, std::placeholders::_1));

    can_connected_ = true;
}

void TracerBase::ConfigureSerial(const std::string uart_name, int32_t baud_rate)
{
    serial_if_ = std::make_shared<ASyncSerial>(uart_name, baud_rate);
    serial_if_->open();

    if (serial_if_->is_open())
        serial_connected_ = true;

    serial_if_->set_receive_callback(std::bind(&TracerBase::ParseUARTBuffer, this,
                                               std::placeholders::_1,
                                               std::placeholders::_2,
                                               std::placeholders::_3));
}

void TracerBase::StartCmdThread()
{
    current_motion_cmd_.linear_velocity = 0;
    current_motion_cmd_.angular_velocity = 0;
    current_motion_cmd_.fault_clear_flag = TracerMotionCmd::FaultClearFlag::NO_FAULT;

    cmd_thread_ = std::thread(std::bind(&TracerBase::ControlLoop, this, cmd_thread_period_ms_));
    cmd_thread_started_ = true;
}

void TracerBase::SendMotionCmd(uint8_t count)
{
    // motion control message
    MotionControlMessage m_msg;

    if (can_connected_)
        m_msg.data.cmd.control_mode = CTRL_MODE_CMD_CAN;
    else if (serial_connected_)
        m_msg.data.cmd.control_mode = CTRL_MODE_CMD_UART;

    motion_cmd_mutex_.lock();
    m_msg.data.cmd.fault_clear_flag = static_cast<uint8_t>(current_motion_cmd_.fault_clear_flag);
    m_msg.data.cmd.linear_velocity_cmd = current_motion_cmd_.linear_velocity;
    m_msg.data.cmd.angular_velocity_cmd = current_motion_cmd_.angular_velocity;
    motion_cmd_mutex_.unlock();

    m_msg.data.cmd.reserved0 = 0;
    m_msg.data.cmd.reserved1 = 0;
    m_msg.data.cmd.count = count;

    if (can_connected_)
        m_msg.data.cmd.checksum = CalcTracerCANChecksum(CAN_MSG_MOTION_CONTROL_CMD_ID, m_msg.data.raw, 8);
    // serial_connected_: checksum will be calculated later when packed into a complete serial frame

    if (can_connected_)
    {
        // send to can bus
        can_frame m_frame;
        EncodeTracerMotionControlMsgToCAN(&m_msg, &m_frame);
        can_if_->send_frame(m_frame);
    }
    else
    {
        // send to serial port
        EncodeMotionControlMsgToUART(&m_msg, tx_buffer_, &tx_cmd_len_);
        serial_if_->send_bytes(tx_buffer_, tx_cmd_len_);
    }
}

void TracerBase::SendLightCmd(uint8_t count)
{
    LightControlMessage l_msg;

    light_cmd_mutex_.lock();
    if (light_ctrl_enabled_)
    {
        l_msg.data.cmd.light_ctrl_enable = LIGHT_ENABLE_CTRL;

        l_msg.data.cmd.front_light_mode = static_cast<uint8_t>(current_light_cmd_.front_mode);
        l_msg.data.cmd.front_light_custom = current_light_cmd_.front_custom_value;
        l_msg.data.cmd.rear_light_mode = static_cast<uint8_t>(current_light_cmd_.rear_mode);
        l_msg.data.cmd.rear_light_custom = current_light_cmd_.rear_custom_value;
    }
    else
    {
        l_msg.data.cmd.light_ctrl_enable = LIGHT_DISABLE_CTRL;

        l_msg.data.cmd.front_light_mode = LIGHT_MODE_CONST_OFF;
        l_msg.data.cmd.front_light_custom = 0;
        l_msg.data.cmd.rear_light_mode = LIGHT_MODE_CONST_OFF;
        l_msg.data.cmd.rear_light_custom = 0;
    }
    light_ctrl_requested_ = false;
    light_cmd_mutex_.unlock();

    l_msg.data.cmd.reserved0 = 0;
    l_msg.data.cmd.count = count;

    if (can_connected_)
        l_msg.data.cmd.checksum = CalcTracerCANChecksum(CAN_MSG_LIGHT_CONTROL_CMD_ID, l_msg.data.raw, 8);
    // serial_connected_: checksum will be calculated later when packed into a complete serial frame

    if (can_connected_)
    {
        // send to can bus
        can_frame l_frame;
        EncodeTracerLightControlMsgToCAN(&l_msg, &l_frame);

        can_if_->send_frame(l_frame);
    }
    else
    {
        // send to serial port
        EncodeLightControlMsgToUART(&l_msg, tx_buffer_, &tx_cmd_len_);
        serial_if_->send_bytes(tx_buffer_, tx_cmd_len_);
    }

    // std::cout << "cmd: " << static_cast<int>(l_msg.data.cmd.front_light_mode) << " , " << static_cast<int>(l_msg.data.cmd.front_light_custom) << " , "
    //           << static_cast<int>(l_msg.data.cmd.rear_light_mode) << " , " << static_cast<int>(l_msg.data.cmd.rear_light_custom) << std::endl;
    // std::cout << "can: ";
    // for (int i = 0; i < 8; ++i)
    //     std::cout << static_cast<int>(l_frame.data[i]) << " ";
    // std::cout << "uart: ";
    // for (int i = 0; i < tx_cmd_len_; ++i)
    //     std::cout << static_cast<int>(tx_buffer_[i]) << " ";
    // std::cout << std::endl;
}

void TracerBase::ControlLoop(int32_t period_ms)
{
    StopWatch ctrl_sw;
    uint8_t cmd_count = 0;
    uint8_t light_cmd_count = 0;
    while (true)
    {
        ctrl_sw.tic();

        // motion control message
        SendMotionCmd(cmd_count++);

        // check if there is request for light control
        if (light_ctrl_requested_)
            SendLightCmd(light_cmd_count++);

        ctrl_sw.sleep_until_ms(period_ms);
        // std::cout << "control loop update frequency: " << 1.0 / ctrl_sw.toc() << std::endl;
    }
}

TracerState TracerBase::GetTracerState()
{
    std::lock_guard<std::mutex> guard(tracer_state_mutex_);
    return tracer_state_;
}

void TracerBase::SetMotionCommand(double linear_vel, double angular_vel, TracerMotionCmd::FaultClearFlag fault_clr_flag)
{
    // make sure cmd thread is started before attempting to send commands
    if (!cmd_thread_started_)
        StartCmdThread();

    if (linear_vel < TracerMotionCmd::min_linear_velocity)
        linear_vel = TracerMotionCmd::min_linear_velocity;
    if (linear_vel > TracerMotionCmd::max_linear_velocity)
        linear_vel = TracerMotionCmd::max_linear_velocity;
    if (angular_vel < TracerMotionCmd::min_angular_velocity)
        angular_vel = TracerMotionCmd::min_angular_velocity;
    if (angular_vel > TracerMotionCmd::max_angular_velocity)
        angular_vel = TracerMotionCmd::max_angular_velocity;

    std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
    current_motion_cmd_.linear_velocity = static_cast<int8_t>(linear_vel / TracerMotionCmd::max_linear_velocity * 100.0);
    current_motion_cmd_.angular_velocity = static_cast<int8_t>(angular_vel / TracerMotionCmd::max_angular_velocity * 100.0);
    current_motion_cmd_.fault_clear_flag = fault_clr_flag;
}

void TracerBase::SetLightCommand(TracerLightCmd cmd)
{
    if (!cmd_thread_started_)
        StartCmdThread();

    std::lock_guard<std::mutex> guard(light_cmd_mutex_);
    current_light_cmd_ = cmd;
    light_ctrl_enabled_ = true;
    light_ctrl_requested_ = true;
}

void TracerBase::DisableLightCmdControl()
{
    std::lock_guard<std::mutex> guard(light_cmd_mutex_);
    light_ctrl_enabled_ = false;
    light_ctrl_requested_ = true;
}

void TracerBase::ParseCANFrame(can_frame *rx_frame)
{
    // validate checksum, discard frame if fails
    if (!rx_frame->data[7] == CalcTracerCANChecksum(rx_frame->can_id, rx_frame->data, rx_frame->can_dlc))
    {
        std::cerr << "ERROR: checksum mismatch, discard frame with id " << rx_frame->can_id << std::endl;
        return;
    }

    // otherwise, update robot state with new frame
    TracerStatusMessage status_msg;
    DecodeTracerStatusMsgFromCAN(rx_frame, &status_msg);
    NewStatusMsgReceivedCallback(status_msg);
}

void TracerBase::ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    // std::cout << "bytes received from serial: " << bytes_received << std::endl;
    TracerStatusMessage status_msg;
    for (int i = 0; i < bytes_received; ++i)
    {
        if (DecodeTracerStatusMsgFromUART(buf[i], &status_msg))
            NewStatusMsgReceivedCallback(status_msg);
    }
}

void TracerBase::NewStatusMsgReceivedCallback(const TracerStatusMessage &msg)
{
    // std::cout << "new status msg received" << std::endl;
    std::lock_guard<std::mutex> guard(tracer_state_mutex_);
    UpdateTracerState(msg, tracer_state_);
}

void TracerBase::UpdateTracerState(const TracerStatusMessage &status_msg, TracerState &state)
{
    switch (status_msg.msg_type)
    {
    case TracerMotionStatusMsg:
    {
        // std::cout << "motion control feedback received" << std::endl;
        const MotionStatusMessage &msg = status_msg.motion_status_msg;
        state.linear_velocity = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.linear_velocity.low_byte) | static_cast<uint16_t>(msg.data.status.linear_velocity.high_byte) << 8) / 1000.0;
        state.angular_velocity = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.angular_velocity.low_byte) | static_cast<uint16_t>(msg.data.status.angular_velocity.high_byte) << 8) / 1000.0;
        break;
    }
    case TracerLightStatusMsg:
    {
        // std::cout << "light control feedback received" << std::endl;
        const LightStatusMessage &msg = status_msg.light_status_msg;
        if (msg.data.status.light_ctrl_enable == LIGHT_DISABLE_CTRL)
            state.light_control_enabled = false;
        else
            state.light_control_enabled = true;
        state.front_light_state.mode = msg.data.status.front_light_mode;
        state.front_light_state.custom_value = msg.data.status.front_light_custom;
        state.rear_light_state.mode = msg.data.status.rear_light_mode;
        state.rear_light_state.custom_value = msg.data.status.rear_light_custom;
        break;
    }
    case TracerSystemStatusMsg:
    {
        // std::cout << "system status feedback received" << std::endl;
        const SystemStatusMessage &msg = status_msg.system_status_msg;
        state.control_mode = msg.data.status.control_mode;
        state.base_state = msg.data.status.base_state;
        state.battery_voltage = (static_cast<uint16_t>(msg.data.status.battery_voltage.low_byte) | static_cast<uint16_t>(msg.data.status.battery_voltage.high_byte) << 8) / 10.0;
        state.fault_code = (static_cast<uint16_t>(msg.data.status.fault_code.low_byte) | static_cast<uint16_t>(msg.data.status.fault_code.high_byte) << 8);
        break;
    }
    case TracerMotorDriverStatusMsg:
    {
        // std::cout << "motor 1 driver feedback received" << std::endl;
        const MotorDriverStatusMessage &msg = status_msg.motor_driver_status_msg;
        for (int i = 0; i < 4; ++i)
        {
            state.motor_states[status_msg.motor_driver_status_msg.motor_id].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
            state.motor_states[status_msg.motor_driver_status_msg.motor_id].rpm = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
            state.motor_states[status_msg.motor_driver_status_msg.motor_id].temperature = msg.data.status.temperature;
        }
        break;
    }
    }
}
} // namespace wescore
