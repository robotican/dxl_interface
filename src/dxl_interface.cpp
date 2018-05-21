/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Elhay Rauper*/


#include "dxl_interface/dxl_interface.h"

namespace dxl
{
    DxlInterface::DxlInterface()
    {

    }

    DxlInterface::PortState DxlInterface::openPort(std::string port_name,
                                                   unsigned int baudrate,
                                                   float protocol)
    {
        protocol_ = protocol;
        if (!loadProtocol(protocol_))
            return INVALID_PROTOCOL;
        port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());
        if (port_handler_->openPort())
        {
            if (port_handler_->setBaudRate(baudrate))
            {
                return SUCCESS;
            }
            return BAUDRATE_FAIL;
        }
        return PORT_FAIL;
    }

    bool DxlInterface::loadProtocol(uint16_t protocol)
    {
        if (protocol == DXL_PROTOCOL1)
        {
            pkt_handler_ = dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL1);
            return true;
        }
        if (protocol == DXL_PROTOCOL2)
        {
            pkt_handler_ = dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL2);
            return true;
        }
        return true;
    }

    /* ping motor, if success - motor model will be filled */
    bool DxlInterface::ping(dxl::Motor &motor)
    {
        int result = COMM_TX_FAIL;
        uint8_t error =  0;

        result = pkt_handler_->ping(port_handler_,
                                       motor.id,
                                       &(motor.spec.model),
                                       &error);

        if (result != COMM_SUCCESS || error != 0)
            return false;
        return true;
    }


    bool DxlInterface::setTorque(dxl::Motor &motor, bool flag)
    {
        uint8_t error = 0;
        int result = COMM_TX_FAIL;

        result = pkt_handler_->write1ByteTxRx(port_handler_,
                                                 motor.id,
                                                 motor.spec.torque_write_addr,
                                                 flag,
                                                 &error);

        if (result != COMM_SUCCESS || error != 0)
            return false;

        motor.in_torque = flag;
        return true;
    }

    bool DxlInterface::reboot(const Motor &motor)
    {
        // Dynamixel LED will flicker while it reboots
        uint8_t error = 0;
        int result = COMM_TX_FAIL;

        result = pkt_handler_->reboot(port_handler_, motor.id, &error);
        if (result != COMM_SUCCESS || error != 0)
            return false;
        return true;
    }


    bool DxlInterface::broadcastPing(std::vector<uint8_t> result_vec, uint16_t protocol)
    {
        int result = COMM_TX_FAIL;

        result = pkt_handler_->broadcastPing(port_handler_, result_vec);
        if (result != COMM_SUCCESS)
            return false;
        return true;
    }

    DxlInterface::~DxlInterface()
    {
        port_handler_->closePort();
    }

    bool DxlInterface::readMotorsPos(std::vector<dxl::Motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, pkt_handler_);
        for (Motor &motor : motors)
        {
            bool read_success = bulk_read.addParam(motor.id,
                                                   motor.spec.pos_read_addr,
                                                   motor.spec.len_present_pos);
            if (!read_success)
                return false;
        }

        int comm_result = bulk_read.txRxPacket();
        if (comm_result != COMM_SUCCESS)
            //packet_handler_->printTxRxResult(comm_result);
            return false;

        for (Motor &motor : motors)
        {
            bool getdata_result = bulk_read.isAvailable(motor.id,
                                                        motor.spec.pos_read_addr,
                                                        motor.spec.len_present_pos);
            if (!getdata_result)
                return false;

            uint32_t ticks = bulk_read.getData(motor.id,
                                               motor.spec.pos_read_addr,
                                               motor.spec.len_present_pos);
            motor.position =  math::ticks2rads(ticks, motor, protocol_) * motor.direction;

            // set max/min position flags
            if (motor.position >= motor.max_pos)
            {
                motor.max_pos_reached = true;
                motor.min_pos_reached = false;
            }
            else if (motor.position <= motor.min_pos)
            {
                motor.max_pos_reached = false;
                motor.min_pos_reached = true;
            }
            else
            {
                motor.max_pos_reached = false;
                motor.min_pos_reached = false;
            }


            if (motor.first_pos_read)
            {
                motor.first_pos_read = false;
                motor.command_position = motor.position;
            }
        }
        return true;
    }

    bool DxlInterface::readMotorsVel(std::vector<dxl::Motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, pkt_handler_);
        for (Motor &motor : motors)
        {
            bool read_success = bulk_read.addParam(motor.id,
                                                   motor.spec.vel_read_addr,
                                                   motor.spec.len_present_speed);
            if (!read_success)
                return false;
        }

        int comm_result = bulk_read.txRxPacket();
        if (comm_result != COMM_SUCCESS)
        {
            //packet_handler_->printTxRxResult(comm_result);
            return false;
        }

        for (Motor &motor : motors)
        {
            bool getdata_result = bulk_read.isAvailable(motor.id,
                                                        motor.spec.vel_read_addr,
                                                        motor.spec.len_present_speed);
            if (!getdata_result)
                return false;

            uint32_t ticks_per_sec = bulk_read.getData(motor.id,
                                                       motor.spec.vel_read_addr,
                                                       motor.spec.len_present_speed);
            motor.velocity = math::ticks_s2rad_s(ticks_per_sec, motor, protocol_);
        }

        return true;
    }

    bool DxlInterface::readMotorsError(std::vector<dxl::Motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, pkt_handler_);
        for (Motor &motor : motors)
        {
            bool read_success = bulk_read.addParam(motor.id,
                                                   motor.spec.error_read_addr,
                                                   DXL_ERR_LEN);
            if (!read_success)
                return false;
        }

        int comm_result = bulk_read.txRxPacket();
        if (comm_result != COMM_SUCCESS)
        {
            //packet_handler_->printTxRxResult(comm_result);
            return false;
        }

        for (Motor &motor : motors)
        {
            bool getdata_result = bulk_read.isAvailable(motor.id,
                                                        motor.spec.error_read_addr,
                                                        DXL_ERR_LEN);
            if (!getdata_result)
                return false;

            motor.error = bulk_read.getData(motor.id,
                                            motor.spec.error_read_addr,
                                            DXL_ERR_LEN);
        }
        return true;
    }

    bool DxlInterface::readMotorsLoad(std::vector<dxl::Motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, pkt_handler_);
        for (Motor &motor : motors)
        {
            bool read_success = bulk_read.addParam(motor.id,
                                                   motor.spec.current_read_addr,
                                                   motor.spec.len_present_curr);
            if (!read_success)
                return false;
        }

        int comm_result = bulk_read.txRxPacket();
        if (comm_result != COMM_SUCCESS)
        {
            //pkt_handler_->printTxRxResult(comm_result);
            return false;
        }

        for (Motor &motor : motors)
        {
            bool getdata_result = bulk_read.isAvailable(motor.id,
                                                        motor.spec.current_read_addr,
                                                        motor.spec.len_present_curr);
            if (!getdata_result)
                return false;


            motor.current = (int16_t)bulk_read.getData(motor.id,
                                                       motor.spec.current_read_addr,
                                                       motor.spec.len_present_curr);

            motor.current *= motor.spec.current_ratio;
        }
        return true;
    }

    bool DxlInterface::bulkWriteVelocity(std::vector<dxl::Motor> &motors)
    {
        dynamixel::GroupBulkWrite bulk_write(port_handler_, pkt_handler_);

        for (Motor &motor : motors)
        {
            bool addparam_success = false;

            // sturate max values
            if (motor.command_velocity > motor.max_vel)
                motor.command_velocity = motor.max_vel;
            else if (motor.command_velocity < -motor.max_vel)
                motor.command_velocity = -motor.max_vel;

            // sturate min values
            int8_t vel_sign = (motor.command_velocity < 0) ? -1 : 1;
            if (fabs(motor.command_velocity) < motor.min_vel)
                motor.command_velocity = motor.min_vel * vel_sign;

            // if vel_cmd will cause motor to go out of limits, set it to 0
            if (motor.max_pos_reached && motor.command_velocity > 0)
            {
                motor.command_velocity = 0;
            }
            else if (motor.min_pos_reached && motor.command_velocity < 0)
            {
                motor.command_velocity = 0;
            }

            int32_t motor_ticks_vel = math::rad_s2ticks_s(motor.command_velocity, motor, protocol_);

            // dxl api interperate 0 ticks velocity as the highest velocity.
            // dxl motor can be very dangerous to operate in high speeds.
            // following code will protect from sending 0 to motors
            // last protection layer - if 0 send 1 tick (slowest possible)
            // this protection will be applied only for position mode, because
            // for velocity mode 0 means stop instead of max velocity
            if (motor.interface_type == dxl::Motor::POSITION && motor_ticks_vel == 0)
                motor_ticks_vel = vel_sign;

            addparam_success = bulk_write.addParam(motor.id,
                                                   motor.spec.vel_write_addr,
                                                   motor.spec.len_goal_speed,
                                                   (uint8_t*)&motor_ticks_vel);

            if (!addparam_success)
                return false;
        }

        int8_t comm_result_ = bulk_write.txPacket();

        if (comm_result_ != COMM_SUCCESS)
            return false;

        bulk_write.clearParam();
        return true;
    }

    bool DxlInterface::bulkWritePosition(std::vector<dxl::Motor> &motors)
    {
        dynamixel::GroupBulkWrite bulk_write(port_handler_, pkt_handler_);

        for (Motor &motor : motors)
        {
            bool addparam_success = false;

            // sturation
            if (motor.command_position > motor.max_pos)
                motor.command_position = motor.max_pos;
            else if (motor.command_position < motor.min_pos)
                motor.command_position = motor.min_pos;


            int32_t motor_pos = math::rads2ticks(motor.command_position * motor.direction, motor, protocol_);
            addparam_success = bulk_write.addParam(motor.id,
                                                   motor.spec.pos_write_addr,
                                                   motor.spec.len_goal_pos,
                                                   (uint8_t*)&motor_pos);
            if (!addparam_success)
                return false;
        }

        int8_t comm_result_ = bulk_write.txPacket();

        if (comm_result_ != COMM_SUCCESS)
            return false;

        bulk_write.clearParam();
        return true;
    }

    bool DxlInterface::bulkWriteGoalTorque(std::vector<Motor> &motors)
    {
        dynamixel::GroupBulkWrite bulk_write(port_handler_, pkt_handler_);

        for (Motor &motor : motors)
        {
            bool addparam_success = false;

            printf("addr: %i, len: %i, command: %i\n", motor.spec.goal_torque_addr,
                   motor.spec.len_goal_torque,
                   motor.command_torque);

            addparam_success = bulk_write.addParam(motor.id,
                                                   motor.spec.goal_torque_addr,
                                                   motor.spec.len_goal_torque,
                                                   (uint8_t*)&motor.command_torque);
            if (!addparam_success)
                return false;
        }

        int8_t comm_result_ = bulk_write.txPacket();

        if (comm_result_ != COMM_SUCCESS)
            return false;

        bulk_write.clearParam();
        return true;
    }
}
