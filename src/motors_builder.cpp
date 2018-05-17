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

#include <dxl_interface/motors_builder.h>

namespace dxl
{
    MotorsBuilder::MotorsBuilder(ros::NodeHandle &nh)
    {
        nh_ = &nh;
        ros::param::get("~load_dxl_hw", load_dxl_hw_);
        if (load_dxl_hw_)
        {
            /* the order of calling the following methods is important      */
            /* because some calls load params / objects for following calls */
            fetchParams();
            buildMotors();
            openPort();
            pingMotors();
            loadSpecs();
            setTorque(true);

            for (Motor &motor :  motors_)
            {
                ROS_INFO ("[armadillo2_hw/dxl_motors_builder]: done building motor id: %d, model: %d",
                          motor.id, motor.spec.model);
            }

            torque_srv_ = nh_->advertiseService("hardware/dxl_torque", &MotorsBuilder::torqueServiceCB, this);
            ROS_INFO("[dxl_motor_builder]: dxl motors are up");
        }
        else
            ROS_WARN("[dxl_motors_builder]: dxl motors hardware is disabled");
    }

    bool MotorsBuilder::getMotor(int motor_id, Motor &motor)
    {
        for (const Motor m : motors_)
        {
            if (m.id == motor_id)
            {
                motor = m;
                return true;
            }
        }
        return false;
    }

    bool MotorsBuilder::setMotorTorque(int motor_id, bool flag)
    {
        comm_mutex_.lock();

        for (Motor& m : motors_)
        {
            if (m.id == motor_id)
            {
                bool success = dxl_interface_.setTorque(m, flag);
                // give motor some time to execute
                ros::Duration(0.1).sleep();
                comm_mutex_.unlock();
                return success;
            }
        }

        comm_mutex_.unlock();
        return false;
    }

    bool MotorsBuilder::setMotorPosition(int motor_id, double position)
    {
        for (Motor& m : motors_)
        {
            if (m.id == motor_id)
            {
                m.command_position = position;
                return true;
            }
        }
        return false;
    }

    bool MotorsBuilder::setMotorVelocity(int motor_id, double velocity)
    {
        for (Motor& m : motors_)
        {
            if (m.id == motor_id)
            {
                m.command_velocity = velocity;
                return true;
            }
        }
        return false;
    }

    bool MotorsBuilder::rebootMotor(int motor_id)
    {
        comm_mutex_.lock();

        for (Motor& m : motors_)
        {
            if (m.id == motor_id)
            {
                bool success = dxl_interface_.reboot(m);
                // give motor some time to recover
                ros::Duration(0.1).sleep();
                comm_mutex_.unlock();
                return success;
            }
        }

        comm_mutex_.unlock();

        return false;
    }

    void MotorsBuilder::read()
    {
        if (!load_dxl_hw_)
            return;

        comm_mutex_.lock();

        //printf("read\n");

        if (!dxl_interface_.readMotorsPos(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: reading motors position failed");
            //throw exceptions::ReadPosException("reading motors position failed");
        }

        if (!dxl_interface_.readMotorsVel(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: reading motors velocity failed");
            //throw exceptions::ReadVelException("reading motors velocity failed");
        }

        if (!dxl_interface_.readMotorsLoad(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: reading motors load failed");
           //throw exceptions::ReadLoadException("reading motors load failed");
        }

        if (!dxl_interface_.readMotorsError(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: reading motors errors failed");
            //throw exceptions::ReadErrorException("reading motors load failed");
        }

        comm_mutex_.unlock();
    }

    void MotorsBuilder::writeToMotor(int motor_id, double position, double velocity)
    {
        if (!load_dxl_hw_)
            return;
        for (Motor motor : motors_)
        {
            if (motor.id == motor_id)
            {
                std::vector<Motor> single_motor_vec;
                motor.command_position = position;
                motor.command_velocity = velocity;
                single_motor_vec.push_back(motor);
                write(single_motor_vec);
            }
        }
    }

    void MotorsBuilder::write(std::vector<Motor> &motors)
    {
        if (!load_dxl_hw_)
            return;

        comm_mutex_.lock();

        //printf("write\n");

        if (!dxl_interface_.bulkWriteVelocity(motors))
        {
            //ROS_ERROR("[dxl_motors_builder]: writing velocity failed");
            //throw exceptions::WriteVelException("writing velocity failed");
        }

        if (!dxl_interface_.bulkWritePosition(motors))
        {
            //ROS_ERROR("[dxl_motors_builder]: writing postision failed");
           //throw exceptions::WritePosException("writing position failed");
        }

        comm_mutex_.unlock();
    }

    void MotorsBuilder::write()
    {
        write(motors_);
    }

    void MotorsBuilder::pingMotors()
    {
        if (!load_dxl_hw_)
            return;

        ros::Duration(1).sleep();

        comm_mutex_.lock();
        for (Motor &motor : motors_)
        {
            int error_counter = 0;
            while (!dxl_interface_.ping(motor))
            {
                error_counter++;
                ROS_WARN("[dxl_motors_builder]: pinging motor id: %d failed", motor.id);
                if (error_counter > MAX_PING_ERRORS)
                {
                    ROS_ERROR("[dxl_motors_builder]: too many ping errors, motor %d is not responding. \n"
                                      "check if motor crashed (red blink) and try to restart. \n"
                                      "also make sure LATENCY_TIMER is set to 1 in dynamixel_sdk, and that the appropriate"
                                      "rule for dynamixel port is existed with LATENCY_TIMER=1.", motor.id);
                    ros::shutdown();
                    exit(EXIT_FAILURE);
                }
                ros::Rate(5).sleep();
            }
        }
        comm_mutex_.unlock();
    }

    void MotorsBuilder::loadSpecs()
    {
        if (!load_dxl_hw_)
            return;
        /* build motors */
        for(int i = 0; i < dxl_spec_config_.size(); i++)
        {
            /* feed motor with user defined settings */
            if(dxl_spec_config_[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("[dxl_motors_builder]: motor spec at index %d param data type is invalid or missing. "
                                  "make sure that this param exist in dxl_spec_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }

            struct dxl::spec spec;

            /* name */
            if(dxl_spec_config_[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("[dxl_motors_builder]: spec name at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.name = static_cast<std::string>(dxl_spec_config_[i]["name"]);

            /* model */
            if(dxl_spec_config_[i]["model"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec model at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.model = static_cast<int>(dxl_spec_config_[i]["model"]);

            /* cpr */
            if(dxl_spec_config_[i]["cpr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec cpr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.cpr = static_cast<int>(dxl_spec_config_[i]["cpr"]);

            /* rpm_factor */
            if(dxl_spec_config_[i]["rpm_factor"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: spec rpm_factor at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.rpm_scale_factor = static_cast<double>(dxl_spec_config_[i]["rpm_factor"]);

            /* torque_const_a */
            if(dxl_spec_config_[i]["torque_const_a"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: spec torque_const_a at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.torque_const_a = static_cast<double>(dxl_spec_config_[i]["torque_const_a"]);

            /* torque_const_b */
            if(dxl_spec_config_[i]["torque_const_b"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: spec torque_const_b at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.torque_const_b = static_cast<double>(dxl_spec_config_[i]["torque_const_b"]);



            /* pos_read_addr */
            if(dxl_spec_config_[i]["pos_read_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec pos_read_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.pos_read_addr = static_cast<int>(dxl_spec_config_[i]["pos_read_addr"]);

            /* vel_read_addr */
            if(dxl_spec_config_[i]["vel_read_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec vel_read_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.vel_read_addr = static_cast<int>(dxl_spec_config_[i]["vel_read_addr"]);


            /* current_read_addr */
            if(dxl_spec_config_[i]["current_read_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec current_read_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.current_read_addr = static_cast<int>(dxl_spec_config_[i]["current_read_addr"]);

            /* error_read_addr */
            if(dxl_spec_config_[i]["error_read_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec error_read_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.error_read_addr = static_cast<int>(dxl_spec_config_[i]["error_read_addr"]);

            /* torque_write_addr */
            if(dxl_spec_config_[i]["torque_write_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec torque_write_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.torque_write_addr = static_cast<int>(dxl_spec_config_[i]["torque_write_addr"]);

            /* vel_write_addr */
            if(dxl_spec_config_[i]["vel_write_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec vel_write_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.vel_write_addr = static_cast<int>(dxl_spec_config_[i]["vel_write_addr"]);


            /* pos_write_addr */
            if(dxl_spec_config_[i]["pos_write_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec pos_write_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.pos_write_addr = static_cast<int>(dxl_spec_config_[i]["pos_write_addr"]);

            /* current_ratio */
            if(dxl_spec_config_[i]["current_ratio"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: spec current_ratio at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.current_ratio = static_cast<double>(dxl_spec_config_[i]["current_ratio"]);

            /* len_present_speed */
            if(dxl_spec_config_[i]["len_present_speed"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_present_speed at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_present_speed = static_cast<int>(dxl_spec_config_[i]["len_present_speed"]);

            /* len_present_pos */
            if(dxl_spec_config_[i]["len_present_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_present_pos at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_present_pos = static_cast<int>(dxl_spec_config_[i]["len_present_pos"]);

            /* len_present_curr */
            if(dxl_spec_config_[i]["len_present_curr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_present_curr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_present_curr = static_cast<int>(dxl_spec_config_[i]["len_present_curr"]);

            /* len_goal_speed */
            if(dxl_spec_config_[i]["len_goal_speed"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_goal_speed at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_goal_speed = static_cast<int>(dxl_spec_config_[i]["len_goal_speed"]);

            /* len_goal_pos */
            if(dxl_spec_config_[i]["len_goal_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_goal_pos at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_goal_pos = static_cast<int>(dxl_spec_config_[i]["len_goal_pos"]);

            specs_[spec.model] = spec;
        }

        /* feed motors with model specs */
        for (Motor &motor : motors_)
        {
            bool spec_found = specs_.find(motor.spec.model) != specs_.end();
            if (spec_found)
            {
                motor.spec.name = specs_[motor.spec.model].name;
                motor.spec.cpr = specs_[motor.spec.model].cpr;
                motor.spec.rpm_scale_factor = specs_[motor.spec.model].rpm_scale_factor;
                motor.spec.torque_const_a = specs_[motor.spec.model].torque_const_a;
                motor.spec.torque_const_b = specs_[motor.spec.model].torque_const_b;

                /* read addrs */
                motor.spec.pos_read_addr = specs_[motor.spec.model].pos_read_addr;
                motor.spec.vel_read_addr = specs_[motor.spec.model].vel_read_addr;
                motor.spec.current_read_addr = specs_[motor.spec.model].current_read_addr;
                motor.spec.error_read_addr = specs_[motor.spec.model].error_read_addr;

                /* write addrs */
                motor.spec.torque_write_addr = specs_[motor.spec.model].torque_write_addr;
                motor.spec.vel_write_addr = specs_[motor.spec.model].vel_write_addr;
                motor.spec.pos_write_addr = specs_[motor.spec.model].pos_write_addr;
                motor.spec.len_present_speed = specs_[motor.spec.model].len_present_speed;
                motor.spec.len_present_pos = specs_[motor.spec.model].len_present_pos;
                motor.spec.len_present_curr = specs_[motor.spec.model].len_present_curr;
                motor.spec.len_goal_speed = specs_[motor.spec.model].len_goal_speed;
                motor.spec.len_goal_pos = specs_[motor.spec.model].len_goal_pos;

                motor.spec.current_ratio = specs_[motor.spec.model].current_ratio;
            }
            else
            {
                ROS_ERROR("[dxl_motors_builder]: couldn't locate model specification for motor %d, model %d. "
                                  "make sure dxl_motor_data.yaml contains all the necessary specs. shutting down...", motor.id, motor.spec.model);
                ros::shutdown();
                exit(EXIT_FAILURE);
            }
        }
    }

    bool MotorsBuilder::setTorque(bool flag)
    {
        if (!load_dxl_hw_)
            return false;
        comm_mutex_.lock();
        std::string str_flag = flag ? "on" : "off";
        bool success = true;
        for (Motor &motor : motors_)
        {
            if (!dxl_interface_.setTorque(motor, flag))
            {
                success = false;
                ROS_WARN("[dxl_motors_builder]: failed set torque of motor id %d to %s", motor.id, str_flag.c_str());
            }
        }
        comm_mutex_.unlock();
        if (success)
        {
            ROS_INFO("[dxl_motors_builder]: motors torque is %s", str_flag.c_str());
            return true;
        }
        return false;
    }

    bool MotorsBuilder::torqueServiceCB(std_srvs::SetBool::Request  &req,
                                           std_srvs::SetBool::Response &res)
    {
        if (!load_dxl_hw_)
            return false;
        res.success = false;
        if (MotorsBuilder::setTorque(req.data))
            res.success = true;
        return true;
    }


    void MotorsBuilder::fetchParams()
    {
        if (!load_dxl_hw_)
            return;
        /* DXL_JOINTS_CONFIG_PARAM */
        if (!nh_->hasParam(DXL_JOINTS_CONFIG_PARAM))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", DXL_JOINTS_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(DXL_JOINTS_CONFIG_PARAM, dxl_joints_config_);
        if (dxl_joints_config_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is invalid (need to be an of type array) or missing. "
                              "make sure that this param exist in dxl_joints_config.yaml and that your launch "
                              "includes this param file. shutting down...", DXL_JOINTS_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        /* SPEC_CONFIG_PARAM */
        if (!nh_->hasParam(SPEC_CONFIG_PARAM))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", SPEC_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(SPEC_CONFIG_PARAM, dxl_spec_config_);
        if (dxl_spec_config_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is invalid (need to be an of type array) or missing. "
                              "make sure that this param exist in dxl_joints_config.yaml and that your launch "
                              "includes this param file. shutting down...", SPEC_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        /* DXL_PROTOCOL_PARAM */
        if (!ros::param::get(DXL_PROTOCOL_PARAM, protocol_))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", DXL_PROTOCOL_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        /* DXL_PORT_PARAM */
        if (!ros::param::get(DXL_PORT_PARAM, dxl_port_))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", DXL_PORT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        /* DXL_PORT_BAUD_PARAM */
        if (!ros::param::get(DXL_PORT_BAUD_PARAM, dxl_baudrate_))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", DXL_PORT_BAUD_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
    }

    void MotorsBuilder::buildMotors()
    {
        if (!load_dxl_hw_)
            return;
        /* build motors */
        for(int i = 0; i < dxl_joints_config_.size(); i++)
        {
            /* feed motor with user defined settings */
            if(dxl_joints_config_[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("[dxl_motors_builder]: dxl motor id at index %d param data type is invalid or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }

            struct Motor new_motor;

            /* defaults to prevent bad movement on startup */
            new_motor.command_position = 0.0;
            new_motor.command_velocity = 0.3;
            new_motor.min_vel = 0;
            new_motor.first_pos_read = true;

            // id
            if(dxl_joints_config_[i]["id"].getType() != XmlRpc::XmlRpcValue::TypeInt) //invalid id field
            {
                ROS_ERROR("[dxl_motors_builder]: dxl motor id at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.id = static_cast<int>(dxl_joints_config_[i]["id"]);

            // joint_name
            if (dxl_joints_config_[i]["joint_name"].getType() != XmlRpc::XmlRpcValue::TypeString) //invalid joint_name field
            {
                ROS_ERROR("[dxl_motors_builder]: dxl motor joint_name at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.joint_name = static_cast<std::string>(dxl_joints_config_[i]["joint_name"]);

            // interface
            if (dxl_joints_config_[i]["interface"].getType() != XmlRpc::XmlRpcValue::TypeString) //invalid interface field
            {
                ROS_ERROR("[dxl_motors_builder]: dxl motor interface_type at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            std::string string_interface_type = static_cast<std::string>(dxl_joints_config_[i]["interface"]);
            new_motor.interface_type = Motor::stringToInterfaceType(string_interface_type);

            // direction
            if(dxl_joints_config_[i]["direction"].getType() != XmlRpc::XmlRpcValue::TypeInt) //invalid direction field
            {
                ROS_ERROR("[dxl_motors_builder]: dxl motor direction at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.direction = static_cast<int>(dxl_joints_config_[i]["direction"]);

            // min vel
            if(dxl_joints_config_[i]["min_vel"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: dxl motor min_vel at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.min_vel = static_cast<double>(dxl_joints_config_[i]["min_vel"]);

            // max vel
            if(dxl_joints_config_[i]["max_vel"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: dxl motor max_vel at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.max_vel = static_cast<double>(dxl_joints_config_[i]["max_vel"]);

            // min pos
            if(dxl_joints_config_[i]["min_pos"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: dxl motor min_pos at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.min_pos = static_cast<double>(dxl_joints_config_[i]["min_pos"]);

            // max pos
            if(dxl_joints_config_[i]["max_pos"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: dxl motor max_pos at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.max_pos = static_cast<double>(dxl_joints_config_[i]["max_pos"]);

            motors_.push_back(new_motor);
        }
    }

    void MotorsBuilder::registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                                           hardware_interface::PositionJointInterface &position_interface,
                                           hardware_interface::PosVelJointInterface &posvel_interface)
    {
        if (!load_dxl_hw_)
            return;
        for(Motor &motor : motors_)
        {
            /* joint state registration */

            joint_state_handles_.push_back(hardware_interface::JointStateHandle (motor.joint_name,
                                                                                 &motor.position,
                                                                                 &motor.velocity,
                                                                                 &motor.current));
            joint_state_interface.registerHandle(joint_state_handles_.back());

            /* joint command registration */
            switch (motor.interface_type)
            {
                case Motor::POSITION:

                    pos_handles_.push_back(hardware_interface::JointHandle (joint_state_interface.getHandle(motor.joint_name),
                                                                            &motor.command_position));
                    position_interface.registerHandle(pos_handles_.back());
                    break;

                case Motor::VELOCITY:
                    // TODO: implement velocity handle registration
                    break;

                case Motor::POS_VEL:
                    posvel_handles_.push_back(hardware_interface::PosVelJointHandle (joint_state_interface.getHandle(motor.joint_name),
                                                                                     &motor.command_position,
                                                                                     &motor.command_velocity));
                    posvel_interface.registerHandle(posvel_handles_.back());
                    break;
            }
        }
    }

    void MotorsBuilder::openPort()
    {
        if (!load_dxl_hw_)
            return;
        dxl::DxlInterface::PortState port_state = dxl_interface_.openPort(dxl_port_,
                                                                          dxl_baudrate_,
                                                                          protocol_);
        switch (port_state)
        {
            case dxl::DxlInterface::PORT_FAIL:
                ROS_ERROR("[dxl_motors_builder]: open dxl port %s failed. "
                                  "make sure cable is connected and port has the right permissions. shutting down...", dxl_port_.c_str());
                ros::shutdown();
                exit (EXIT_FAILURE);
            case dxl::DxlInterface::BAUDRATE_FAIL:
                ROS_ERROR("[dxl_motors_builder]: setting dxl baudrate to %d failed. shutting down...", dxl_baudrate_);
                ros::shutdown();
                exit (EXIT_FAILURE);
            case dxl::DxlInterface::INVALID_PROTOCOL:
                ROS_ERROR("[dxl_motors_builder]: protocol version %f is invalid. shutting down...", protocol_);
                ros::shutdown();
                exit (EXIT_FAILURE);
            case dxl::DxlInterface::SUCCESS:
                ROS_INFO("[dxl_motors_builder]: dxl port opened successfully \nport name: %s \nbaudrate: %d", dxl_port_.c_str(), dxl_baudrate_);
        }
    }
}
