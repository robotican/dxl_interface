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

#ifndef DXL_INTERFACE_SPEC_H
#define DXL_INTERFACE_SPEC_H

#include <cstdint>
#include <string>

namespace dxl
{
    struct spec
    {
        std::string name;
        uint16_t model = 0;
        float torque_const_a = 0;
        float torque_const_b = 0;
        int cpr = 0;
        double rpm_scale_factor = 0;
        double current_ratio = 0;

        uint16_t pos_read_addr = 0;
        uint16_t vel_read_addr = 0;
        uint16_t current_read_addr = 0;
        uint16_t error_read_addr = 0;

        uint16_t torque_write_addr = 0;
        uint16_t vel_write_addr = 0;
        uint16_t pos_write_addr = 0;

        uint16_t len_present_speed = 0;
        uint16_t len_present_pos = 0;
        uint16_t len_present_curr = 0;
        uint16_t len_goal_speed = 0;
        uint16_t len_goal_pos = 0;
    };
}


#endif //DXL_INTERFACE_SPEC_H
