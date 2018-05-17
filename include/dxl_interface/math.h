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

#ifndef DXL_INTERFACE_CONVERSIONS_H
#define DXL_INTERFACE_CONVERSIONS_H

#include <cstdint>
#include <dxl_interface/model.h>
#include <math.h>
#include <dxl_interface/protocol.h>
#include <dxl_interface/motor.h>

namespace dxl
{
    namespace math
    {
        static double ticks2rads(int32_t ticks, struct Motor &motor, float protocol)
        {
            if (protocol == DXL_PROTOCOL2)
            {
                switch (motor.spec.model)
                {
                    case (uint16_t)Model::XH430_V350_R :
                    {
                        const double from_ticks = 1.0 / (motor.spec.cpr / 2.0);
                        return static_cast<double>(M_PI-(ticks) * from_ticks * M_PI);
                    }
                    case (uint16_t)Model::MX28 :
                    {
                        double cprDev2 = motor.spec.cpr / 2.0f;
                        return (static_cast<double>(ticks) - cprDev2) * M_PI / cprDev2;
                    }
                    case (uint16_t)Model::H42_20_S300_R :
                    {
                        const double from_ticks = 1.0 / (motor.spec.cpr / 2.0);
                        return static_cast<double>((ticks) * from_ticks * M_PI);
                    }
                    case (uint16_t)Model::X54_100_S500_R :
                    {
                        const double from_ticks = 1.0 / (motor.spec.cpr / 2.0);
                        return static_cast<double>((ticks) * from_ticks * M_PI);
                    }
                    case (uint16_t)Model::X54_200_S500_R :
                    {
                        const double from_ticks = 1.0 / (motor.spec.cpr / 2.0);
                        return static_cast<double>((ticks) * from_ticks * M_PI);
                    }
                    case (uint16_t)Model::RH_P12_RN:
                    {
                        // this case return meters for this motor
                        return static_cast<double>((0.109 / -740) * (ticks - 740));
                    }
                }
            }
            else if (protocol == DXL_PROTOCOL1)
            {
                double cprDev2 = motor.spec.cpr / 2.0f;
                return (static_cast<double>(ticks) - cprDev2) * M_PI / cprDev2;
            }
            return 0;
        }

        static int32_t rads2ticks(double rads, struct Motor &motor, float protocol)
        {

            if (protocol == DXL_PROTOCOL2)
            {
                switch (motor.spec.model)
                {
                    case (uint16_t)Model::XH430_V350_R :
                    {
                        return static_cast<int32_t>(round((-rads *180.0/ M_PI+180.0)/ 0.088));
                    }
                    case (uint16_t)Model::MX28 :
                    {
                        double half_cpr = motor.spec.cpr / 2.0f;
                        return static_cast<int32_t>(round(half_cpr + (rads * half_cpr / M_PI)));
                    }
                    case (uint16_t)Model::H42_20_S300_R :
                    {
                        double half_cpr = motor.spec.cpr / 2.0f;
                        return static_cast<int32_t>(round((rads / M_PI) * half_cpr));
                    }
                    case (uint16_t)Model::X54_100_S500_R :
                    {
                        double half_cpr = motor.spec.cpr / 2.0f;
                        return static_cast<int32_t>(round((rads / M_PI) * half_cpr));
                    }
                    case (uint16_t)Model::X54_200_S500_R :
                    {
                        double half_cpr = motor.spec.cpr / 2.0f;
                        return static_cast<int32_t>(round((rads / M_PI) * half_cpr));
                    }
                    case (uint16_t)Model::RH_P12_RN:
                    {
                        // this case return ticks from meters
                        return static_cast<double>((-740 * rads / 0.109) + 740);
                    }
                }
            }
            else if (protocol == DXL_PROTOCOL1)
            {
                double half_cpr = motor.spec.cpr / 2.0f;
                return static_cast<int32_t>(round(half_cpr + (rads * half_cpr / M_PI)));
            }
            return 0;
        }

        /* rads per sec to ticks per sec */
        static int32_t rad_s2ticks_s(double rads, struct Motor &motor, float protocol)
        {
            if (protocol == DXL_PROTOCOL2)
                return static_cast<int32_t >(rads / 2.0 / M_PI * 60.0 / motor.spec.rpm_scale_factor);
            else
                return static_cast<int32_t >(83.49f * (rads)-0.564f);
        }

        /* ticks per sec to rads per sec */
        static double ticks_s2rad_s(int32_t ticks, struct Motor &motor, float protocol)
        {
            if (protocol == DXL_PROTOCOL2)
                return ((double)ticks) * 2.0 * M_PI / 60.0 *motor.spec.rpm_scale_factor;
            else
                return (100.0f / 8349.0f) * ((double)ticks) + (94.0f / 13915.0f);
        }
    }

}




#endif //DXL_INTERFACE_CONVERSIONS_H
