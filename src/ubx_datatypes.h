//
// Created by Clemens Elflein on 15.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#ifndef XBOT_UBX_DATATYPES_H
#define XBOT_UBX_DATATYPES_H

namespace xbot {
    namespace driver {
        namespace gps {


#pragma pack(push, 1)
            struct UbxNavPvt {
                enum {
                    CLASS_ID = 1u,
                    MESSAGE_ID = 7u,
                    VALID_DATE = 1u,
                    VALID_TIME = 2u,
                    VALID_FULLY_RESOLVED = 4u,
                    VALID_MAG = 8u,
                    FIX_TYPE_NO_FIX = 0u,
                    FIX_TYPE_DEAD_RECKONING_ONLY = 1u,
                    FIX_TYPE_2D = 2u,
                    FIX_TYPE_3D = 3u,
                    FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4u,
                    FIX_TYPE_TIME_ONLY = 5u,
                    FLAGS_GNSS_FIX_OK = 1u,
                    FLAGS_DIFF_SOLN = 2u,
                    FLAGS_PSM_MASK = 28u,
                    PSM_OFF = 0u,
                    PSM_ENABLED = 4u,
                    PSM_ACQUIRED = 8u,
                    PSM_TRACKING = 12u,
                    PSM_POWER_OPTIMIZED_TRACKING = 16u,
                    PSM_INACTIVE = 20u,
                    FLAGS_HEAD_VEH_VALID = 32u,
                    FLAGS_CARRIER_PHASE_MASK = 192u,
                    CARRIER_PHASE_NO_SOLUTION = 0u,
                    CARRIER_PHASE_FLOAT = 64u,
                    CARRIER_PHASE_FIXED = 128u,
                    FLAGS2_CONFIRMED_AVAILABLE = 32u,
                    FLAGS2_CONFIRMED_DATE = 64u,
                    FLAGS2_CONFIRMED_TIME = 128u,
                };


                uint32_t iTOW;
                uint16_t year;
                uint8_t month;
                uint8_t day;
                uint8_t hour;
                uint8_t min;
                uint8_t sec;
                uint8_t valid;
                uint32_t tAcc;
                int32_t nano;
                uint8_t fixType;
                uint8_t flags;
                uint8_t flags2;
                uint8_t numSV;
                int32_t lon;
                int32_t lat;
                int32_t height;
                int32_t hMSL;
                uint32_t hAcc;
                uint32_t vAcc;
                int32_t velN;
                int32_t velE;
                int32_t velD;
                int32_t gSpeed;
                int32_t headMot;
                uint32_t sAcc;
                uint32_t headAcc;
                uint16_t pDOP;
                uint16_t flags3;
                uint8_t reserved1[4];
                int32_t headVeh;
                int16_t magDec;
                uint16_t magAcc;
            } __attribute__((packed));
#pragma pack(pop)
        }
    }
}
#endif //XBOT_UBX_DATATYPES_H
