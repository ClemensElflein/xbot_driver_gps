//
// Created by Clemens Elflein on 15.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#pragma once

#include <vector>

#pragma pack(push, 1)

// UBX-NAV-PVT message
// see 5.14.11: https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf
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

        FLAGS2_CONFIRMED_AVAILABLE = 32u,
        FLAGS2_CONFIRMED_DATE = 64u,
        FLAGS2_CONFIRMED_TIME = 128u,

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
    };


    // GPS time of week of the navigation epoch.
    // (ms)
    uint32_t iTOW;

    // Year (UTC)
    uint16_t year;

    // Month, range 1..12 (UTC)
    uint8_t month;

    // Day of month, range 1..31 (UTC)
    uint8_t day;

    // Hour of day, range 0..23 (UTC)
    uint8_t hour;

    // Minute of hour, range 0..59 (UTC)
    uint8_t min;

    // Seconds of minute, range 0..60 (UTC)
    uint8_t sec;

    // Validity flags, see VALID_ constants
    uint8_t valid;

    // Time accuracy estimate (UTC) (ns)
    uint32_t tAcc;

    // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
    int32_t nano;

    // GNSSfix Type, see FIX_TYPE_ constants
    uint8_t fixType;

    // Fix status flags, see FLAGS_ constants
    uint8_t flags;

    // Additional flags, see FLAGS2_ constants
    uint8_t flags2;

    // Number of satellites used in Nav Solution
    uint8_t numSV;

    // Longitude (deg)
    int32_t lon;

    // Latitude (deg)
    int32_t lat;

    // Height above ellipsoid (mm)
    int32_t height;

    // Height above mean sea level (mm)
    int32_t hMSL;

    // Horizontal accuracy estimate (mm)
    uint32_t hAcc;

    // Vertical accuracy estimate (mm)
    uint32_t vAcc;

    // NED north velocity (mm/s)
    int32_t velN;

    // NED east velocity (mm/s)
    int32_t velE;

    // NED down velocity (mm/s)
    int32_t velD;

    // Ground Speed (2-D) (mm/s)
    int32_t gSpeed;

    // Heading of motion (2-D) (deg)
    int32_t headMot;

    // Speed accuracy estimate (mm/s)
    uint32_t sAcc;

    // Heading accuracy estimate (both motion and vehicle) (deg)
    uint32_t headAcc;

    // Position DOP
    uint16_t pDOP;

    // Additional flags
    uint16_t flags3;

    // Reserved
    uint8_t reserved1[4];

    // Heading of vehicle (2-D) (deg)
    int32_t headVeh;

    // Magnetic declination (deg)
    int16_t magDec;

    // Magnetic declination accuracy (deg)
    uint16_t magAcc;
} __attribute__((packed));

typedef std::shared_ptr<UbxNavPvt> UbxNavPvtPtr;
typedef std::shared_ptr<UbxNavPvt const> UbxNavPvtConstPtr;

#pragma pack(pop)
