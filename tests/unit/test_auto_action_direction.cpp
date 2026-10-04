/// test_auto_action_direction.cpp — Climate action in hardware AUTO (HA HEAT_COOL / AUTO)
/// taken from the unit's own state instead of a setpoint estimate.
/// Deps: cn105_protocol.h, cn105_types.h (production)
///
/// Before: updateAction() guessed heating/cooling/idle in HEAT_COOL and AUTO by
/// comparing the room temperature with target_temperature_low/high. With a single
/// setpoint, ESPHome stores target_temperature in the same union slot as
/// target_temperature_low and leaves target_temperature_high at NAN, so the guess
/// could never report cooling there.
///
/// After: the direction comes from the 0x09 status packet (auto sub mode AUTO_HEAT /
/// AUTO_COOL, or the PREHEAT sub mode) and active versus idle from the operating
/// flag, as in HEAT and COOL. The setpoint estimate stays as the fallback when the
/// unit reports no direction.
///
/// Pattern: auto_direction_from_sub_modes() is the production function; the
/// HEAT_COOL / AUTO branch of updateAction() is mirrored below (it is a method on
/// the full CN105Climate object), kept faithful to climateControls.cpp.
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "cn105_protocol.h"
#include "cn105_types.h"

using cn105_protocol::AutoDirection;
using cn105_protocol::auto_direction_from_sub_modes;

namespace {

enum class Action { IDLE, HEATING, COOLING };

// Mirror of CN105Climate::setActionIfOperatingTo() (stage fallback not configured).
Action ifOperating(Action action, bool operating) {
    return operating ? action : Action::IDLE;
}

// Mirror of the HEAT_COOL / AUTO cases of CN105Climate::updateAction() for a unit
// that supports both HEAT and COOL: setActionFromHardwareAutoDirection() first, then
// the pre-existing deadband estimate.
Action autoModeAction(const char* sub_mode, const char* auto_sub_mode, bool operating,
                      float current, float low, float high) {
    switch (auto_direction_from_sub_modes(sub_mode, auto_sub_mode)) {
    case AutoDirection::HEATING:
        return ifOperating(Action::HEATING, operating);
    case AutoDirection::COOLING:
        return ifOperating(Action::COOLING, operating);
    default:
        break;
    }
    if (current >= high) {
        return ifOperating(Action::COOLING, operating);
    } else if (current <= low) {
        return ifOperating(Action::HEATING, operating);
    }
    return ifOperating(Action::IDLE, operating);
}

// Single setpoint: ESPHome's union puts target_temperature in target_temperature_low;
// target_temperature_high stays NAN (componentEntries.cpp setup()).
Action singleSetpointAutoAction(const char* sub_mode, const char* auto_sub_mode,
                                bool operating, float current, float target) {
    return autoModeAction(sub_mode, auto_sub_mode, operating, current, target, NAN);
}

const char* const NORMAL = SUB_MODE_MAP[0];
const char* const PREHEAT = SUB_MODE_MAP[3];
const char* const AUTO_OFF = AUTO_SUB_MODE_MAP[0];
const char* const AUTO_COOL = AUTO_SUB_MODE_MAP[1];
const char* const AUTO_HEAT = AUTO_SUB_MODE_MAP[2];
const char* const AUTO_LEADER = AUTO_SUB_MODE_MAP[3];

}  // namespace

// ══════════════════════════════════════════════════════════════
// auto_direction_from_sub_modes() — production function
// ══════════════════════════════════════════════════════════════

TEST(AutoDirectionTest, MapStringsAreTheOnesTheFunctionCompares) {
    // Pins the lookup-table entries the function relies on.
    EXPECT_STREQ(PREHEAT, "PREHEAT");
    EXPECT_STREQ(AUTO_COOL, "AUTO_COOL");
    EXPECT_STREQ(AUTO_HEAT, "AUTO_HEAT");
}

TEST(AutoDirectionTest, AutoHeatIsHeating) {
    EXPECT_EQ(auto_direction_from_sub_modes(NORMAL, AUTO_HEAT), AutoDirection::HEATING);
}

TEST(AutoDirectionTest, AutoCoolIsCooling) {
    EXPECT_EQ(auto_direction_from_sub_modes(NORMAL, AUTO_COOL), AutoDirection::COOLING);
}

TEST(AutoDirectionTest, PreheatIsHeating) {
    EXPECT_EQ(auto_direction_from_sub_modes(PREHEAT, AUTO_HEAT), AutoDirection::HEATING);
    EXPECT_EQ(auto_direction_from_sub_modes(PREHEAT, AUTO_OFF), AutoDirection::HEATING);
    EXPECT_EQ(auto_direction_from_sub_modes(PREHEAT, nullptr), AutoDirection::HEATING);
}

TEST(AutoDirectionTest, OtherAutoSubModesGiveNoDirection) {
    for (int i = 0; i < 7; i++) {
        const char* asm_value = AUTO_SUB_MODE_MAP[i];
        if (std::strcmp(asm_value, "AUTO_HEAT") == 0 || std::strcmp(asm_value, "AUTO_COOL") == 0) {
            continue;
        }
        EXPECT_EQ(auto_direction_from_sub_modes(NORMAL, asm_value), AutoDirection::UNKNOWN)
            << "auto sub mode " << asm_value;
    }
}

TEST(AutoDirectionTest, OtherSubModesGiveNoDirectionOnTheirOwn) {
    for (int i = 0; i < 6; i++) {
        const char* sub_mode = SUB_MODE_MAP[i];
        if (std::strcmp(sub_mode, "PREHEAT") == 0) {
            continue;
        }
        EXPECT_EQ(auto_direction_from_sub_modes(sub_mode, AUTO_OFF), AutoDirection::UNKNOWN)
            << "sub mode " << sub_mode;
    }
}

TEST(AutoDirectionTest, NothingReceivedYet) {
    EXPECT_EQ(auto_direction_from_sub_modes(nullptr, nullptr), AutoDirection::UNKNOWN);
}

// ══════════════════════════════════════════════════════════════
// Climate action in HEAT_COOL / AUTO (mirror of updateAction())
// ══════════════════════════════════════════════════════════════

TEST(AutoModeActionTest, AutoHeatWithCompressorResting_IsIdle) {
    // AUTO_HEAT stays set between heating cycles: it gives the direction, not activity.
    EXPECT_EQ(singleSetpointAutoAction(NORMAL, AUTO_HEAT, /*operating=*/false, 24.5f, 24.0f),
              Action::IDLE);
}

TEST(AutoModeActionTest, AutoHeatOperating_IsHeating_EvenAboveSetpoint) {
    // The estimate said idle here (room 24.5 > setpoint 24.0) while the unit heated.
    EXPECT_EQ(singleSetpointAutoAction(NORMAL, AUTO_HEAT, true, 24.5f, 24.0f), Action::HEATING);
}

TEST(AutoModeActionTest, AutoCoolOperating_IsCooling_EvenBelowSetpoint) {
    // The estimate said heating here (room at or below the setpoint) while the unit cooled.
    EXPECT_EQ(singleSetpointAutoAction(NORMAL, AUTO_COOL, true, 23.5f, 24.0f), Action::COOLING);
    EXPECT_EQ(autoModeAction(NORMAL, AUTO_COOL, true, 19.5f, 20.0f, 24.0f), Action::COOLING);
}

TEST(AutoModeActionTest, AutoCoolNotOperating_IsIdle) {
    EXPECT_EQ(singleSetpointAutoAction(NORMAL, AUTO_COOL, false, 26.0f, 24.0f), Action::IDLE);
}

TEST(AutoModeActionTest, SingleSetpoint_CanReportCooling) {
    EXPECT_EQ(singleSetpointAutoAction(NORMAL, AUTO_COOL, true, 26.0f, 24.0f), Action::COOLING);
}

TEST(AutoModeActionTest, Preheat_IsHeatingWhileOperating) {
    EXPECT_EQ(singleSetpointAutoAction(PREHEAT, nullptr, true, 25.0f, 24.0f), Action::HEATING);
    EXPECT_EQ(singleSetpointAutoAction(PREHEAT, AUTO_HEAT, true, 22.0f, 24.0f), Action::HEATING);
}

// ── Fallback: no direction from the unit → the pre-existing estimate, unchanged ──

TEST(AutoModeActionTest, Fallback_DualSetpointDeadband) {
    for (const char* asm_value : {static_cast<const char*>(nullptr), AUTO_OFF, AUTO_LEADER}) {
        EXPECT_EQ(autoModeAction(NORMAL, asm_value, true, 25.0f, 20.0f, 24.0f), Action::COOLING);
        EXPECT_EQ(autoModeAction(NORMAL, asm_value, true, 19.0f, 20.0f, 24.0f), Action::HEATING);
        EXPECT_EQ(autoModeAction(NORMAL, asm_value, true, 22.0f, 20.0f, 24.0f), Action::IDLE);
        EXPECT_EQ(autoModeAction(NORMAL, asm_value, false, 25.0f, 20.0f, 24.0f), Action::IDLE);
    }
}

TEST(AutoModeActionTest, Fallback_SingleSetpointKeepsItsOldLimits) {
    // Unchanged for units without an auto sub mode: heating at or below the setpoint
    // while operating, idle above it, never cooling (target_temperature_high is NAN).
    EXPECT_EQ(singleSetpointAutoAction(NORMAL, nullptr, true, 24.0f, 24.0f), Action::HEATING);
    EXPECT_EQ(singleSetpointAutoAction(NORMAL, nullptr, true, 26.0f, 24.0f), Action::IDLE);
}

TEST(AutoModeActionTest, Fallback_MfzBitfieldStatesUnchanged) {
    // AUTO_INACTIVE / AUTO_IDLE / AUTO_ACTIVE (MFZ units) carry no direction.
    for (int i = 4; i < 7; i++) {
        EXPECT_EQ(autoModeAction(NORMAL, AUTO_SUB_MODE_MAP[i], true, 25.0f, 20.0f, 24.0f),
                  Action::COOLING) << AUTO_SUB_MODE_MAP[i];
        EXPECT_EQ(autoModeAction(NORMAL, AUTO_SUB_MODE_MAP[i], true, 19.0f, 20.0f, 24.0f),
                  Action::HEATING) << AUTO_SUB_MODE_MAP[i];
    }
}
