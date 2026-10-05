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
/// After: the direction is the last one the unit reported in its 0x09 status packet
/// during this AUTO session (AUTO_HEAT / AUTO_COOL, or the PREHEAT sub mode), kept
/// through values without a direction (AUTO_LEADER, which an MSZ-AP unit reports for
/// the whole time it actually cools); active versus idle comes from the operating
/// flag, as in HEAT and COOL. The setpoint estimate stays as the fallback until a
/// direction has been seen.
///
/// Pattern: next_auto_direction() is the production function; the HEAT_COOL / AUTO
/// branch of updateAction() is mirrored below (it is a method on the full
/// CN105Climate object), kept faithful to climateControls.cpp.
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <vector>
#include "cn105_protocol.h"
#include "cn105_types.h"

using cn105_protocol::AutoDirection;
using cn105_protocol::next_auto_direction;

namespace {

enum class Action { IDLE, HEATING, COOLING };

// Mirror of CN105Climate::setActionIfOperatingTo() (stage fallback not configured).
Action ifOperating(Action action, bool operating) {
    return operating ? action : Action::IDLE;
}

// Mirror of the HEAT_COOL / AUTO cases of CN105Climate::updateAction() for a unit
// that supports both HEAT and COOL: setActionFromHardwareAutoDirection() first, then
// the pre-existing deadband estimate.
Action autoModeAction(AutoDirection direction, bool operating, float current, float low, float high) {
    switch (direction) {
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

const char* const NORMAL = SUB_MODE_MAP[0];
const char* const PREHEAT = SUB_MODE_MAP[3];
const char* const AUTO_OFF = AUTO_SUB_MODE_MAP[0];
const char* const AUTO_COOL = AUTO_SUB_MODE_MAP[1];
const char* const AUTO_HEAT = AUTO_SUB_MODE_MAP[2];
const char* const AUTO_LEADER = AUTO_SUB_MODE_MAP[3];
const char* const AUTO_INACTIVE = AUTO_SUB_MODE_MAP[4];
const char* const AUTO_IDLE = AUTO_SUB_MODE_MAP[5];
const char* const AUTO_ACTIVE = AUTO_SUB_MODE_MAP[6];

// One poll: the 0x09 sub modes, the 0x06 operating flag, room temperature.
struct Poll {
    const char* sub_mode;
    const char* auto_sub_mode;
    bool operating;
    float current;
};

// Replays polls in single-setpoint HEAT_COOL/AUTO: the direction is updated as in
// getPowerFromResponsePacket(), the action as in updateAction(). target_temperature
// shares its union slot with target_temperature_low; target_temperature_high is NAN.
std::vector<Action> replaySingleSetpoint(const std::vector<Poll>& polls, float target) {
    AutoDirection direction = AutoDirection::UNKNOWN;  // boot
    std::vector<Action> actions;
    for (const auto& p : polls) {
        direction = next_auto_direction(direction, p.sub_mode, p.auto_sub_mode);
        actions.push_back(autoModeAction(direction, p.operating, p.current, target, NAN));
    }
    return actions;
}

}  // namespace

// ══════════════════════════════════════════════════════════════
// next_auto_direction() — production function
// ══════════════════════════════════════════════════════════════

TEST(AutoDirectionTest, MapStringsAreTheOnesTheFunctionCompares) {
    // Pins the lookup-table entries the function relies on.
    EXPECT_STREQ(PREHEAT, "PREHEAT");
    EXPECT_STREQ(AUTO_OFF, "AUTO_OFF");
    EXPECT_STREQ(AUTO_COOL, "AUTO_COOL");
    EXPECT_STREQ(AUTO_HEAT, "AUTO_HEAT");
    EXPECT_STREQ(AUTO_LEADER, "AUTO_LEADER");
    EXPECT_STREQ(AUTO_INACTIVE, "AUTO_INACTIVE");
}

TEST(AutoDirectionTest, DirectionValuesSetIt) {
    for (AutoDirection prev : {AutoDirection::UNKNOWN, AutoDirection::HEATING, AutoDirection::COOLING}) {
        EXPECT_EQ(next_auto_direction(prev, NORMAL, AUTO_HEAT), AutoDirection::HEATING);
        EXPECT_EQ(next_auto_direction(prev, NORMAL, AUTO_COOL), AutoDirection::COOLING);
        EXPECT_EQ(next_auto_direction(prev, PREHEAT, AUTO_OFF), AutoDirection::HEATING);
        EXPECT_EQ(next_auto_direction(prev, PREHEAT, nullptr), AutoDirection::HEATING);
    }
}

TEST(AutoDirectionTest, AutoNotSelectedClearsIt) {
    EXPECT_EQ(next_auto_direction(AutoDirection::COOLING, NORMAL, AUTO_OFF), AutoDirection::UNKNOWN);
    EXPECT_EQ(next_auto_direction(AutoDirection::HEATING, NORMAL, AUTO_INACTIVE), AutoDirection::UNKNOWN);
}

TEST(AutoDirectionTest, ValuesWithoutDirectionKeepIt) {
    for (const char* asm_value : {AUTO_LEADER, AUTO_IDLE, AUTO_ACTIVE, static_cast<const char*>(nullptr)}) {
        EXPECT_EQ(next_auto_direction(AutoDirection::COOLING, NORMAL, asm_value), AutoDirection::COOLING)
            << (asm_value ? asm_value : "nullptr");
        EXPECT_EQ(next_auto_direction(AutoDirection::HEATING, NORMAL, asm_value), AutoDirection::HEATING)
            << (asm_value ? asm_value : "nullptr");
        EXPECT_EQ(next_auto_direction(AutoDirection::UNKNOWN, NORMAL, asm_value), AutoDirection::UNKNOWN)
            << (asm_value ? asm_value : "nullptr");
    }
}

TEST(AutoDirectionTest, EveryMapEntryIsHandled) {
    // Only AUTO_HEAT / AUTO_COOL set a direction, only AUTO_OFF / AUTO_INACTIVE clear it.
    for (int i = 0; i < 7; i++) {
        const char* v = AUTO_SUB_MODE_MAP[i];
        AutoDirection next = next_auto_direction(AutoDirection::HEATING, NORMAL, v);
        if (std::strcmp(v, "AUTO_COOL") == 0) {
            EXPECT_EQ(next, AutoDirection::COOLING);
        } else if (std::strcmp(v, "AUTO_OFF") == 0 || std::strcmp(v, "AUTO_INACTIVE") == 0) {
            EXPECT_EQ(next, AutoDirection::UNKNOWN) << v;
        } else {
            EXPECT_EQ(next, AutoDirection::HEATING) << v;
        }
    }
    // Sub modes other than PREHEAT carry no direction.
    for (int i = 0; i < 6; i++) {
        const char* s = SUB_MODE_MAP[i];
        if (std::strcmp(s, "PREHEAT") == 0) continue;
        EXPECT_EQ(next_auto_direction(AutoDirection::COOLING, s, AUTO_LEADER), AutoDirection::COOLING) << s;
    }
}

// ══════════════════════════════════════════════════════════════
// Recorded sessions (single setpoint, mirror of updateAction())
// ══════════════════════════════════════════════════════════════

TEST(AutoModeSessionTest, MszApCoolingReportsLeaderWhileTheCompressorRuns) {
    // Night test 2026-10-05, MSZ-AP unit, heat -> heat_cool at setpoint 23, room 27.
    const auto a = replaySingleSetpoint({
        {NORMAL, AUTO_OFF, true, 27.0f},     // 02:57:06 still HEAT
        {NORMAL, AUTO_COOL, true, 27.0f},    // 02:57:19 AUTO chose cooling (changeover)
        {NORMAL, AUTO_LEADER, true, 27.0f},  // 03:00:28 compressor starts cooling, 0x03
        {NORMAL, AUTO_LEADER, true, 24.0f},  // 03:37 still cooling at 119 W
        {NORMAL, AUTO_LEADER, false, 24.0f}, // 03:40:46 setpoint raised to 26: unit pauses
    }, 23.0f);
    ASSERT_EQ(a.size(), 5u);
    EXPECT_EQ(a[1], Action::COOLING);
    EXPECT_EQ(a[2], Action::COOLING);  // was IDLE with AUTO_LEADER treated as "no direction"
    EXPECT_EQ(a[3], Action::COOLING);
    EXPECT_EQ(a[4], Action::IDLE);
}

TEST(AutoModeSessionTest, MszApHeatingKeepsAutoHeatWithTheCompressorOnAndOff) {
    // HA baseline, MSZ-AP unit in AUTO for 23.4 h: AUTO_COOL for 3 min, then AUTO_HEAT
    // throughout, with PREHEAT at each start and long compressor rests.
    const auto a = replaySingleSetpoint({
        {NORMAL, AUTO_COOL, false, 20.5f},
        {PREHEAT, AUTO_HEAT, true, 20.5f},
        {NORMAL, AUTO_HEAT, true, 21.0f},
        {NORMAL, AUTO_HEAT, true, 24.5f},   // room above setpoint, still heating
        {NORMAL, AUTO_HEAT, false, 24.5f},  // compressor resting
        {PREHEAT, AUTO_HEAT, true, 23.5f},
    }, 24.0f);
    EXPECT_EQ(a[0], Action::IDLE);
    EXPECT_EQ(a[1], Action::HEATING);
    EXPECT_EQ(a[2], Action::HEATING);
    EXPECT_EQ(a[3], Action::HEATING);
    EXPECT_EQ(a[4], Action::IDLE);
    EXPECT_EQ(a[5], Action::HEATING);
}

TEST(AutoModeSessionTest, AutoCoolOperatingIsCoolingEvenBelowSetpoint) {
    // The estimate said heating here (room at or below the setpoint) while the unit cooled.
    const auto a = replaySingleSetpoint({{NORMAL, AUTO_COOL, true, 23.5f}}, 24.0f);
    EXPECT_EQ(a[0], Action::COOLING);
    EXPECT_EQ(autoModeAction(AutoDirection::COOLING, true, 19.5f, 20.0f, 24.0f), Action::COOLING);
}

TEST(AutoModeSessionTest, NewAutoSessionDoesNotReuseAnOldDirection) {
    const auto a = replaySingleSetpoint({
        {NORMAL, AUTO_HEAT, true, 22.0f},    // heating session
        {NORMAL, AUTO_OFF, true, 26.0f},     // user switched to COOL: AUTO not selected
        {NORMAL, AUTO_LEADER, true, 26.0f},  // back in AUTO, no direction seen yet
    }, 24.0f);
    EXPECT_EQ(a[0], Action::HEATING);
    EXPECT_EQ(a[2], Action::IDLE);  // estimate (single setpoint, room above target)
}

// ── Fallback: no direction seen yet → the pre-existing estimate, unchanged ──

TEST(AutoModeFallbackTest, BootDuringALeaderSessionUsesTheEstimate) {
    // Known limit: after a reboot in the middle of a cooling session that only reports
    // AUTO_LEADER, the estimate is used until the unit reports a direction again.
    const auto a = replaySingleSetpoint({{NORMAL, AUTO_LEADER, true, 26.0f}}, 23.0f);
    EXPECT_EQ(a[0], Action::IDLE);
}

TEST(AutoModeFallbackTest, DualSetpointDeadband) {
    EXPECT_EQ(autoModeAction(AutoDirection::UNKNOWN, true, 25.0f, 20.0f, 24.0f), Action::COOLING);
    EXPECT_EQ(autoModeAction(AutoDirection::UNKNOWN, true, 19.0f, 20.0f, 24.0f), Action::HEATING);
    EXPECT_EQ(autoModeAction(AutoDirection::UNKNOWN, true, 22.0f, 20.0f, 24.0f), Action::IDLE);
    EXPECT_EQ(autoModeAction(AutoDirection::UNKNOWN, false, 25.0f, 20.0f, 24.0f), Action::IDLE);
}

TEST(AutoModeFallbackTest, SingleSetpointKeepsItsOldLimits) {
    // Unchanged until a direction is seen: heating at or below the setpoint while
    // operating, idle above it, never cooling (target_temperature_high is NAN).
    EXPECT_EQ(autoModeAction(AutoDirection::UNKNOWN, true, 24.0f, 24.0f, NAN), Action::HEATING);
    EXPECT_EQ(autoModeAction(AutoDirection::UNKNOWN, true, 26.0f, 24.0f, NAN), Action::IDLE);
}

TEST(AutoModeFallbackTest, MfzBitfieldUnitsNeverLearnADirection) {
    // AUTO_INACTIVE / AUTO_IDLE / AUTO_ACTIVE carry no direction: the estimate stays.
    const auto a = replaySingleSetpoint({
        {NORMAL, AUTO_INACTIVE, false, 25.0f},
        {NORMAL, AUTO_IDLE, false, 25.0f},
        {NORMAL, AUTO_ACTIVE, true, 23.0f},
    }, 24.0f);
    EXPECT_EQ(a[2], Action::HEATING);  // estimate: room below the single setpoint
}

// ══════════════════════════════════════════════════════════════
// Poll-cycle ordering: one AUTO action per cycle, after the 0x09 reply
// (mirror of statusChanged / getPowerFromResponsePacket / refreshAutoModeAction /
// terminateCycle / publishWantedSettingsStateToHA)
// ══════════════════════════════════════════════════════════════

namespace {

// A poll cycle asks 0x02, 0x03, 0x06, 0x09 in that order (registerInfoRequests()), so the
// operating flag (0x06) is processed before the direction (0x09) of the same cycle.
struct CycleSim {
    bool once_per_cycle;               // true: this PR; false: the previous revision (every reply)
    bool hw_auto = true;               // HA mode HEAT_COOL / AUTO
    bool operating = false;
    const char* sub_mode = NORMAL;
    const char* auto_sub_mode = AUTO_OFF;
    AutoDirection direction = AutoDirection::UNKNOWN;
    float current = 24.0f;
    float target = 26.0f;              // single setpoint
    bool due = false;                  // auto_mode_action_due_
    bool set_pending = false;          // wantedSettings.hasChanged && !hasBeenSent
    Action action = Action::IDLE;
    std::vector<Action> published;     // every action change that reached HA

    void compute() {
        const Action previous = action;
        action = autoModeAction(direction, operating, current, target, NAN);
        if (action != previous) published.push_back(action);
    }
    // updateAction() without the refresh flag.
    void updateAction() {
        if (!hw_auto) return;
        if (once_per_cycle) {
            due = true;
        } else {
            compute();
        }
    }
    // refreshAutoModeAction().
    void refresh() {
        if (!once_per_cycle || !due || set_pending) return;
        due = false;
        compute();
    }
    // 0x06 reply -> statusChanged() when the flag changed.
    void status06(bool op) {
        if (op == operating) return;
        operating = op;
        updateAction();
    }
    // 0x09 reply -> getPowerFromResponsePacket().
    void sub09(const char* sub, const char* asm_value) {
        const bool changed = std::strcmp(sub, sub_mode) != 0 || std::strcmp(asm_value, auto_sub_mode) != 0;
        sub_mode = sub;
        auto_sub_mode = asm_value;
        if (changed) {
            direction = next_auto_direction(direction, sub_mode, auto_sub_mode);
            updateAction();
        }
        refresh();
    }
    // 0x02 reply with changed settings -> publishStateToHA().
    void settingsChanged() { updateAction(); }
    // End of the cycle -> terminateCycle().
    void endCycle() { refresh(); }
    // HA switches into HEAT_COOL/AUTO -> control(): mode set, SET queued, no updateAction().
    void userEntersAuto(float new_target) {
        hw_auto = true;
        target = new_target;
        set_pending = true;
    }
    // SET written -> publishWantedSettingsStateToHA() calls updateAction().
    void setSent() {
        set_pending = false;
        updateAction();
    }
};

}  // namespace

TEST(AutoModeCycleTest, HeatingStart_NoCoolingBlip) {
    // Night test 05:22:33: after a cooling session (0x01 then 0x03), AUTO starts heating.
    // 0x06 at .298 brings operating 0->1 while the kept direction is still COOLING;
    // 0x09 at .529 brings 0x03 -> 0x02 (AUTO_HEAT) in the same cycle.
    for (bool once : {false, true}) {
        CycleSim s{once};
        s.direction = AutoDirection::COOLING;
        s.auto_sub_mode = AUTO_LEADER;
        s.status06(true);
        s.sub09(NORMAL, AUTO_HEAT);
        s.endCycle();
        if (once) {
            ASSERT_EQ(s.published.size(), 1u);
            EXPECT_EQ(s.published[0], Action::HEATING);
        } else {
            // The previous revision published "cooling" for 250 ms, then "heating".
            ASSERT_EQ(s.published.size(), 2u);
            EXPECT_EQ(s.published[0], Action::COOLING);
            EXPECT_EQ(s.published[1], Action::HEATING);
        }
    }
}

TEST(AutoModeCycleTest, CoolToHeatCool_NoIdleGap) {
    // Night test 05:02:09-13: COOL (cooling, operating) -> HA heat_cool at setpoint 23, room 24.
    // In COOL the unit reports 0x00, which clears the direction. Sequence: mode change (SET
    // queued); in-flight 0x06/0x09 still COOL (0x00); SET written; next cycle 0x02 reports AUTO;
    // 0x09 reports 0x01 (AUTO_COOL).
    for (bool once : {false, true}) {
        CycleSim s{once};
        s.hw_auto = false;
        s.operating = true;
        s.current = 24.0f;
        s.action = Action::COOLING;  // shown in COOL
        s.userEntersAuto(23.0f);
        s.status06(true);
        s.sub09(NORMAL, AUTO_OFF);
        s.endCycle();
        s.setSent();
        s.settingsChanged();
        s.status06(true);
        s.sub09(NORMAL, AUTO_COOL);
        s.endCycle();
        EXPECT_EQ(s.action, Action::COOLING);
        if (once) {
            EXPECT_TRUE(s.published.empty());  // stays "cooling" throughout
        } else {
            // The previous revision showed "idle" for 3.6 s (estimate) until the 0x01 poll.
            ASSERT_EQ(s.published.size(), 2u);
            EXPECT_EQ(s.published[0], Action::IDLE);
            EXPECT_EQ(s.published[1], Action::COOLING);
        }
    }
}

TEST(AutoModeCycleTest, CompressorStopIsShownOnTheSamePoll) {
    // Night test 05:07:39: operating 1 -> 0 with 0x03 unchanged: idle at that poll's 0x09 reply.
    CycleSim s{true};
    s.direction = AutoDirection::COOLING;
    s.auto_sub_mode = AUTO_LEADER;
    s.operating = true;
    s.action = Action::COOLING;
    s.status06(false);
    EXPECT_EQ(s.action, Action::COOLING);  // not yet: waiting for the 0x09 reply
    s.sub09(NORMAL, AUTO_LEADER);
    EXPECT_EQ(s.action, Action::IDLE);
}

TEST(AutoModeCycleTest, UnitWithoutA0x09ReplyIsRefreshedAtCycleEnd) {
    CycleSim s{true};
    s.current = 25.0f;
    s.status06(true);
    EXPECT_EQ(s.action, Action::IDLE);
    s.endCycle();                      // 0x09 timed out; terminateCycle() refreshes
    EXPECT_EQ(s.action, Action::HEATING);  // estimate: room 25 below the single setpoint 26
}

TEST(AutoModeCycleTest, NotWhileASetIsWaiting) {
    CycleSim s{true};
    s.direction = AutoDirection::HEATING;
    s.set_pending = true;
    s.status06(true);
    s.sub09(NORMAL, AUTO_HEAT);
    s.endCycle();
    EXPECT_EQ(s.action, Action::IDLE);     // replies describe the unit's previous settings
    s.setSent();
    s.status06(true);
    s.sub09(NORMAL, AUTO_HEAT);
    EXPECT_EQ(s.action, Action::HEATING);
}
