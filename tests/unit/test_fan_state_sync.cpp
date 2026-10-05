/// test_fan_state_sync.cpp — A fan speed the unit refuses must reach HA.
/// Deps: cn105_types.h, cn105_protocol.h (production)
///
/// Bug: after a fan command the component publishes the requested speed at once
/// (publishWantedSettingsStateToHA -> checkFanSettings(wantedSettings, false)),
/// without touching currentSettings.fan. The read path then compares the unit's
/// next report with currentSettings.fan, i.e. with the unit's previous report.
/// When the unit refuses the speed (for example HIGH while it runs its own AUTO)
/// both reports are equal, heatpumpUpdate() sees "no change", and HA keeps showing
/// the refused speed until something else changes the fan.
///
/// Fix: the sent speed becomes currentSettings.fan, so the next report is compared
/// with what was sent. A differing report inside the grace window after the user
/// command (the unit may not have applied the SET yet) is held back; the next one
/// after the window is applied. Nothing is re-sent.
///
/// Pattern: production structs (heatpumpSettings / wantedHeatpumpSettings) and the
/// production predicate fan_disagrees_within_grace(); the CN105Climate methods are
/// mirrored below, kept faithful to climateControls.cpp (setFanSpeed),
/// hp_writings.cpp (sendWantedSettingsDelegate / publishWantedSettingsStateToHA)
/// and hp_readings.cpp (heatpumpUpdate / publishStateToHA / checkFanSettings).
#include <gtest/gtest.h>
#include <cstring>
#include "cn105_protocol.h"
#include "cn105_types.h"

namespace {

constexpr uint32_t kGraceMs = RECEIVED_SETPOINT_GRACE_WINDOW_MS;

const char* fanSetting(const char* value) {  // FAN_MAP pointer, as lookups return
    for (int i = 0; i < 6; i++) {
        if (std::strcmp(FAN_MAP[i], value) == 0) return FAN_MAP[i];
    }
    return nullptr;
}

struct FanSim {
    bool fixed;                       // true: this PR; false: the code before it
    heatpumpSettings current{};       // currentSettings
    wantedHeatpumpSettings wanted{};  // wantedSettings
    const char* shown = nullptr;      // fan mode published to HA (FAN_MAP string)

    // Mirror of checkFanSettings(): hasChanged() against currentSettings.fan.
    void checkFanSettings(const heatpumpSettings& s, bool updateCurrentSettings) {
        if (s.fan == nullptr) return;
        if (current.fan == nullptr || std::strcmp(current.fan, s.fan) != 0) {
            if (updateCurrentSettings) current.fan = s.fan;
            shown = s.fan;
        }
    }

    // Mirror of control() -> controlFan() -> setFanSpeed() -> finalizeControlIfUpdated().
    void userSetsFan(const char* value, uint32_t now) {
        wanted.fan = fanSetting(value);
        if (fixed) {
            wanted.last_user_fan = wanted.fan;
            wanted.last_user_fan_ms = now;
        }
        wanted.hasChanged = true;
        shown = wanted.fan;  // optimistic publish_state()
    }

    // Mirror of sendWantedSettingsDelegate(): the SET packet, then
    // publishWantedSettingsStateToHA(), then wantedSettings.resetSettings().
    void sendWanted() {
        if (wanted.fan != nullptr) checkFanSettings(wanted, /*updateCurrentSettings=*/fixed);
        wanted.resetSettings();
    }

    // Mirror of getSettingsFromResponsePacket() -> heatpumpUpdate() -> publishStateToHA(),
    // for a report whose other fields equal currentSettings.
    void unitReports(const char* value, uint32_t now) {
        heatpumpSettings received = current;
        received.fan = fanSetting(value);
        if (!(received != current)) return;  // heatpumpUpdate(): no change, nothing published
        const bool ignore = fixed && cn105_protocol::fan_disagrees_within_grace(
            received.fan, wanted.last_user_fan, wanted.last_user_fan_ms, now, kGraceMs);
        if (wanted.fan == nullptr && !ignore) {
            checkFanSettings(received, true);
        }
    }
};

FanSim startAt(const char* fan, bool fixed) {
    FanSim sim{fixed};
    sim.unitReports(fan, 100);  // first report after connect
    return sim;
}

}  // namespace

// ══════════════════════════════════════════════════════════════
// The bug, pinned on the old flow
// ══════════════════════════════════════════════════════════════

TEST(FanStateSyncTest, OldFlow_RefusedSpeedStaysShown) {
    FanSim sim = startAt("AUTO", /*fixed=*/false);
    sim.userSetsFan("4", 10000);  // HIGH
    sim.sendWanted();
    sim.unitReports("AUTO", 11500);
    sim.unitReports("AUTO", 14000);
    sim.unitReports("AUTO", 60000);
    EXPECT_STREQ(sim.shown, "4");  // HA keeps showing HIGH
}

// ══════════════════════════════════════════════════════════════
// The fix
// ══════════════════════════════════════════════════════════════

TEST(FanStateSyncTest, RefusedSpeedReachesHaAfterGraceWindow) {
    FanSim sim = startAt("AUTO", true);
    sim.userSetsFan("4", 10000);
    sim.sendWanted();
    EXPECT_STREQ(sim.current.fan, "4");  // comparison point moved to what was sent

    sim.unitReports("AUTO", 11500);      // inside the grace window: held back
    EXPECT_STREQ(sim.shown, "4");

    sim.unitReports("AUTO", 10000 + kGraceMs + 500);
    EXPECT_STREQ(sim.shown, "AUTO");
    EXPECT_STREQ(sim.current.fan, "AUTO");
}

TEST(FanStateSyncTest, AcceptedSpeed_StaleReportDoesNotBounce) {
    FanSim sim = startAt("AUTO", true);
    sim.userSetsFan("2", 10000);         // MEDIUM
    sim.sendWanted();
    sim.unitReports("AUTO", 11000);      // report from before the SET took effect
    EXPECT_STREQ(sim.shown, "2");
    sim.unitReports("2", 12500);
    sim.unitReports("2", 20000);
    EXPECT_STREQ(sim.shown, "2");
    EXPECT_STREQ(sim.current.fan, "2");
}

TEST(FanStateSyncTest, RemoteChangeWithoutRecentCommandIsAppliedAtOnce) {
    FanSim sim = startAt("AUTO", true);
    sim.unitReports("QUIET", 50000);     // e.g. IR remote
    EXPECT_STREQ(sim.shown, "QUIET");
}

TEST(FanStateSyncTest, RemoteChangeLongAfterACommandIsAppliedAtOnce) {
    FanSim sim = startAt("AUTO", true);
    sim.userSetsFan("1", 10000);         // LOW, accepted
    sim.sendWanted();
    sim.unitReports("1", 11500);
    sim.unitReports("3", 40000);         // MIDDLE from the IR remote
    EXPECT_STREQ(sim.shown, "3");
}

TEST(FanStateSyncTest, PendingUserCommandIsNotOverwritten) {
    FanSim sim = startAt("AUTO", true);
    sim.userSetsFan("4", 10000);         // not sent yet
    sim.unitReports("QUIET", 10050);
    EXPECT_STREQ(sim.shown, "4");
}

// ══════════════════════════════════════════════════════════════
// Production pieces
// ══════════════════════════════════════════════════════════════

TEST(FanStateSyncTest, LastUserFanSurvivesResetButFanIsNotResent) {
    wantedHeatpumpSettings ws{};
    ws.fan = FAN_MAP[5];
    ws.last_user_fan = FAN_MAP[5];
    ws.last_user_fan_ms = 1234;

    ws.resetSettings();

    // createPacket() only adds the fan byte when wantedSettings.fan is set.
    EXPECT_EQ(ws.fan, nullptr);
    EXPECT_STREQ(ws.last_user_fan, "4");
    EXPECT_EQ(ws.last_user_fan_ms, 1234u);
}

TEST(FanStateSyncTest, GracePredicate) {
    using cn105_protocol::fan_disagrees_within_grace;
    EXPECT_TRUE(fan_disagrees_within_grace("AUTO", "4", 1000, 2000, 3000));
    EXPECT_FALSE(fan_disagrees_within_grace("4", "4", 1000, 2000, 3000));
    EXPECT_FALSE(fan_disagrees_within_grace("AUTO", "4", 1000, 4000, 3000));   // window over
    EXPECT_FALSE(fan_disagrees_within_grace("AUTO", nullptr, 0, 2000, 3000));  // no command
    EXPECT_FALSE(fan_disagrees_within_grace(nullptr, "4", 1000, 2000, 3000));
}

TEST(FanStateSyncTest, FanMapBytes) {
    // MEDIUM is speed "2" (byte 0x03) and MIDDLE is speed "3" (byte 0x05), so MIDDLE is the faster one.
    EXPECT_STREQ(FAN_MAP[3], "2");
    EXPECT_EQ(FAN[3], 0x03);
    EXPECT_STREQ(FAN_MAP[4], "3");
    EXPECT_EQ(FAN[4], 0x05);
}
