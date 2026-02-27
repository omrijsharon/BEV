#include "ipc/shared_state.hpp"

#include <iostream>
#include <string>

#ifdef __linux__
#include <unistd.h>
#endif

int main() {
    std::string err;
#ifdef __linux__
    const std::string name = "/bev_test_shared_state_" + std::to_string(static_cast<long long>(::getpid()));
    bev::ipc::SharedState a;
    bev::ipc::SharedState b;
    if (!a.create(name, err)) {
        std::cerr << "create failed: " << err << "\n";
        return 1;
    }
    if (!b.attach(name, err)) {
        std::cerr << "attach failed: " << err << "\n";
        return 1;
    }
    const auto* block = b.block();
    a.setPauseMapUpdates(true);
    if (!b.pauseMapUpdates()) {
        std::cerr << "pause flag not visible\n";
        return 1;
    }
    a.requestShutdown();
    if (!b.shutdownRequested()) {
        std::cerr << "shutdown flag not visible\n";
        return 1;
    }
    a.setProtocolMismatch(true);
    if (!b.protocolMismatch()) {
        std::cerr << "protocol mismatch flag not visible\n";
        return 1;
    }
    const uint32_t rg = a.requestReload();
    if (rg == 0U || b.reloadGeneration() != rg) {
        std::cerr << "reload generation not visible\n";
        return 1;
    }
    a.addSyncSample(true, false, 1000U);
    a.addSyncSample(false, true, 3000U);
    const auto k = b.syncKpiSnapshot();
    if (k.sync_samples != 2U ||
        k.interpolated_samples != 1U ||
        k.stale_attitude_count != 1U ||
        k.sync_error_abs_ns_max < 3000U) {
        std::cerr << "sync KPI mismatch\n";
        return 1;
    }
    a.publishTrackerWatchdog(6U, true);
    a.publishMapWatchdog(32U, true);
    if (block == nullptr) {
        std::cerr << "shared state block missing\n";
        return 1;
    }
    if (block->watchdog.tracker_reinit_total.load(std::memory_order_acquire) != 1U ||
        block->watchdog.tracker_failure_streak.load(std::memory_order_acquire) != 6U ||
        block->watchdog.alert_tracker_failure.load(std::memory_order_acquire) != 1U ||
        block->watchdog.map_pause_streak.load(std::memory_order_acquire) != 32U ||
        block->watchdog.map_instability_total.load(std::memory_order_acquire) != 1U ||
        block->watchdog.alert_map_instability.load(std::memory_order_acquire) != 1U) {
        std::cerr << "watchdog publish mismatch\n";
        return 1;
    }
    a.markReady(bev::ipc::RoleId::Camera, "camera");
    a.updateHeartbeat(bev::ipc::RoleId::Camera, 123, 7, 2);
    block = b.block();
    if (block == nullptr || block->roles[0].ready.load(std::memory_order_acquire) != 1U) {
        std::cerr << "role ready not visible\n";
        return 1;
    }
    a.resetRoleHealthForStartup();
    if (block->roles[0].ready.load(std::memory_order_acquire) != 0U ||
        block->roles[0].heartbeat_ns.load(std::memory_order_acquire) != 0U ||
        block->roles[0].sequence_id.load(std::memory_order_acquire) != 0U ||
        block->roles[0].dropped.load(std::memory_order_acquire) != 0U) {
        std::cerr << "role health reset not applied\n";
        return 1;
    }
    const auto k2 = b.syncKpiSnapshot();
    if (k2.sync_samples != 0U || k2.interpolated_samples != 0U || k2.stale_attitude_count != 0U) {
        std::cerr << "sync KPI reset not applied\n";
        return 1;
    }
    if (block->watchdog.tracker_reinit_total.load(std::memory_order_acquire) != 0U ||
        block->watchdog.map_instability_total.load(std::memory_order_acquire) != 0U ||
        block->watchdog.alert_tracker_failure.load(std::memory_order_acquire) != 0U ||
        block->watchdog.alert_map_instability.load(std::memory_order_acquire) != 0U) {
        std::cerr << "watchdog reset not applied\n";
        return 1;
    }
    return 0;
#else
    bev::ipc::SharedState a;
    if (a.create("/bev_test_shared_state_nonlinux", err)) {
        std::cerr << "non-linux create should fail\n";
        return 1;
    }
    return 0;
#endif
}
