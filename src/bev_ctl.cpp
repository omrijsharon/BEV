#include "core/config.hpp"
#include "ipc/control_plane.hpp"
#include "ipc/pipeline_names.hpp"
#include "ipc/shared_state.hpp"

#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: bev_ctl <command> [config_path]\n";
        return 1;
    }

    const std::string command = argv[1];
    const std::string config_path = (argc > 2) ? argv[2] : "config/config.yaml";

    bev::AppConfig cfg;
    std::string err;
    if (!bev::loadConfig(config_path, cfg, err)) {
        std::cerr << "config load failed: " << err << "\n";
        return 1;
    }

    const auto names = bev::ipc::makePipelineNames(cfg.ipc.shm_namespace);
    std::string response;
    if (!bev::ipc::unixControlRequest(names.control_socket, command + "\n", response, err)) {
        if (command == "health") {
            bev::ipc::SharedState shared;
            if (shared.attach(names.shared_state, err)) {
                std::ostringstream oss;
                oss << "OK health (fallback-shm)\n";
                const auto* block = shared.block();
                if (block != nullptr) {
                    oss << "pause_map=" << block->pause_map_updates.load(std::memory_order_acquire)
                        << " shutdown=" << block->shutdown_requested.load(std::memory_order_acquire)
                        << " reload_generation=" << block->reload_generation.load(std::memory_order_acquire)
                        << " protocol_mismatch=" << block->protocol_mismatch.load(std::memory_order_acquire)
                        << "\n";
                    const auto k = shared.syncKpiSnapshot();
                    const double interp_ratio = (k.sync_samples > 0)
                        ? static_cast<double>(k.interpolated_samples) / static_cast<double>(k.sync_samples)
                        : 0.0;
                    const double mean_abs_error_us = (k.sync_samples > 0)
                        ? (static_cast<double>(k.sync_error_abs_ns_accum) / static_cast<double>(k.sync_samples)) / 1000.0
                        : 0.0;
                    oss << "sync_kpi samples=" << k.sync_samples
                        << " stale=" << k.stale_attitude_count
                        << " interp_ratio=" << interp_ratio
                        << " mean_abs_error_us=" << mean_abs_error_us
                        << " max_abs_error_us=" << (static_cast<double>(k.sync_error_abs_ns_max) / 1000.0)
                        << "\n";
                    for (uint32_t i = 0; i < static_cast<uint32_t>(bev::ipc::RoleId::Count); ++i) {
                        const auto& role = block->roles[i];
                        const char* fallback = (i == 0U) ? "camera" :
                                               (i == 1U) ? "fc" :
                                               (i == 2U) ? "bev" :
                                               (i == 3U) ? "track" :
                                               (i == 4U) ? "map" : "web";
                        const char* role_name = (role.name[0] != '\0') ? role.name : fallback;
                        oss << role_name
                            << " ready=" << role.ready.load(std::memory_order_acquire)
                            << " heartbeat_ns=" << role.heartbeat_ns.load(std::memory_order_acquire)
                            << " seq=" << role.sequence_id.load(std::memory_order_acquire)
                            << " dropped=" << role.dropped.load(std::memory_order_acquire)
                            << "\n";
                        const auto& stage = block->stage_kpi[i];
                        oss << "stage_" << role_name
                            << " samples=" << stage.samples.load(std::memory_order_acquire)
                            << " latency_last_us=" << (stage.latency_last_ns.load(std::memory_order_acquire) / 1000U)
                            << " latency_mean_us=" << (stage.latency_mean_ns.load(std::memory_order_acquire) / 1000U)
                            << " latency_max_us=" << (stage.latency_max_ns.load(std::memory_order_acquire) / 1000U)
                            << " queue_depth=" << stage.queue_depth.load(std::memory_order_acquire)
                            << " dropped=" << stage.dropped.load(std::memory_order_acquire)
                            << "\n";
                    }
                    oss << "snapshot_camera valid=" << block->latest_camera.valid.load(std::memory_order_acquire)
                        << " seq=" << block->latest_camera.sequence_id.load(std::memory_order_acquire)
                        << " ts_ns=" << block->latest_camera.timestamp_ns.load(std::memory_order_acquire)
                        << " slot=" << block->latest_camera.frame_slot_id.load(std::memory_order_acquire)
                        << " gen=" << block->latest_camera.generation.load(std::memory_order_acquire)
                        << "\n";
                    oss << "snapshot_fc valid=" << block->latest_fc.valid.load(std::memory_order_acquire)
                        << " sample_id=" << block->latest_fc.sample_id.load(std::memory_order_acquire)
                        << " ts_ns=" << block->latest_fc.timestamp_ns.load(std::memory_order_acquire)
                        << "\n";
                    oss << "snapshot_bev valid=" << block->latest_bev.valid.load(std::memory_order_acquire)
                        << " seq=" << block->latest_bev.sequence_id.load(std::memory_order_acquire)
                        << " ts_ns=" << block->latest_bev.timestamp_ns.load(std::memory_order_acquire)
                        << " slot=" << block->latest_bev.frame_slot_id.load(std::memory_order_acquire)
                        << " gen=" << block->latest_bev.generation.load(std::memory_order_acquire)
                        << "\n";
                    oss << "snapshot_map valid=" << block->latest_map.valid.load(std::memory_order_acquire)
                        << " seq=" << block->latest_map.sequence_id.load(std::memory_order_acquire)
                        << " ts_ns=" << block->latest_map.timestamp_ns.load(std::memory_order_acquire)
                        << " slot=" << block->latest_map.frame_slot_id.load(std::memory_order_acquire)
                        << " gen=" << block->latest_map.generation.load(std::memory_order_acquire)
                        << "\n";
                    oss << "thermal valid=" << block->thermal.valid.load(std::memory_order_acquire)
                        << " throttled=" << block->thermal.throttled.load(std::memory_order_acquire)
                        << " temp_mC=" << block->thermal.temp_milli_c.load(std::memory_order_acquire)
                        << " fps_cap=" << block->thermal.fps_cap.load(std::memory_order_acquire)
                        << "\n";
                    oss << "watchdog msp_stale_total=" << block->watchdog.msp_stale_total.load(std::memory_order_acquire)
                        << " msp_stale_streak=" << block->watchdog.msp_stale_streak.load(std::memory_order_acquire)
                        << " tracker_reinit_total=" << block->watchdog.tracker_reinit_total.load(std::memory_order_acquire)
                        << " tracker_failure_streak=" << block->watchdog.tracker_failure_streak.load(std::memory_order_acquire)
                        << " map_pause_streak=" << block->watchdog.map_pause_streak.load(std::memory_order_acquire)
                        << " map_instability_total=" << block->watchdog.map_instability_total.load(std::memory_order_acquire)
                        << " alert_msp_stale=" << block->watchdog.alert_msp_stale.load(std::memory_order_acquire)
                        << " alert_tracker_failure=" << block->watchdog.alert_tracker_failure.load(std::memory_order_acquire)
                        << " alert_map_instability=" << block->watchdog.alert_map_instability.load(std::memory_order_acquire)
                        << "\n";
                }
                std::cout << oss.str();
                return 0;
            }
        }
        std::cerr << "control request failed: " << err << "\n";
        return 1;
    }

    std::cout << response;
    return 0;
}
