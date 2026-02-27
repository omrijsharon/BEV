#pragma once

#include <string>

namespace bev::ipc {

struct PipelineNames {
    std::string pool_cam;
    std::string pool_bev;
    std::string pool_bev_web;
    std::string pool_map;
    std::string q_cam_to_bev;
    std::string q_fc_to_bev;
    std::string q_bev_to_track;
    std::string q_track_to_map;
    std::string q_track_to_web;
    std::string q_map_to_web;
    std::string shared_state;
    std::string control_socket;
};

inline PipelineNames makePipelineNames(const std::string& ns) {
    PipelineNames n{};
    n.pool_cam = ns + "_pool_cam";
    n.pool_bev = ns + "_pool_bev";
    n.pool_bev_web = ns + "_pool_bev_web";
    n.pool_map = ns + "_pool_map";
    n.q_cam_to_bev = ns + "_q_cam_to_bev";
    n.q_fc_to_bev = ns + "_q_fc_to_bev";
    n.q_bev_to_track = ns + "_q_bev_to_track";
    n.q_track_to_map = ns + "_q_track_to_map";
    n.q_track_to_web = ns + "_q_track_to_web";
    n.q_map_to_web = ns + "_q_map_to_web";
    n.shared_state = ns + "_shared_state";
    std::string sock_ns = ns;
    for (char& c : sock_ns) {
        if (c == '/') {
            c = '_';
        }
    }
    if (!sock_ns.empty() && sock_ns.front() == '_') {
        sock_ns.erase(sock_ns.begin());
    }
    n.control_socket = "/tmp/" + sock_ns + "_control.sock";
    return n;
}

}  // namespace bev::ipc
