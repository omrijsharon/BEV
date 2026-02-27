#include "ipc/descriptors.hpp"

#include <iostream>

int main() {
    const auto bev = bev::ipc::makeBevFrameDesc();
    const auto frame = bev::ipc::makeFrameDesc();
    const auto att = bev::ipc::makeAttitudeDesc();
    const auto track = bev::ipc::makeTrackResultDesc();
    const auto map = bev::ipc::makeMapFrameDesc();

    if (bev.protocol_version != bev::ipc::kIpcProtocolVersion ||
        frame.protocol_version != bev::ipc::kIpcProtocolVersion ||
        att.protocol_version != bev::ipc::kIpcProtocolVersion ||
        track.protocol_version != bev::ipc::kIpcProtocolVersion ||
        map.protocol_version != bev::ipc::kIpcProtocolVersion) {
        std::cerr << "descriptor protocol version mismatch\n";
        return 1;
    }

    if (sizeof(bev::ipc::BevFrameDesc) != 52 ||
        sizeof(bev::ipc::FrameDesc) != 52 ||
        sizeof(bev::ipc::AttitudeDesc) != 52 ||
        sizeof(bev::ipc::TrackResultDesc) != 64 ||
        sizeof(bev::ipc::MapFrameDesc) != 64) {
        std::cerr << "descriptor ABI size mismatch\n";
        return 1;
    }

    return 0;
}
