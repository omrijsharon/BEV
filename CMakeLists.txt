cmake_minimum_required(VERSION 3.16)
project(BEV VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BEV_BUILD_TESTS "Build BEV tests" ON)
option(BEV_BUILD_ARDUINO_MSP "Build Arduino-dependent BetaflightMSP source" OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui calib3d)

add_library(bev_core
    src/core/config.cpp
    src/core/math_utils.cpp
    src/core/time_utils.cpp
    src/core/telemetry.cpp
    src/msp/msp_bridge.cpp
    src/msp/betaflight_msp_adapter.cpp
    src/camera/camera_calibration.cpp
    src/camera/camera_ingest.cpp
    src/bev/bev_warp.cpp
    src/tracking/keypoint_initializer.cpp
    src/tracking/track_manager.cpp
    src/tracking/template_tracker.cpp
    src/registration/registration_refiner.cpp
    src/motion/motion_detector.cpp
    src/map/minimap_builder.cpp
    src/web/mjpeg_server.cpp
    src/web/dashboard_renderer.cpp
)

target_include_directories(bev_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(bev_core PUBLIC ${OpenCV_LIBS})

if(BEV_BUILD_ARDUINO_MSP)
    add_library(betaflight_msp src/msp/BetaflightMSP.cpp)
    target_include_directories(betaflight_msp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/msp)
endif()

add_executable(bev_app src/main.cpp)
target_link_libraries(bev_app PRIVATE bev_core)

if(BEV_BUILD_TESTS)
    enable_testing()

    add_executable(test_math_utils tests/unit/test_math_utils.cpp)
    target_link_libraries(test_math_utils PRIVATE bev_core)
    add_test(NAME test_math_utils COMMAND test_math_utils)

    add_executable(test_config_validation tests/unit/test_config_validation.cpp)
    target_link_libraries(test_config_validation PRIVATE bev_core)
    add_test(NAME test_config_validation COMMAND test_config_validation)

    add_executable(test_tracker_coordinates tests/unit/test_tracker_coordinates.cpp)
    target_link_libraries(test_tracker_coordinates PRIVATE bev_core)
    add_test(NAME test_tracker_coordinates COMMAND test_tracker_coordinates)

    add_executable(test_motion_threshold tests/unit/test_motion_threshold.cpp)
    target_link_libraries(test_motion_threshold PRIVATE bev_core)
    add_test(NAME test_motion_threshold COMMAND test_motion_threshold)

    add_executable(test_msp_bridge tests/unit/test_msp_bridge.cpp)
    target_link_libraries(test_msp_bridge PRIVATE bev_core)
    add_test(NAME test_msp_bridge COMMAND test_msp_bridge)

    add_executable(test_bev_warp tests/unit/test_bev_warp.cpp)
    target_link_libraries(test_bev_warp PRIVATE bev_core)
    add_test(NAME test_bev_warp COMMAND test_bev_warp)

    add_executable(test_camera_calibration tests/unit/test_camera_calibration.cpp)
    target_link_libraries(test_camera_calibration PRIVATE bev_core)
    add_test(NAME test_camera_calibration COMMAND test_camera_calibration)

    add_executable(test_mjpeg_server tests/unit/test_mjpeg_server.cpp)
    target_link_libraries(test_mjpeg_server PRIVATE bev_core)
    add_test(NAME test_mjpeg_server COMMAND test_mjpeg_server)

    add_executable(test_mjpeg_server_lifecycle tests/unit/test_mjpeg_server_lifecycle.cpp)
    target_link_libraries(test_mjpeg_server_lifecycle PRIVATE bev_core)
    add_test(NAME test_mjpeg_server_lifecycle COMMAND test_mjpeg_server_lifecycle)

    add_executable(test_dashboard_renderer tests/unit/test_dashboard_renderer.cpp)
    target_link_libraries(test_dashboard_renderer PRIVATE bev_core)
    add_test(NAME test_dashboard_renderer COMMAND test_dashboard_renderer)

    add_executable(test_keypoint_initializer tests/unit/test_keypoint_initializer.cpp)
    target_link_libraries(test_keypoint_initializer PRIVATE bev_core)
    add_test(NAME test_keypoint_initializer COMMAND test_keypoint_initializer)

    add_executable(test_track_manager tests/unit/test_track_manager.cpp)
    target_link_libraries(test_track_manager PRIVATE bev_core)
    add_test(NAME test_track_manager COMMAND test_track_manager)

    add_executable(test_registration_refiner tests/unit/test_registration_refiner.cpp)
    target_link_libraries(test_registration_refiner PRIVATE bev_core)
    add_test(NAME test_registration_refiner COMMAND test_registration_refiner)

    add_executable(test_template_match tests/unit/test_template_match.cpp)
    target_link_libraries(test_template_match PRIVATE bev_core)
    add_test(NAME test_template_match COMMAND test_template_match)

    add_executable(test_minimap_builder tests/unit/test_minimap_builder.cpp)
    target_link_libraries(test_minimap_builder PRIVATE bev_core)
    add_test(NAME test_minimap_builder COMMAND test_minimap_builder)

    add_executable(test_sequence_registration tests/integration/test_sequence_registration.cpp)
    target_link_libraries(test_sequence_registration PRIVATE bev_core)
    add_test(NAME test_sequence_registration COMMAND test_sequence_registration)

    add_executable(test_motion_replay tests/integration/test_motion_replay.cpp)
    target_link_libraries(test_motion_replay PRIVATE bev_core)
    add_test(NAME test_motion_replay COMMAND test_motion_replay)

    add_executable(test_minimap_replay tests/integration/test_minimap_replay.cpp)
    target_link_libraries(test_minimap_replay PRIVATE bev_core)
    add_test(NAME test_minimap_replay COMMAND test_minimap_replay)
endif()
