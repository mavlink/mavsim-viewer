cmake_minimum_required(VERSION 3.14)
project(hawkeye C)

set(CMAKE_C_STANDARD 11)

option(BUILD_TESTING "Build tests" OFF)

# MAVLink c_library_v2 (header-only, submodule)
set(MAVLINK_DIR ${CMAKE_SOURCE_DIR}/lib/c_library_v2)

# Core library: ULog parser + replay (no raylib dependency)
add_library(hawkeye-core STATIC
    src/ulog_parser.c
    src/ulog_replay.c
    src/data_source_ulog.c
)
target_include_directories(hawkeye-core PUBLIC src/)
target_include_directories(hawkeye-core PRIVATE
    ${MAVLINK_DIR}
    ${MAVLINK_DIR}/common
)
target_compile_definitions(hawkeye-core PRIVATE MAVLINK_COMM_NUM_BUFFERS=16)
if(UNIX AND NOT APPLE)
    target_link_libraries(hawkeye-core PRIVATE m)
endif()

# Raylib via FetchContent (skip when only building tests)
if(NOT BUILD_TESTING_ONLY)
    include(FetchContent)
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG        5.5
        GIT_SHALLOW    TRUE
    )
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(raylib)

    add_executable(hawkeye
        src/main.c
        src/mavlink_receiver.c
        src/data_source_mavlink.c
        src/vehicle.c
        src/scene.c
        src/hud.c
        src/hud_help.c
        src/hud_instruments.c
        src/hud_transport.c
        src/hud_telemetry.c
        src/replay_conflict.c
        src/replay_trail.c
        src/replay_markers.c
        src/ui_marker_input.c
        src/debug_panel.c
        src/ortho_panel.c
        src/asset_path.c
        src/theme.c
        src/tactical_hud.c
    )

    target_include_directories(hawkeye PRIVATE
        ${MAVLINK_DIR}
        ${MAVLINK_DIR}/common
    )

    # Need 16 MAVLink parse channels for multi-vehicle support
    target_compile_definitions(hawkeye PRIVATE MAVLINK_COMM_NUM_BUFFERS=16)

    target_link_libraries(hawkeye PRIVATE raylib hawkeye-core)

    if(APPLE)
        target_link_libraries(hawkeye PRIVATE
            "-framework IOKit"
            "-framework Cocoa"
            "-framework OpenGL"
        )
    elseif(WIN32)
        target_link_libraries(hawkeye PRIVATE ws2_32)
    elseif(UNIX)
        target_link_libraries(hawkeye PRIVATE m pthread dl)
    endif()

    # Copy resources to build directory
    add_custom_command(TARGET hawkeye POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/models $<TARGET_FILE_DIR:hawkeye>/models
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/shaders $<TARGET_FILE_DIR:hawkeye>/shaders
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/fonts $<TARGET_FILE_DIR:hawkeye>/fonts
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/textures $<TARGET_FILE_DIR:hawkeye>/textures
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/themes $<TARGET_FILE_DIR:hawkeye>/themes
    )

    # Install rules
    include(GNUInstallDirs)

    target_compile_definitions(hawkeye PRIVATE
        HAWKEYE_INSTALL_DATADIR="${CMAKE_INSTALL_FULL_DATADIR}/hawkeye")

    install(TARGETS hawkeye RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(DIRECTORY models/ DESTINATION ${CMAKE_INSTALL_DATADIR}/hawkeye/models
            FILES_MATCHING PATTERN "*.obj" PATTERN "*.mtl")
    install(DIRECTORY shaders/ DESTINATION ${CMAKE_INSTALL_DATADIR}/hawkeye/shaders
            FILES_MATCHING PATTERN "*.vs" PATTERN "*.fs")
    install(DIRECTORY fonts/ DESTINATION ${CMAKE_INSTALL_DATADIR}/hawkeye/fonts
            FILES_MATCHING PATTERN "*.ttf")
    install(DIRECTORY textures/ DESTINATION ${CMAKE_INSTALL_DATADIR}/hawkeye/textures
            FILES_MATCHING PATTERN "*.png")
    install(DIRECTORY themes/ DESTINATION ${CMAKE_INSTALL_DATADIR}/hawkeye/themes
            FILES_MATCHING PATTERN "*.mvt")
    install(FILES fonts/OFL.txt DESTINATION ${CMAKE_INSTALL_DATADIR}/hawkeye/fonts)

    # CPack packaging
    set(CPACK_PACKAGE_NAME "hawkeye")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Real-time 3D flight visualizer for PX4 with ULog replay and multi-drone analysis")
    set(CPACK_PACKAGE_CONTACT "https://github.com/PX4/Hawkeye")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "libgl1, libx11-6, libxrandr2, libxinerama1, libxcursor1, libxi6")
    set(CPACK_DEBIAN_PACKAGE_SECTION "science")
    set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
    if(DEFINED HAWKEYE_VERSION)
        set(CPACK_PACKAGE_VERSION "${HAWKEYE_VERSION}")
    endif()
    include(CPack)
endif()

if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
