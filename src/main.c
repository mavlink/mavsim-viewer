#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "mavlink_receiver.h"
#include "vehicle.h"
#include "scene.h"
#include "hud.h"

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  -udp <port>    UDP listen port (default: 19410)\n");
    printf("  -mc            Multicopter model (default)\n");
    printf("  -fw            Fixed-wing model\n");
    printf("  -ts            Tailsitter model\n");
    printf("  -w <width>     Window width (default: 1280)\n");
    printf("  -h <height>    Window height (default: 720)\n");
}

int main(int argc, char *argv[]) {
    uint16_t port = 19410;
    vehicle_type_t vtype = VEHICLE_MULTICOPTER;
    int win_w = 1280;
    int win_h = 720;
    bool debug = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-udp") == 0 && i + 1 < argc) {
            port = (uint16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-mc") == 0) {
            vtype = VEHICLE_MULTICOPTER;
        } else if (strcmp(argv[i], "-fw") == 0) {
            vtype = VEHICLE_FIXEDWING;
        } else if (strcmp(argv[i], "-ts") == 0) {
            vtype = VEHICLE_TAILSITTER;
        } else if (strcmp(argv[i], "-d") == 0) {
            debug = true;
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            win_w = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
            win_h = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    // Init Raylib
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(win_w, win_h, "MAVSim Viewer");
    SetTargetFPS(60);

    // Init MAVLink receiver
    mavlink_receiver_t recv;
    recv.debug = debug;
    if (mavlink_receiver_init(&recv, port) != 0) {
        fprintf(stderr, "Failed to init MAVLink receiver on port %u\n", port);
        CloseWindow();
        return 1;
    }

    // Init vehicle and scene
    vehicle_t vehicle;
    vehicle_init(&vehicle, vtype);

    scene_t scene;
    scene_init(&scene);

    hud_t hud;
    hud_init(&hud);

    // Main loop
    while (!WindowShouldClose()) {
        // Poll MAVLink
        mavlink_receiver_poll(&recv);

        // Update vehicle from latest MAVLink state
        vehicle_update(&vehicle, &recv.state);

        // Update HUD timer
        hud_update(&hud, GetFrameTime(), recv.connected);

        // Handle input
        scene_handle_input(&scene);

        // Update camera
        scene_update_camera(&scene, vehicle.position, vehicle.rotation);

        // Render
        BeginDrawing();

            // Sky background
            scene_draw_sky(&scene);

            BeginMode3D(scene.camera);
                scene_draw(&scene);
                vehicle_draw(&vehicle, scene.view_mode);
            EndMode3D();

            // HUD
            hud_draw(&hud, &vehicle, recv.connected, GetScreenWidth(), GetScreenHeight());

        EndDrawing();
    }

    // Cleanup
    hud_cleanup(&hud);
    vehicle_cleanup(&vehicle);
    scene_cleanup(&scene);
    mavlink_receiver_close(&recv);
    CloseWindow();

    return 0;
}
