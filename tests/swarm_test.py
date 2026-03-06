#!/usr/bin/env python3
"""
MAVSDK swarm test for PX4 SIH SITL multi-vehicle.

Orchestrates N vehicles through: set spawn offsets, takeoff, fly to waypoint, land.
The test script handles all formation/grid logic externally -- the viewer just renders.

Prerequisites:
  - PX4 SIH instances running:
      cd /path/to/PX4-Autopilot
      make px4_sitl_sih
      ./Tools/simulation/sitl_multiple_run.sh 5 sihsim_quadx px4_sitl_sih

  - One mavsdk_server per vehicle (ports 50051+i, connecting to udpin://0.0.0.0:14540+i)

  - pip install mavsdk

  - mavsim-viewer running:
      ./build/mavsim-viewer -n 5

Usage:
  python tests/swarm_test.py --n 5

Verification:
  1. Build PX4:    cd PX4-Autopilot && make px4_sitl_sih
  2. Launch swarm:  ./Tools/simulation/sitl_multiple_run.sh 5 sihsim_quadx px4_sitl_sih
  3. Launch viewer: ./build/mavsim-viewer -n 5
  4. Run test:      python tests/swarm_test.py --n 5
  5. Expected:
     - All 5 vehicles take off simultaneously in tight line formation (2m spacing)
     - Fly North 20m together maintaining formation
     - Hold 5 seconds
     - Return to spawn positions
     - Land all together
     - Viewer shows all 5 vehicles, TAB cycles between them, HUD updates per selection
"""

import argparse
import asyncio
import sys

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

BASE_LAT = 47.397742
BASE_LON = 8.545594
METERS_PER_DEG_LON = 75500.0  # at lat ~47.4


def get_spawn_lon(index, n_vehicles, spacing_m):
    """Calculate longitude offset for line formation along East axis."""
    offset_m = (index - (n_vehicles - 1) / 2.0) * spacing_m
    return BASE_LON + offset_m / METERS_PER_DEG_LON


async def set_spawn_offset(drone, index, n_vehicles, spacing_m):
    """Set SIH spawn position parameters for line formation."""
    spawn_lon = get_spawn_lon(index, n_vehicles, spacing_m)
    print(f"[Vehicle {index}] Setting spawn: lat={BASE_LAT:.6f} lon={spawn_lon:.6f}")
    await drone.param.set_param_float("SIH_LOC_LAT0", BASE_LAT)
    await drone.param.set_param_float("SIH_LOC_LON0", spawn_lon)


async def connect_drone(index, grpc_port, udp_port):
    """Connect to a single PX4 instance via MAVSDK."""
    drone = System(port=grpc_port)
    address = f"udpin://0.0.0.0:{udp_port}"
    print(f"[Vehicle {index}] Connecting on {address} (gRPC:{grpc_port})...")
    await drone.connect(system_address=address)

    # Wait for connection with timeout
    async def wait_connected():
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"[Vehicle {index}] Connected")
                return
    await asyncio.wait_for(wait_connected(), timeout=30)

    return drone


async def wait_for_gps(drone, index):
    """Wait until the vehicle has a GPS fix."""
    print(f"[Vehicle {index}] Waiting for GPS fix...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"[Vehicle {index}] GPS fix acquired")
            return
    raise RuntimeError(f"Vehicle {index}: GPS fix timeout")



async def arm_and_takeoff(drone, index, altitude, max_retries=10):
    """Arm and take off to specified altitude, with retry on COMMAND_DENIED."""
    await drone.action.set_takeoff_altitude(altitude)

    for attempt in range(max_retries):
        try:
            print(f"[Vehicle {index}] Arming (attempt {attempt + 1})...")
            await drone.action.arm()
            print(f"[Vehicle {index}] Armed, taking off to {altitude}m...")
            await drone.action.takeoff()
            break
        except Exception as e:
            if "COMMAND_DENIED" in str(e) and attempt < max_retries - 1:
                print(f"[Vehicle {index}] Arm denied, waiting for estimator... ({e})")
                await asyncio.sleep(1)
            else:
                raise

    # Wait until near target altitude (with timeout)
    async def wait_altitude():
        async for position in drone.telemetry.position():
            if position.relative_altitude_m >= altitude * 0.9:
                print(f"[Vehicle {index}] Reached {position.relative_altitude_m:.1f}m")
                return
    try:
        await asyncio.wait_for(wait_altitude(), timeout=30)
    except asyncio.TimeoutError:
        print(f"[Vehicle {index}] WARNING: altitude wait timed out, continuing...")


async def start_offboard(drone, index, initial_ned):
    """Start offboard mode with initial setpoint."""
    await drone.offboard.set_position_ned(initial_ned)
    try:
        await drone.offboard.start()
        print(f"[Vehicle {index}] Offboard mode started")
    except OffboardError as e:
        print(f"[Vehicle {index}] Offboard start failed: {e}")
        raise


async def fly_to_ned(drone, index, position_ned, tolerance=1.0):
    """Fly to NED position and wait until reached."""
    await drone.offboard.set_position_ned(position_ned)
    print(f"[Vehicle {index}] Flying to N={position_ned.north_m:.1f} "
          f"E={position_ned.east_m:.1f} D={position_ned.down_m:.1f}")

    # Wait until close to target (with timeout)
    async def wait_position():
        async for odom in drone.telemetry.position_velocity_ned():
            pos = odom.position
            dn = pos.north_m - position_ned.north_m
            de = pos.east_m - position_ned.east_m
            dd = pos.down_m - position_ned.down_m
            dist = (dn**2 + de**2 + dd**2) ** 0.5
            if dist < tolerance:
                print(f"[Vehicle {index}] Reached target (dist={dist:.2f}m)")
                return
    try:
        await asyncio.wait_for(wait_position(), timeout=30)
    except asyncio.TimeoutError:
        print(f"[Vehicle {index}] WARNING: position wait timed out, continuing...")


async def land_drone(drone, index):
    """Switch to land mode."""
    print(f"[Vehicle {index}] Landing...")
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass
    await drone.action.land()

    # Wait for disarm (indicates touchdown)
    async def wait_disarm():
        async for armed in drone.telemetry.armed():
            if not armed:
                print(f"[Vehicle {index}] Landed and disarmed")
                return
    try:
        await asyncio.wait_for(wait_disarm(), timeout=30)
    except asyncio.TimeoutError:
        print(f"[Vehicle {index}] WARNING: disarm wait timed out")


async def run_swarm_mission(n_vehicles, spacing, altitude, base_udp_port, grpc_base,
                            speed=1.0):
    """Run the complete swarm mission."""
    # Scale sleep durations by speed factor (faster sim = shorter waits)
    def sim_sleep(real_seconds):
        return asyncio.sleep(real_seconds / speed)

    # 1. Connect to all vehicles (sequential to avoid mavsdk_server port contention)
    print(f"\n=== Connecting to {n_vehicles} vehicles (speed={speed}x) ===")
    drones = []
    for i in range(n_vehicles):
        drone = await connect_drone(i, grpc_base + i, base_udp_port + i)
        drones.append(drone)

    # 2. Wait for GPS on all
    print("\n=== Waiting for GPS ===")
    await asyncio.gather(*[
        asyncio.wait_for(wait_for_gps(d, i), timeout=60)
        for i, d in enumerate(drones)
    ])

    # 3. Set spawn offsets for line formation
    print("\n=== Setting spawn positions ===")
    for i, d in enumerate(drones):
        await set_spawn_offset(d, i, n_vehicles, spacing)

    # Wait for SIH to apply new positions and estimator to reinitialize
    # Note: do NOT scale this by speed — estimator needs real wall-clock time
    print("\n=== Waiting for estimator to reinitialize ===")
    await asyncio.sleep(5)
    await asyncio.gather(*[
        asyncio.wait_for(wait_for_gps(d, i), timeout=60)
        for i, d in enumerate(drones)
    ])

    # Compute East offsets so vehicles maintain formation spacing in offboard mode
    # (offboard NED is relative to home, which doesn't update with SIH spawn params)
    east_offsets = [(i - (n_vehicles - 1) / 2.0) * spacing for i in range(n_vehicles)]

    # 4. Arm and takeoff (parallel)
    print("\n=== Arming and taking off ===")
    await asyncio.gather(*[
        arm_and_takeoff(d, i, altitude)
        for i, d in enumerate(drones)
    ])

    # 5. Start offboard mode on all vehicles
    print("\n=== Starting offboard mode ===")
    await asyncio.gather(*[
        start_offboard(d, i, PositionNedYaw(0.0, east_offsets[i], -altitude, 0.0))
        for i, d in enumerate(drones)
    ])

    # 6. Fly 20m North maintaining formation
    print("\n=== Flying 20m North ===")
    await asyncio.gather(*[
        fly_to_ned(d, i, PositionNedYaw(20.0, east_offsets[i], -altitude, 0.0))
        for i, d in enumerate(drones)
    ])

    # 7. Hold position
    print("\n=== Holding position for 5s ===")
    await sim_sleep(5)

    # 8. Return to above spawn positions
    print("\n=== Returning to spawn positions ===")
    await asyncio.gather(*[
        fly_to_ned(d, i, PositionNedYaw(0.0, east_offsets[i], -altitude, 0.0))
        for i, d in enumerate(drones)
    ])

    # 9. Land all together
    print("\n=== Landing ===")
    await asyncio.gather(*[
        land_drone(d, i)
        for i, d in enumerate(drones)
    ])

    print("\n=== Mission complete ===")


async def main():
    parser = argparse.ArgumentParser(description="MAVSDK swarm test for PX4 SIH")
    parser.add_argument("--n", type=int, default=5, help="Number of vehicles (default: 5)")
    parser.add_argument("--spacing", type=float, default=2.0,
                        help="Formation spacing in meters (default: 2.0)")
    parser.add_argument("--altitude", type=float, default=10.0,
                        help="Takeoff altitude AGL in meters (default: 10.0)")
    parser.add_argument("--base-port", type=int, default=14540,
                        help="PX4 MAVLink base UDP port (default: 14540)")
    parser.add_argument("--grpc-base", type=int, default=50051,
                        help="MAVSDK gRPC base port (default: 50051)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="PX4 sim speed factor (default: 1.0, use 10 for 10x)")
    args = parser.parse_args()

    try:
        await run_swarm_mission(args.n, args.spacing, args.altitude,
                                args.base_port, args.grpc_base, args.speed)
    except Exception as e:
        print(f"\nMission failed: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
