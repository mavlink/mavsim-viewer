# Command-Line Reference (Hawkeye)

Every Hawkeye CLI flag with description and default value.

## Options

| Flag                             | Description                                                   | Default         |
| -------------------------------- | ------------------------------------------------------------- | --------------- |
| `-udp <port>`                    | UDP base port for MAVLink telemetry                           | `19410`         |
| `-n <count>`                     | Number of vehicles (max 16)                                   | `1`             |
| `-mc`                            | Default to multicopter model                                  | on              |
| `-fw`                            | Default to fixed-wing model                                   | off             |
| `-ts`                            | Default to tailsitter model                                   | off             |
| `-origin <lat> <lon> <alt>`      | NED origin in degrees (lat, lon) and meters (alt)             | PX4 SIH default |
| `-w <width>`                     | Window width in pixels                                        | `1280`          |
| `-h <height>`                    | Window height in pixels                                       | `720`           |
| `--replay <f1.ulg> [f2.ulg ...]` | Replay one or more ULog files                                 | (none)          |
| `--ghost <f1.ulg> [f2.ulg ...]`  | Replay with ghost overlay (non-primary drones at 35% opacity) | (none)          |
| `-d`                             | Enable debug output to stderr                                 | off             |

## Port allocation

In live MAVLink mode, each vehicle binds its own UDP socket at `base_port + N`:

| Vehicle    | Port (default base 19410) |
| ---------- | ------------------------- |
| Vehicle 1  | 19410                     |
| Vehicle 2  | 19411                     |
| Vehicle 3  | 19412                     |
| ...        | ...                       |
| Vehicle 16 | 19425                     |

Change the base port with `-udp`.
All subsequent vehicle ports increment from there.

PX4 SITL instances use the same convention when launched via `sitl_multiple_run.sh`, so PX4 instance N and Hawkeye vehicle N automatically match up.

## Usage examples

### Single vehicle, default SIH

```sh
hawkeye
```

Binds to 19410, default quadrotor model.

### Fixed-wing in a larger window

```sh
hawkeye -fw -w 1920 -h 1080
```

### Five-vehicle swarm on an alternate base port

```sh
hawkeye -n 5 -udp 14540
```

Binds vehicles 1–5 on 14540–14544.

### Custom origin

```sh
hawkeye -origin 47.641468 -122.140165 0
```

Sets the NED reference point (Microsoft campus, Redmond, WA).
Useful when PX4 SIH is configured to launch from a non-default location.

### Replay a single log

```sh
hawkeye --replay flight.ulg
```

### Multi-drone replay

```sh
hawkeye --replay drone1.ulg drone2.ulg drone3.ulg
```

Up to 16 files.
See [ULog Replay](replay.md) for details.

### Ghost overlay: compare two flights

```sh
hawkeye --ghost before.ulg after.ulg
```

First file is the primary drone (fully opaque). All other files render at 35% opacity with color tint. Useful for overlaying a baseline flight against a tuning experiment.
