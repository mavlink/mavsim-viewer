# Your first replay

For users with an existing PX4 ULog (`.ulg`) file.
No PX4 source tree required.

If you haven't installed Hawkeye yet, see [Installation](./installation.md) first.

```sh
hawkeye --replay path/to/flight.ulg
```

Hawkeye opens, pre-scans the log for vehicle type and flight mode transitions, prints a summary to the console, and starts playing the flight automatically:

```sh
ULog replay: flight.ulg (287.3s, 289 index entries)
  Flight modes: Takeoff@12s Mission@15s RTL@250s Land@276s
  Takeoff: 12.3s (CUSUM conf=92%)
```

Use the transport keys to navigate:

| Key                   | Action                             |
| --------------------- | ---------------------------------- |
| `Space`               | Pause / resume                     |
| `+` / `-`             | Increase / decrease playback speed |
| `←` / `→`             | Seek 5 seconds                     |
| `Shift+←` / `Shift+→` | Frame step (20 ms)                 |
| `R`                   | Restart from beginning             |

See [ULog Replay](./replay.md) for the full list of transport controls, marker keybinds, and analysis features.

## Next steps

- [ULog Replay](./replay.md) — Markers, interpolation, seek, and frame-by-frame inspection
- [First swarm](./first-swarm.md) — Load multiple logs together
- [Command-Line Reference](./cli.md) — Every CLI flag with examples
