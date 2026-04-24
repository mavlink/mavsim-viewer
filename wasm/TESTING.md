# WASM Replay Testing Matrix

One log per position resolution tier from `../logs/`. Tests every branch of the fallback chain documented in `POSITION_RESOLUTION_TEST_CHECKLIST.md`. The WASM extractor and replay must handle all of these identically to native.

## Position resolution tiers

| # | Category | Log file | Expected behavior |
|---|----------|----------|-------------------|
| 1 | **C01: Home valid** | `../logs/C01_HOME_POS_VALID/0192e367-56ff-4828-9831-1d28b2be0b9e.ulg` | `home=valid` in console. Origin set from first state after home arrives. Drone flies normally. Best case. |
| 2 | **C02: Home zeros, GPOS valid** | `../logs/C02_HOME_POS_ZEROS__GPOS_VALID/2b12d921-2048-4365-a2a2-cb95193adfe7.ulg` | `home=valid` from first GPOS (fallback). Drone positions from global position data. |
| 3 | **C02: Home zeros, LPOS ref valid** | `../logs/C02_HOME_POS_ZEROS__LPOS_REF_VALID/8a612aef-d2ba-4249-b710-261c737ea56c.ulg` | Origin derived from LPOS reference point (`ref_lat/ref_lon`). Drone positions via local-to-global conversion. |
| 4 | **C02: Home zeros, LPOS no global** | `../logs/C02_HOME_POS_ZEROS__LPOS_NO_GLOBAL/272adb09-4f3a-4ed8-9025-24c65aad1b99.ulg` | No global reference. Drone renders at local NED coordinates relative to origin. May sit near world origin. |
| 5 | **C02: Home zeros, LPOS ref zeros** | `../logs/C02_HOME_POS_ZEROS__LPOS_REF_ZEROS/f69c1496-c3f8-4d2c-9933-9c2c0cac14d9.ulg` | LPOS ref exists but is zero. Should fall back to origin from first nonzero position after ~20 frames. |
| 6 | **C03: No home, GPOS valid** | `../logs/C03_NO_HOME__GPOS_VALID/31409991-e4e0-4c90-a44e-2f35648e38ac.ulg` | `home=none` in console. Origin auto-resolves from first GPOS sample in `vehicle_update`. Drone flies normally. |
| 7 | **C05: No home, no GPOS, LPOS ref valid** | `../logs/C05_NO_HOME__NO_GPOS__LPOS_REF_VALID/3ceed808-83eb-47c1-a21c-2c08fb1ecb7e.ulg` | Position derived entirely from LPOS + reference point. No GPS. Drone should still move. |
| 8 | **C07: No home, no GPOS, LPOS no global** | `../logs/C07_NO_HOME__NO_GPOS__LPOS_NO_GLOBAL/05f68e25-b07d-4967-93a0-40a7aa496d19.ulg` | Worst viable position tier. Drone at local NED coords, no global reference. May appear at origin with local motion only. |
| 9 | **C09: No position data** | `../logs/C09_NO_HOME__NO_GPOS__NO_LPOS/66188e38-448c-442f-ac87-81e11b023a5c.ulg` | Attitude only, no position. Drone stays at origin, rotates in place. `state.valid` may never become true. Console should show `0 lpos, 0 gpos`. |
| 10 | **C10: No attitude** | `../logs/C10_NO_ATTITUDE/2c458da4-2e39-49d9-8526-79388b388930.ulg` | No vehicle_attitude topic. Extractor should report `0 att`. Drone may not render or may sit frozen. Native requires attitude to load; WASM extractor should handle gracefully (no crash). |

## Playback controls

Run these on any C01 log (known-good positioning):

| # | Test | Steps | Pass criteria |
|---|------|-------|---------------|
| 11 | **Play/Pause** | Load log, click Pause, wait 3s, click Play | Drone stops on Pause, resumes on Play. No crash. |
| 12 | **Seek** | Load log, drag slider to ~50%, release | Drone jumps to mid-flight position. Playback continues from there. No crash or freeze. |
| 13 | **Reload** | Load one log, then load a different log | First log's trail clears. New log plays. No crash, no leftover state from first log. |
| 14 | **Loop** | Load log, let it play to the end | Playback restarts from the beginning. Drone returns to start position. |

## Event extraction

Check the browser console `WASM replay:` line for each log:

| # | Log category | Expected counts |
|---|-------------|-----------------|
| 15 | C01 | Nonzero att, nonzero lpos, likely nonzero gpos |
| 16 | C03 | Nonzero att, likely nonzero lpos, nonzero gpos |
| 17 | C07 | Nonzero att, nonzero lpos, 0 gpos |
| 18 | C09 | 0 att OR very few, 0 lpos, 0 gpos |
| 19 | C10 | 0 att, may have lpos/gpos but no attitude to display |

## How to test

1. Start the dev server: `cd wasm/build && python3 -m http.server 8000`
2. Open `http://localhost:8000/`
3. For each row, use Choose File to load the log
4. Watch for ~10 seconds, check console output
5. Record: pass / fail / notes

Quick format for reporting back:
```
1: pass
2: pass
3: drone stuck at origin
4: pass
...
```
