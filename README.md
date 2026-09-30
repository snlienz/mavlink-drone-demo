# mavlink-drone-demo

## Lessons Learned

### Part 1: Takeoff

The goal of Part 1 is to learn the basics of flying the drone. Before takeoff, the program waits for `EKF_STATUS_REPORT` to indicate that the GPS/EKF position estimate is ready. Two useful telemetry messages are `GLOBAL_POSITION_INT`, which provides position and altitude, and `HEARTBEAT`, which reports the vehicle's status and mode.

### Part 2: Square Flight

Part 2 builds on Part 1. To keep the example simple, the conversion between distance and latitude/longitude uses a local flat-Earth (equirectangular) approximation. This works for short distances, but Earth is not a perfect sphere. See [Movable Type Scripts: Calculate distance, bearing and more](https://www.movable-type.co.uk/scripts/latlong.html) for more background.

### Part 3: Failsafe

Part 3 handles abnormal system conditions. The low-battery example uses polling because the condition is not time-critical. More urgent hazards may need faster, event-driven handling; wind-related telemetry such as `WIND_COV` could help detect or assess those conditions.

