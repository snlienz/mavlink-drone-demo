# part3_failsafe.py flow

Single-loop state machine: mission flying and battery monitoring share one
MAVLink connection and interleave on every received message, rather than
running on separate threads.

```mermaid
flowchart TD
    Start([start]) --> Connect["Drone(conn_string)\nwait_heartbeat + request_data_stream"]
    Connect -->|no heartbeat in 30s| Err
    Connect --> Ready["wait_until_ready() -> set_mode(GUIDED) -> arm() -> takeoff(15m)"]
    Ready -->|any step fails| Err
    Ready --> Home["current_position()\ncompute 4 corner lat/lon"]

    Home --> Loop["square-flight loop, one iteration per received message"]

    Loop --> Inject{elapsed >= 20s\nand not yet injected?}
    Inject -->|yes| SetParam["set_param(SIM_BATT_VOLTAGE, 10.5)\nconfirm via PARAM_VALUE"]
    SetParam --> Poll
    Inject -->|no| Poll["poll next GLOBAL_POSITION_INT or SYS_STATUS\n(resend goto to current corner every 1s)"]

    Poll --> MsgType{message type?}

    MsgType -->|SYS_STATUS| VoltCheck{voltage < 11.0V?}
    VoltCheck -->|no| Loop
    VoltCheck -->|yes| Abort["log abort reason + timestamp\nset_mode(RTL)\nstop sending goto"]
    Abort --> Rtl

    MsgType -->|GLOBAL_POSITION_INT| CornerCheck{within 2m of\ncurrent corner?}
    CornerCheck -->|no| Loop
    CornerCheck -->|yes, more corners left| NextCorner["advance to next corner"]
    NextCorner --> Loop
    CornerCheck -->|yes, last corner| Complete["square complete,\nno failsafe triggered"]
    Complete --> Rtl["rtl_and_wait()\nset_mode(RTL) if not already,\npoll HEARTBEAT + GLOBAL_POSITION_INT\nuntil disarmed"]

    Rtl -->|timeout| Err
    Rtl --> Done["print Landed and disarmed.\nprint mission outcome"]

    Done --> Success(["return 0, exit success"])
    Err["catch DroneError, print error to stderr"] --> Fail(["return 1, exit non-zero"])
```
