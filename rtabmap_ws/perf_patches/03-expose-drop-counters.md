# Phase 12 -- expose the frame-drop counters (the missing P99 metric)

## Why this matters more than any latency number

Both inter-thread queues in RTAB-Map are **drop-oldest and never blocking**. A stage that
overruns its budget does not increase latency -- it silently throws frames away. So a
system dropping 40% of frames can still report a beautiful 20 ms mean.

Three independent drop points, all effectively silent today:

1. `rtabmap_odom/src/OdometryROS.cpp:461-483` -- try-lock handoff into the VO worker:
   ```cpp
   	else
   	{
   		//RCLCPP_WARN(get_logger(), "Dropping image/scan data");    // <-- COMMENTED OUT
   		++droppedMsgs_;
   	}
   ```
   `droppedMsgs_` and `processedMsgs_` are incremented but never published.

2. `corelib/src/OdometryThread.cpp:182-188` -- `Odom/ImageBufferSize` (default 1) overflow:
   ```cpp
   	UDEBUG("Data buffer is full, the oldest data is removed to add the new one.");
   ```
   UDEBUG only, i.e. invisible in a normal run.

3. `corelib/src/RtabmapThread.cpp:571-579` -- `Rtabmap/ImageBufferSize` (default 1) overflow.
   This one does warn (`ULOGGER_WARN`), but only when `_rate > 0`.

Separately, `Rtabmap/DetectionRate` (default 1 Hz) deliberately discards frames at
`RtabmapThread.cpp:536-539` -- that is by design, not a fault, and must not be
counted as a drop.

## Minimal change

In `rtabmap_odom/src/OdometryROS.cpp`, publish the ratio alongside the existing
OdomInfo publish (around :1054-1078, which already runs on the worker thread and is
already subscriber-count-gated):

```cpp
// new publisher, created next to odomInfoPub_ (OdometryROS.cpp:112-121):
//   dropRatioPub_ = this->create_publisher<std_msgs::msg::Float32>("odom_drop_ratio", rclcpp::QoS(1));

if(dropRatioPub_->get_subscription_count())
{
    const int total = processedMsgs_ + droppedMsgs_;
    std_msgs::msg::Float32 m;
    m.data = total ? float(droppedMsgs_) / float(total) : 0.0f;
    dropRatioPub_->publish(m);
}
```

Then:
```bash
ros2 topic echo /odom_drop_ratio
```

## Acceptance criterion for "30 FPS"

Do not accept a mean-latency number alone. Require **all three**:

| Metric | Source | Target |
|---|---|---|
| `/odom` publish rate | `ros2 topic hz /odom` | >= 29.5 Hz sustained |
| drop ratio | `/odom_drop_ratio` (this patch) | < 0.01 |
| `time_estimation` P95 | `/odom_info_lite` | < 33.33 ms |

The third alone is meaningless without the first two, because drops make the
surviving frames look fast.
