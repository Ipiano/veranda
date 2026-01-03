# Design Proposal: Modern ROS2 Integration

## Overview

This document proposes leveraging modern ROS2 features to improve the threading architecture beyond the basic upgrade.

> **Related documents**:
> - [TODO.md](TODO.md) - Phase 2.1 (Fix threading architecture)
> - [TODO_update_ros.md](TODO_update_ros.md) - Phase 1.1 (ROS2 upgrade to Jazzy) ✅ **COMPLETED**
> - [TODO_component_ros_separation.md](TODO_component_ros_separation.md) - Component testability improvements

## Current State (Post-Upgrade)

### What's Been Done ✅

1. **ROS2 Jazzy Upgrade**: Completed - all API changes applied
2. **Threading Fix**: The `spin_some()` timer hack has been removed
   - Now uses `QtConcurrent::run([node]() { rclcpp::spin(node); })` for background spinning
   - Subscriber callbacks use Qt signal/slot with `qRegisterMetaType` for thread-safe delivery
3. **Build System**: Uses `colcon build`

### Remaining Opportunities (This Document)

The improvements below are **optional enhancements** that can further improve performance and architecture:

1. **MultiThreadedExecutor**: Currently uses single-threaded spin - could parallelize callbacks
2. **Callback Groups**: Could provide finer-grained concurrency control
3. **QoS Profiles**: Currently uses hardcoded QoS values
4. **Parameter Server**: Not using ROS2 parameter system
5. **Lifecycle Nodes**: Not using managed lifecycle transitions

### Current Architecture (Post-Upgrade)

```
┌──────────────────────────────────┐  ┌──────────────────────────────────┐
│          Main Thread             │  │       ROS2 Background Thread     │
│  ┌─────────────┐  ┌───────────┐  │  │  ┌─────────────────────────────┐ │
│  │ Qt Event    │  │ Physics   │  │  │  │ rclcpp::spin(node)          │ │
│  │ Loop        │  │ Step      │  │  │  │                             │ │
│  └──────┬──────┘  └───────────┘  │  │  └──────────────┬──────────────┘ │
│         │                        │  │                 │                │
│         │  Qt Queued Signals     │  │                 │                │
│         │◄───────────────────────┼──┼─────────────────┘                │
│         ▼                        │  │                                  │
│  ┌─────────────────────────────┐ │  │                                  │
│  │ _processMessage() slots     │ │  │                                  │
│  │ (thread-safe via Qt)        │ │  │                                  │
│  └─────────────────────────────┘ │  │                                  │
└──────────────────────────────────┘  └──────────────────────────────────┘
```

## Proposed Improvements (Optional)

### ROS2 Version

**Already upgraded to ROS2 Jazzy Jalisco** ✅

The improvements below leverage Jazzy's mature multi-threading support with executors.

### Proposed Threading Architecture (MultiThreadedExecutor)

```
┌────────────────────────────────────────────────────────────────────┐
│                                                                    │
│  ┌──────────────────┐     ┌──────────────────┐                    │
│  │   Main Thread    │     │   ROS2 Thread    │                    │
│  │                  │     │                  │                    │
│  │  ┌────────────┐  │     │  ┌────────────┐  │                    │
│  │  │ Qt Event   │  │     │  │ MultiThread│  │                    │
│  │  │ Loop       │  │◄────┼──│ Executor   │  │                    │
│  │  └────────────┘  │     │  └────────────┘  │                    │
│  │        │         │     │        │         │                    │
│  │        ▼         │     │        ▼         │                    │
│  │  ┌────────────┐  │     │  ┌────────────┐  │                    │
│  │  │ UI Updates │  │     │  │ Callbacks  │  │                    │
│  │  │ + Physics  │  │     │  │ (pub/sub)  │  │                    │
│  │  └────────────┘  │     │  └────────────┘  │                    │
│  └──────────────────┘     └──────────────────┘                    │
│           ▲                        │                               │
│           │    Thread-Safe Queue   │                               │
│           └────────────────────────┘                               │
└────────────────────────────────────────────────────────────────────┘
```

### Implementation Details

#### 1. ROS2 Executor Selection

Use `rclcpp::executors::MultiThreadedExecutor` for parallel callback processing:

```cpp
// New main.cpp structure
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("veranda");

    // Create executor with callback groups
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Qt application
    QApplication app(argc, argv);

    // Run executor in dedicated thread
    std::thread ros_thread([&executor]() {
        executor.spin();
    });

    // ... setup simulator ...

    int ret = app.exec();

    // Cleanup
    rclcpp::shutdown();
    ros_thread.join();

    return ret;
}
```

#### 2. Callback Groups for Concurrency Control

Use callback groups to control which callbacks can run in parallel:

```cpp
// In component base class or SimulatorCore
class ComponentBase {
protected:
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    void setupCallbackGroup(rclcpp::Node::SharedPtr node) {
        // MutuallyExclusive: callbacks in this group won't run in parallel
        callback_group_ = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    }

    // Use when creating subscriptions
    auto createSubscription(/* ... */) {
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;
        return node_->create_subscription<MsgType>(
            topic, qos, callback, options);
    }
};
```

#### 3. Thread-Safe Communication with Qt

Use Qt's thread-safe queued connections for ROS2→Qt communication:

```cpp
// In component classes
class Omni_Drive : public WorldObjectComponent {
    Q_OBJECT

signals:
    // Signal for thread-safe message delivery
    void velocityCommandReceived(double x, double y, double theta);

private:
    void setupSubscription() {
        // ROS2 callback - runs in executor thread
        auto callback = [this](const geometry_msgs::msg::Pose2D::SharedPtr msg) {
            // Emit signal - Qt will queue this for main thread
            emit velocityCommandReceived(msg->x, msg->y, msg->theta);
        };

        subscription_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
            topic_, rclcpp::QoS(10), callback);
    }

private slots:
    // Slot runs in main thread, safe to access physics
    void onVelocityCommand(double x, double y, double theta) {
        _targetXVelocity = x;
        _targetYVelocity = y;
        _targetAngularVelocity = theta;
    }
};

// In constructor, ensure queued connection
connect(this, &Omni_Drive::velocityCommandReceived,
        this, &Omni_Drive::onVelocityCommand,
        Qt::QueuedConnection);
```

#### 4. Modern QoS Profiles

Replace hardcoded QoS values with named profiles:

```cpp
// Create a QoS utility header
namespace veranda::qos {

// Sensor data - best effort, keep latest
inline rclcpp::QoS sensor_data() {
    return rclcpp::SensorDataQoS();
}

// Commands - reliable, small queue
inline rclcpp::QoS commands() {
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();
}

// Parameters - reliable, transient local for late joiners
inline rclcpp::QoS parameters() {
    return rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .transient_local();
}

} // namespace veranda::qos

// Usage in components
_sendChannel = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    topic, veranda::qos::sensor_data());
```

#### 5. Parameter Server Integration

Use ROS2 parameter system for runtime configuration:

```cpp
class SimulatorCore : public rclcpp::Node {
public:
    SimulatorCore() : Node("veranda") {
        // Declare parameters with defaults
        this->declare_parameter("physics.tick_rate", 30.0);
        this->declare_parameter("physics.step_time", 1.0/30.0);
        this->declare_parameter("physics.velocity_iterations", 8);
        this->declare_parameter("physics.position_iterations", 3);

        // Setup parameter change callback
        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&SimulatorCore::onParameterChange, this, _1));
    }

private:
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& params)
    {
        for (const auto& param : params) {
            if (param.get_name() == "physics.tick_rate") {
                updateTickRate(param.as_double());
            }
            // ... handle other parameters
        }
        return rcl_interfaces::msg::SetParametersResult{.successful = true};
    }
};
```

#### 6. Lifecycle Node Consideration

Consider using lifecycle nodes for controlled startup/shutdown:

```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class VerandaNode : public rclcpp_lifecycle::LifecycleNode {
public:
    VerandaNode() : LifecycleNode("veranda") {}

    // Called during configuration
    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
        // Setup publishers, subscribers (but don't activate)
        return CallbackReturn::SUCCESS;
    }

    // Called when transitioning to active
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
        // Start physics, enable message flow
        physics_->start();
        return CallbackReturn::SUCCESS;
    }

    // Called when transitioning to inactive
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
        // Pause physics, stop message flow
        physics_->stop();
        return CallbackReturn::SUCCESS;
    }

    // Called during cleanup
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
        // Release resources
        return CallbackReturn::SUCCESS;
    }
};
```

### Implementation Steps

#### Step 1: Complete ROS2 Jazzy Upgrade ✅ DONE

The basic ROS2 Jazzy upgrade has been completed:
- All `package.xml` files updated
- CMakeLists.txt updated for Jazzy
- Deprecated rclcpp APIs updated
- Background thread spinning implemented (QtConcurrent + queued signals)

#### Step 2: Threading Refactor (Optional Enhancement)

1. Implement `MultiThreadedExecutor` approach
2. Add callback groups to components
3. Update signal/slot connections for thread safety
4. Add mutex protection for shared state if needed

#### Step 3: Add Modern ROS2 Features

1. Implement QoS profiles
2. Add parameter server support
3. Consider lifecycle node migration
4. Add launch file support

#### Step 4: Testing and Validation

1. Verify message latency improvements
2. Profile CPU usage compared to old approach
3. Test with high-frequency sensors (100Hz+)
4. Stress test with multiple robots

### API Changes

#### Breaking Changes

| Old API | New API | Reason |
|---------|---------|--------|
| `rclcpp::spin(node)` | `executor.spin()` | Multi-threaded execution |
| Direct callback to member | Signal/slot via queue | Thread safety |
| Hardcoded QoS depth | `veranda::qos::*` profiles | Configurability |

#### New APIs

| API | Purpose |
|-----|---------|
| `ComponentBase::callback_group_` | Control callback parallelism |
| `veranda::qos::sensor_data()` | Standard QoS for sensors |
| `veranda::qos::commands()` | Standard QoS for control |
| ROS2 parameters | Runtime configuration |

### Testing Strategy

1. **Unit Tests**: Test callback group isolation
2. **Integration Tests**: Verify message flow with executor
3. **Latency Tests**: Measure improvement over 30ms polling
4. **Stress Tests**: Multiple publishers at high rates
5. **Thread Safety Tests**: Verify no data races with TSAN

### Rollback Plan

If issues arise:
1. Executor approach can fall back to `SingleThreadedExecutor`
2. Keep `spin_some()` as a compile-time option
3. Maintain compatibility shims for old API during transition

### Success Metrics

| Metric | Current | Target |
|--------|---------|--------|
| Message latency | 30ms average | <5ms average |
| Max publish rate | ~33Hz | 1000Hz+ |
| UI responsiveness | Blocks on ROS | Independent |
| CPU efficiency | Polling waste | Event-driven |

## References

- [ROS2 Jazzy Migration Guide](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
- [ROS2 Executors Documentation](https://docs.ros.org/en/jazzy/Concepts/About-Executors.html)
- [ROS2 Callback Groups](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html)
- [ROS2 QoS Profiles](https://docs.ros.org/en/jazzy/Concepts/About-Quality-of-Service-Settings.html)
