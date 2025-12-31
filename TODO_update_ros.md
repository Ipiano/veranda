# ROS 2 Upgrade Plan: Crystal → Jazzy Jalisco

This document outlines the work required to upgrade Veranda from ROS 2 Crystal to ROS 2 Jazzy Jalisco (the latest LTS release).

## Current State

- **Master branch**: Targets ROS 2 Crystal (using ament build)
- **Target**: ROS 2 Jazzy Jalisco (May 2024, LTS until May 2029)

### Existing Upgrade Work

The `hygum` git remote contains partial upgrade work across several branches:

| Branch | Target ROS 2 Version | Status |
|--------|---------------------|--------|
| `hygum/dashing-support-and-ros-parameters` | Dashing | Partial |
| `hygum/eloquent` | Eloquent | Partial |
| `hygum/eloquent-develop` | Eloquent | Development |
| `hygum/foxy` | Foxy | Most complete |

**Recommendation**: Start from `hygum/foxy` as the base for Jazzy upgrade.

---

## Changes Already in hygum/foxy Branch

The `hygum/foxy` branch (14 commits ahead of master) addresses these critical changes:

### 1. C++ Standard Upgrade (C++11 → C++14)

All CMakeLists.txt files updated:
```cmake
# Old (Crystal)
set(CMAKE_CXX_STANDARD 11)

# New (Foxy)
set(CMAKE_CXX_STANDARD 14)
```

**Files affected:**
- `veranda/Packages/builtin_plugin_logic/pkg-veranda_builtin_sensors/CMakeLists.txt`
- `veranda/Packages/builtin_plugin_logic/pkg-veranda_builtin_wheels/CMakeLists.txt`
- `veranda/Packages/builtin_plugin_logic/pkg-veranda_builtin_shapes/CMakeLists.txt`
- `veranda/Packages/core/pkg-veranda_core/CMakeLists.txt`
- `veranda/Packages/core/pkg-veranda_core_api/CMakeLists.txt`
- `veranda/Packages/qt_frontend/pkg-veranda_qt_frontend/CMakeLists.txt`
- `veranda/Packages/qt_plugins/pkg-veranda_builtin_plugin_qt_wrappers/CMakeLists.txt`
- `veranda/Packages/qt_plugins/pkg-veranda_qt_plugin_api/CMakeLists.txt`
- `veranda/Packages/libraries/pkg-veranda_box2d/CMakeLists.txt`

### 2. QoS Parameters for Subscriptions

ROS 2 Dashing+ requires explicit QoS depth parameter:

```cpp
// Old (Crystal) - no QoS parameter
_receiveChannel = _rosNode->create_subscription<std_msgs::msg::Float32>(
    inputChannel.toStdString(), callback);

// New (Foxy+) - QoS depth required
_receiveChannel = _rosNode->create_subscription<std_msgs::msg::Float32>(
    inputChannel.toStdString(), 10, callback);
```

**Files affected:**
- `pkg-veranda_builtin_wheels/src/ackermann_steer.cpp`
- `pkg-veranda_builtin_wheels/src/fixed_wheel.cpp`
- `pkg-veranda_builtin_wheels/src/omni_drive.cpp`

### 3. Publisher API Change

The `publish()` method now takes a reference instead of shared_ptr:

```cpp
// Old (Crystal)
_sendChannel->publish(data);  // data is std::shared_ptr<MessageT>

// New (Foxy+)
_sendChannel->publish(*data);  // dereference the shared_ptr
```

**Files affected:**
- `pkg-veranda_builtin_sensors/src/gps_sensor.cpp`
- `pkg-veranda_builtin_sensors/src/lidar_sensor.cpp`
- `pkg-veranda_builtin_sensors/src/touch_sensor.cpp`
- `pkg-veranda_builtin_wheels/src/encoder.cpp`
- `pkg-veranda_builtin_wheels/src/omni_drive.cpp`

### 4. Plugin Discovery via ament_index

Replaced relative path plugin discovery with ament_index:

```cpp
// Old (Crystal) - relative path hack
QDirIterator dir(QCoreApplication::applicationDirPath() + "/../../veranda_plugins", ...);

// New (Foxy+) - ament_index resource lookup
std::string plugins_path;
ament_index_cpp::get_resource("veranda_plugin_path", "veranda_qt_frontend", plugins_path);
QDirIterator dir(plugins_path.c_str(), ...);
```

**CMakeLists.txt addition:**
```cmake
find_package(ament_index_cpp REQUIRED)
ament_index_register_resource(veranda_plugin_path CONTENT "${CMAKE_INSTALL_PREFIX}/lib/veranda_qt_frontend")
```

**package.xml addition:**
```xml
<build_depend>ament_index_cpp</build_depend>
<exec_depend>ament_index_cpp</exec_depend>
```

### 5. Threading Fix

Re-enabled multi-threaded ROS spin (was disabled in Crystal due to instability):

```cpp
// Old (Crystal) - timer-based spin_some hack
QTimer spinTimer;
spinTimer.setInterval(30);
QObject::connect(&spinTimer, &QTimer::timeout, [&](){
    rclcpp::spin_some(node);
});

// New (Foxy+) - separate thread for spin
QFutureWatcher<void> rosThread;
rosThread.setFuture(QtConcurrent::run([node](){rclcpp::spin(node);}));
```

### 6. ROS 2 Parameters Support

Added support for launch file parameters:

```cpp
rclcpp::NodeOptions options;
shared_ptr<rclcpp::Node> node = make_shared<rclcpp::Node>("veranda",
    options.allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true));

// Parameter usage
bool auto_load_asset = parameter_node->get_parameter("auto_load_asset").as_bool();
std::string json_asset_path = parameter_node->get_parameter("json_asset_path").as_string();
```

### 7. package.xml Format Update

Updated from format 2 to format 3 with separated build/exec dependencies.

---

## Additional Work Required for Jazzy

### 1. C++ Standard Upgrade (C++14 → C++17)

Jazzy requires C++17:

```cmake
# Change in all CMakeLists.txt
set(CMAKE_CXX_STANDARD 17)
```

### 2. Subscription Callback Signature Change

**Breaking change in Jazzy**: Callbacks must use `const` message type:

```cpp
// Old (Foxy and earlier)
void callback(std::shared_ptr<MessageT> msg);

// New (Jazzy) - REQUIRED
void callback(std::shared_ptr<const MessageT> msg);
```

**Files requiring changes:**
- `pkg-veranda_builtin_wheels/src/ackermann_steer.cpp`
- `pkg-veranda_builtin_wheels/src/fixed_wheel.cpp`
- `pkg-veranda_builtin_wheels/src/omni_drive.cpp`

**Example fix in omni_drive.cpp:**
```cpp
// Old
auto callback = [this](const geometry_msgs::msg::Pose2D::SharedPtr msg) -> void {
    _receiveMessage(msg);
};

// New (Jazzy)
auto callback = [this](const std::shared_ptr<const geometry_msgs::msg::Pose2D> msg) -> void {
    _receiveMessage(msg);
};
```

### 3. QoS Event Header Removal

If any code uses `rclcpp/qos_event.hpp`, update to:

```cpp
// Old (deprecated in Iron, removed in Jazzy)
#include <rclcpp/qos_event.hpp>

// New
#include <rclcpp/event_handler.hpp>
```

### 4. CMake Modernization

Replace deprecated Qt5 CMake functions:

```cmake
# Old (deprecated in Qt 5.11)
qt5_use_modules(${EXE_NAME} ${QT_COMPONENTS})

# New
target_link_libraries(${EXE_NAME} PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui)
```

### 5. Timer Callback Enhancement (Optional)

Jazzy adds `rclcpp::TimerInfo` to timer callbacks for timing information:

```cpp
// Old
auto callback = []() { /* timer code */ };

// New (optional, for timing info)
auto callback = [](rclcpp::TimerInfo & info) {
    auto expected = info.expected_call_time;
    auto actual = info.actual_call_time;
    /* timer code */
};
```

---

## Plugin Architecture

### Current Approach

Veranda uses **Qt's QPluginLoader** for plugin discovery, not ROS pluginlib. This is intentional given the Qt-based frontend.

**Plugin interface hierarchy:**
```
WorldObjectComponent_Plugin_If  (Qt plugin interface)
    ├── Circle_Plugin
    ├── Rectangle_Plugin
    ├── Polygon_Plugin
    ├── GPS_Plugin
    ├── Lidar_Plugin
    ├── Touch_Plugin
    ├── Fixed_Wheel_Plugin
    ├── Omni_Drive_Plugin
    └── Ackermann_Steer_Plugin
```

### hygum/foxy Improvements

- Plugin install path now uses ament_index resource registration
- Plugin discovery uses `ament_index_cpp::get_resource()`
- Plugins install to `${CMAKE_INSTALL_PREFIX}/lib/veranda_qt_frontend`

### Migration to ROS pluginlib (Optional)

If desired, migrating to ROS pluginlib would require:

1. Add `pluginlib` dependency to all plugin packages
2. Create plugin description XML files for each plugin
3. Replace Qt plugin macros with pluginlib macros:
   ```cpp
   // Qt style
   Q_PLUGIN_METADATA(IID "org.roboscience.veranda.worldObjectComponent.defaults.gps")
   Q_INTERFACES(WorldObjectComponent_Plugin_If)

   // ROS pluginlib style
   PLUGINLIB_EXPORT_CLASS(GPS_Plugin, WorldObjectComponent_Plugin_If)
   ```
4. Update CMakeLists.txt to export plugin descriptions
5. Update plugin loading code in main.cpp

**Recommendation**: Keep Qt plugin system unless there's a specific need for ROS pluginlib features.

---

## Platform Requirements

### Jazzy Supported Platforms

| Platform | Architecture | Status |
|----------|--------------|--------|
| Ubuntu 24.04 (Noble) | amd64, arm64 | Tier 1 |
| RHEL 9 | amd64 | Tier 1 |
| Windows 10 | amd64 | Tier 1 |

**Note**: Ubuntu 22.04 (Jammy) is NOT supported by Jazzy. If you need Ubuntu 22.04 support, use ROS 2 Humble or Iron instead.

### Qt Requirements

- Qt 5.10+ required (Qt 5.15 LTS recommended)
- Consider Qt 6 migration for long-term support

---

## File-by-File Change Summary

### CMakeLists.txt Changes (All packages)

| Change | Description |
|--------|-------------|
| C++ Standard | `set(CMAKE_CXX_STANDARD 17)` |
| Qt linking | Replace `qt5_use_modules()` with `target_link_libraries()` |
| ament_index | Add `find_package(ament_index_cpp REQUIRED)` where needed |

### Source File Changes

| File | Changes Required |
|------|-----------------|
| `main.cpp` | Already updated in hygum/foxy |
| `gps_sensor.cpp` | `publish(*data)`, const callback signature |
| `lidar_sensor.cpp` | `publish(*data)`, const callback signature |
| `touch_sensor.cpp` | `publish(*data)`, const callback signature |
| `encoder.cpp` | `publish(*data)` |
| `omni_drive.cpp` | QoS param, `publish(*data)`, const callback |
| `fixed_wheel.cpp` | QoS param, const callback |
| `ackermann_steer.cpp` | QoS param, const callback |

### package.xml Changes

- Update to format 3 (done in hygum/foxy)
- Add `ament_index_cpp` dependencies where needed

---

## Testing Checklist

After upgrade, verify:

- [ ] Project builds without warnings
- [ ] All plugins load correctly
- [ ] GPS sensor publishes Pose2D messages
- [ ] Lidar sensor publishes LaserScan messages
- [ ] Touch sensor publishes contact data
- [ ] Omni drive receives velocity commands
- [ ] Fixed wheel receives speed commands
- [ ] Ackermann steering receives commands
- [ ] Robot JSON files load correctly
- [ ] Image maps load correctly
- [ ] Simulation runs at correct physics rate
- [ ] ROS parameter loading works
- [ ] Multi-threaded spin is stable

---

## Estimated Effort

| Phase | Description | Effort |
|-------|-------------|--------|
| 1. Merge hygum/foxy | Rebase or merge foxy branch to master | 1-2 hours |
| 2. C++17 + CMake | Update all CMakeLists.txt files | 2-4 hours |
| 3. Callback signatures | Fix subscription callback types | 4-6 hours |
| 4. Testing | Build, run, debug on Jazzy | 8-16 hours |
| 5. Documentation | Update README, CLAUDE.md | 1-2 hours |

**Total**: 2-4 days for experienced ROS 2 developer

---

## References

- [ROS 2 Jazzy Release Notes](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
- [ROS 2 Foxy Release Notes](https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html)
- [ROS 2 Dashing Release Notes](https://docs.ros.org/en/foxy/Releases/Release-Dashing-Diademata.html)
- [rclcpp Jazzy API Documentation](https://docs.ros.org/en/jazzy/p/rclcpp/generated/index.html)
- [Migrating C++ Packages to ROS 2](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-CPP-Package-Example.html)
