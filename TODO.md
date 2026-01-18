# TODO.md

This file tracks known issues and planned improvements for the Veranda project, organized by priority and dependency.

---

## Phase 1: Infrastructure Modernization

These changes should be completed first as they affect the entire codebase and will influence how subsequent improvements are implemented.

> **See detailed implementation plans**: [TODO_update_ros.md](TODO_update_ros.md), [TODO_upgrade_qt.md](TODO_upgrade_qt.md), [TODO_modernize_cmake.md](TODO_modernize_cmake.md)

### 1.1 Upgrade ROS2 to Jazzy Jalisco ✅ COMPLETED

**Priority**: Critical
**Files**: All `package.xml`, `CMakeLists.txt`, and ROS2-dependent source files

The project was locked to ROS2 Ardent Apalone (2017), which is long EOL. Upgraded to ROS2 Jazzy Jalisco (latest LTS).

> **Detailed plan**: See [TODO_update_ros.md](TODO_update_ros.md)

- [x] Update `package.xml` files for ROS2 Humble/Iron package format
- [x] Replace `ament build` with `colcon build` throughout documentation
- [x] Update ROS2 API calls for compatibility (rclcpp changes between versions)
- [x] Update message type includes if any have changed
- [x] Fix threading model - newer ROS2 versions have better multi-threading support
  - [x] Remove the `spin_some()` timer hack in `main.cpp:67-89`
  - [x] Implement background thread ROS2 spinning with `QtConcurrent::run`
  - [x] Update `qRegisterMetaType` calls for correct cross-thread signal/slot
- [ ] Update `README.md` with new build instructions
- [ ] Remove or archive legacy `bouncy` and `crystal` branches

### 1.2 Bare Bones Qt6 Upgrade

**Priority**: High
**Files**: All Qt-dependent source files, CMakeLists.txt, .pro files

**Strategy**: This is a two-phase upgrade approach:
- **Phase 1.2 (this section)**: Bare bones upgrade to get the project building with Qt6
- **Phase 2 (see section 2.5)**: Modernize to leverage Qt6-specific features and patterns

> **Detailed upgrade plan**: See [TODO_upgrade_qt.md](TODO_upgrade_qt.md)
> **Modernization plan**: See [TODO_modern_qt6.md](TODO_modern_qt6.md)

**Bare bones upgrade tasks**:
- [ ] Update `find_package(Qt5 ...)` to `find_package(Qt6 ...)` with fallback
- [ ] Replace deprecated `qt5_wrap_cpp()` with `qt_wrap_cpp()` or rely on AUTOMOC
- [ ] Replace deprecated `qt5_use_modules()` with `target_link_libraries(... Qt6::Core Qt6::Gui Qt6::Widgets)`
- [ ] Update deprecated Qt5 API usage:
  - Check `QGraphicsItem::setRotation()` usage
  - Verify signal/slot connection syntax compatibility
  - Update any deprecated container methods
- [ ] Test QGraphicsView rendering for Qt6 compatibility
- [x] Update `.pro` files for Qt6 or remove if no longer maintaining Qt Creator support
  - ✅ .pro files removed

### 1.3 Bare Bones CMake Modernization ✅ COMPLETED

**Priority**: High
**Files**: All `CMakeLists.txt` files

**Strategy**: This is a two-phase modernization approach:
- **Phase 1.3 (this section)**: Minimum CMake updates required for Qt6 and ROS2 Jazzy compatibility ✅
- **Phase 2 (see section 2.6)**: Full modernization to modern CMake 3.16+ patterns ✅

> **Detailed modernization plan**: See [TODO_modernize_cmake.md](TODO_modernize_cmake.md) ✅ **COMPLETED**

**Bare bones CMake tasks**:
- [x] Increase minimum CMake version from 3.5 to 3.28
- [x] Replace `qt5_use_modules()` with modern `target_link_libraries()`:
  ```cmake
  # Old (deprecated)
  qt5_use_modules(${PROJECT_NAME} ${QT_COMPONENTS})

  # New
  target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Core Qt5::Gui Qt5::Widgets)
  ```
- [x] Update C++ standard from C++11 to C++17 (required for ROS2 Jazzy)
- [x] Fix plugin discovery path (`main.cpp:105-106`) to use proper ROS2 resource paths or ament index

---

## Phase 2: Architecture & Code Quality

These changes address software design issues and should be done after Phase 1 infrastructure is stable.

> **See detailed plans**: [TODO_modern_ros.md](TODO_modern_ros.md), [TODO_component_ros_separation.md](TODO_component_ros_separation.md)

### 2.1 Fix Threading and Event Loop Architecture (Partially Complete)

**Priority**: Medium (basic fix done, improvements optional)
**Files**: `main.cpp`, `simulator_core.cpp`, `basic_physics.cpp`

**Basic threading fix completed** ✅:
- ROS2 spinning moved to background thread via `QtConcurrent::run`
- Subscriber callbacks use Qt signal/slot with `qRegisterMetaType` for thread-safe delivery
- Removed the 30ms `spin_some()` polling hack

> **Detailed design for further improvements**: See [TODO_modern_ros.md](TODO_modern_ros.md)

**Optional enhancements** (not yet implemented):
- [ ] Use `MultiThreadedExecutor` for parallel callback processing
- [ ] Add callback groups for finer concurrency control
- [ ] Consider using `QThread` with `moveToThread()` for physics engine
- [ ] Add proper synchronization for shared data (world objects, models)
- [ ] Profile and verify no race conditions

### 2.2 Resolve Wrapper Pattern Issues

**Priority**: Medium
**Effort**: Medium
**Files**: `world_object_wrappers.h`, `simulator_core.cpp`, UI code

The `WorldObjectProperties` and `WorldObjectPhysics` wrappers are defeated by escape-hatch methods.

- [ ] Decide: either remove wrappers entirely OR refactor to eliminate `getObject()`/`getComponent()` methods
- [ ] If keeping wrappers, redesign file save/load to not need direct object access
- [ ] Rename overloaded signals in `SimulatorCore`:
  ```cpp
  // Current (ambiguous)
  void objectsAdded(QVector<QPair<WorldObjectPhysics*, object_id>> objs);
  void objectsAdded(QVector<QPair<WorldObjectProperties*, object_id>> objs);

  // Proposed
  void objectsAddedToPhysics(QVector<QPair<WorldObjectPhysics*, object_id>> objs);
  void objectsAddedToUi(QVector<QPair<WorldObjectProperties*, object_id>> objs);
  ```
- [ ] Remove deprecated `usesChannels()` method or remove deprecation comment

### 2.3 Improve Component Testability

**Priority**: Medium
**Files**: Sensor and wheel component classes

Components are tightly coupled to ROS2, making unit testing difficult.

> **Detailed design**: See [TODO_component_ros_separation.md](TODO_component_ros_separation.md)

- [ ] Abstract ROS2 channel creation behind an interface
- [ ] Allow dependency injection of mock publishers/subscribers for testing
- [ ] Add input validation for ROS2 message values (bounds checking)
- [ ] Add unit tests for component physics behavior

### 2.4 Fix Physics Engine Issues

**Priority**: Medium
**Files**: `basic_physics.cpp`, component files

- [ ] Fix `BasicPhysics::clear()` to ensure all objects release body references before world deletion
- [ ] Make physics iteration parameters configurable (currently hardcoded `8, 3` in `world->Step()`)
- [ ] Add named constants for magic numbers
- [ ] Fix timestamp accumulation overflow risk in `simulator_core.cpp:71-77`:
  ```cpp
  _timestampMsg->data[0] += elapsed;  // Precision loss after hours
  ```
- [ ] Add comments explaining Box2D shape ownership (shapes are copied, safe to delete after fixture creation)

### 2.5 Qt6 Modernization (Post-Upgrade)

**Priority**: Low
**Files**: All Qt-dependent source files

After the bare bones Qt6 upgrade (Phase 1.2), modernize to use Qt6-specific features.

> **Detailed plan**: See [TODO_modern_qt6.md](TODO_modern_qt6.md)

- [ ] Implement Qt6 property bindings for reactive updates
- [ ] Optimize QGraphicsView with Qt6 rendering improvements
- [ ] Apply Qt6 concurrent processing patterns
- [ ] Consider QML integration for future UI work

### 2.6 Full CMake Modernization (Post-Upgrade) ✅ COMPLETED

**Priority**: Medium
**Files**: All `CMakeLists.txt` files

Full modern CMake patterns have been applied.

> **Detailed plan**: See [TODO_modernize_cmake.md](TODO_modernize_cmake.md) ✅ **COMPLETED**

- [x] Replace global `include_directories()` with `target_include_directories()`
- [x] Replace global `add_definitions()` with `target_compile_definitions()`
- [x] Use generator expressions for platform-specific settings
- [ ] Export proper CMake targets for downstream packages (optional future improvement)
- [ ] Create shared CMake module for common patterns (optional future improvement)
- [ ] Replace embedded Box2D with `FetchContent` or `find_package()` (optional future improvement)

---

## Phase 3: Performance Optimization

These optimizations should be done after architectural issues are resolved.

### 3.1 Optimize Sensor Raycasting

**Priority**: Medium
**Files**: `lidar_sensor.cpp`, `touch_sensor.cpp`

Current implementation does one Box2D raycast per lidar ray per update.

> **Detailed design**: See [TODO_raycasting_optimization.md](TODO_raycasting_optimization.md)

- [ ] Profile raycast performance with multiple sensors
- [ ] Consider batch raycasting or caching spatial data
- [ ] Implement sensor sleep/wake based on physics activity
- [ ] Add configurable sensor update rates independent of physics tick

### 3.2 Reduce Signal/Slot Overhead

**Priority**: Low
**Files**: `model.h`, `world_object_component.cpp`

- [ ] Add epsilon check before emitting `transformChanged` signal:
  ```cpp
  if(std::abs(dx) > EPSILON || std::abs(dy) > EPSILON || std::abs(dt) > EPSILON)
      transformChanged(this, dx, dy, dt);
  ```
- [ ] Consider batching model updates rather than per-model signals
- [ ] Profile signal emission frequency during typical simulation

### 3.3 Vehicle Physics Refactor (Mecanum Wheels)

**Priority**: Low (unless mecanum wheels are critical feature)
**Files**: Wheel components, physics engine

Address mecanum wheel force cancellation and numerical instability.

> **Detailed design**: See [TODO_vehicle_physics.md](TODO_vehicle_physics.md)

- [ ] Implement rigid island detection for multi-body robots
- [ ] Custom force summation for mecanum wheel configurations
- [ ] Direct velocity updates to bypass Box2D constraint solver for rigid islands

### 3.4 Optimize UI Rendering

**Priority**: Low
**Files**: `qgraphicssimulationviewer.cpp`

- [ ] Replace full model rebuild on change with incremental updates in `modelChanged()`
- [ ] Implement view frustum culling for off-screen objects
- [ ] Consider using `QGraphicsScene::setItemIndexMethod()` for spatial indexing
- [ ] Profile and optimize `_updateColoring()` which recurses through all models

---

## Phase 4: Testing & Documentation

### 4.1 Expand Test Coverage

**Priority**: Medium
**Files**: New test files in `tests/` directories

Current coverage is limited to Property and Model datatypes.

> **Note**: Component testability improvements in Phase 2.3 will enable more comprehensive testing

- [ ] Add physics integration tests (body creation, forces, collisions)
- [ ] Add component behavior tests (sensor output, wheel response)
- [ ] Add signal/slot integration tests
- [ ] Add plugin loading tests
- [ ] Add file save/load round-trip tests
- [ ] Set up CI pipeline for automated testing

### 4.2 Fix Property Validator Inconsistency

**Priority**: Low
**Files**: `property.h`, `test_property.cpp`

The `angle_validator` implementation wraps angles to [0, 360), but tests expect rejection of out-of-range values.

- [ ] Clarify intended behavior (wrap vs reject)
- [ ] Fix either implementation or tests to match
- [ ] Add test cases for edge cases (exactly 360, negative angles, large angles)

### 4.3 Migrate to GoogleTest (Optional)

**Priority**: Very Low
**Files**: Test infrastructure

Current test suite uses Catch2. Migration to GoogleTest would align with ROS2 ecosystem.

> **Detailed plan**: See [TODO_migrate_to_gtest.md](TODO_migrate_to_gtest.md)

**Note**: Given the small test suite (only 2 test files), this migration provides limited value and can be deferred indefinitely.

- [ ] Decide if migration is worth the effort
- [ ] If yes, follow detailed migration plan

### 4.4 Clean Up Technical Debt

**Priority**: Low
**Files**: Various

- [ ] Move TODO/FIXME comments to this file or issue tracker:
  - `main.cpp:68` - ROS2 threading
  - `world_object_wrappers.h:1` - Wrapper removal
  - `basic_physics.h:17` - Gravity support
- [ ] Establish consistent error handling policy (exceptions vs return codes vs silent failures)
- [ ] Replace remaining magic numbers with named constants
- [ ] Update Doxygen comments for any changed APIs

---

## Notes

### Dependencies Between Tasks

```
Phase 1.1 (ROS2 Upgrade) ✅ COMPLETED
    └── Phase 2.1 (Threading) - Basic fix done, optional improvements available

Phase 1.2 (Qt6 Upgrade)
    └── Phase 3.3 (UI Optimization) - May have different performance characteristics

Phase 1.3 (CMake Modernization) ✅ COMPLETED
    └── Phase 4.1 (Testing) - Modern CMake makes test integration easier

Phase 2.6 (Full CMake Modernization) ✅ COMPLETED

Phase 2.2 (Wrappers) + Phase 2.3 (Testability)
    └── Phase 4.1 (Testing) - Cleaner architecture enables better tests
```

## Related Documentation

Detailed implementation plans for specific tasks:

- **[TODO_update_ros.md](TODO_update_ros.md)**: ROS2 upgrade to Jazzy Jalisco ✅ **COMPLETED**
- **[TODO_upgrade_qt.md](TODO_upgrade_qt.md)**: Qt 5 to Qt 6 upgrade (bare bones)
- **[TODO_modern_qt6.md](TODO_modern_qt6.md)**: Qt 6 modernization (post-upgrade)
- **[TODO_modernize_cmake.md](TODO_modernize_cmake.md)**: Full CMake modernization ✅ **COMPLETED**
- **[TODO_modern_ros.md](TODO_modern_ros.md)**: Modern ROS2 threading architecture (optional improvements)
- **[TODO_component_ros_separation.md](TODO_component_ros_separation.md)**: Component testability improvements
- **[TODO_raycasting_optimization.md](TODO_raycasting_optimization.md)**: Sensor performance optimization
- **[TODO_vehicle_physics.md](TODO_vehicle_physics.md)**: Mecanum wheel physics refactor
- **[TODO_migrate_to_gtest.md](TODO_migrate_to_gtest.md)**: GoogleTest migration (optional)
