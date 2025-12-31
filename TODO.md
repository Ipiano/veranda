# TODO.md

This file tracks known issues and planned improvements for the Veranda project, organized by priority and dependency.

---

## Phase 1: Infrastructure Modernization

These changes should be completed first as they affect the entire codebase and will influence how subsequent improvements are implemented.

### 1.1 Upgrade ROS2 to Latest LTS

**Priority**: Critical
**Effort**: Large
**Files**: All `package.xml`, `CMakeLists.txt`, and ROS2-dependent source files

The project is locked to ROS2 Ardent Apalone (2017), which is long EOL. Upgrade to current LTS (Humble or newer).

- [ ] Update `package.xml` files for ROS2 Humble/Iron package format
- [ ] Replace `ament build` with `colcon build` throughout documentation
- [ ] Update ROS2 API calls for compatibility (rclcpp changes between versions)
- [ ] Update message type includes if any have changed
- [ ] Fix threading model - newer ROS2 versions have better multi-threading support
  - Remove the `spin_some()` timer hack in `main.cpp:67-89`
  - Implement proper multi-threaded executor or callback-based spinning
  - Consider `rclcpp::executors::MultiThreadedExecutor` with proper synchronization
- [ ] Update `README.md` with new build instructions
- [ ] Remove or archive legacy `bouncy` and `crystal` branches

### 1.2 Upgrade to Qt6

**Priority**: High
**Effort**: Medium
**Files**: All Qt-dependent source files, CMakeLists.txt, .pro files

- [ ] Update `find_package(Qt5 ...)` to `find_package(Qt6 ...)` with fallback
- [ ] Replace deprecated `qt5_wrap_cpp()` with `qt_wrap_cpp()` or rely on AUTOMOC
- [ ] Replace deprecated `qt5_use_modules()` with `target_link_libraries(... Qt6::Core Qt6::Gui Qt6::Widgets)`
- [ ] Update deprecated Qt5 API usage:
  - Check `QGraphicsItem::setRotation()` usage
  - Verify signal/slot connection syntax compatibility
  - Update any deprecated container methods
- [ ] Test QGraphicsView rendering for Qt6 compatibility
- [ ] Update `.pro` files for Qt6 or remove if no longer maintaining Qt Creator support

### 1.3 Modernize CMake Patterns

**Priority**: High
**Effort**: Medium
**Files**: All `CMakeLists.txt` files

- [ ] Increase minimum CMake version from 3.5 to 3.16+
- [ ] Replace `qt5_use_modules()` with modern `target_link_libraries()`:
  ```cmake
  # Old (deprecated)
  qt5_use_modules(${PROJECT_NAME} ${QT_COMPONENTS})

  # New
  target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets)
  ```
- [ ] Replace global `include_directories()` with `target_include_directories()`:
  ```cmake
  # Old
  include_directories("${veranda_box2d_INCLUDE_DIRS}")

  # New
  target_include_directories(${PROJECT_NAME} PRIVATE ${veranda_box2d_INCLUDE_DIRS})
  ```
- [ ] Use generator expressions for platform-specific settings
- [ ] Consider upgrading C++ standard from C++11 to C++17
- [ ] Replace embedded Box2D source with `FetchContent` or `find_package()`:
  ```cmake
  include(FetchContent)
  FetchContent_Declare(
    box2d
    GIT_REPOSITORY https://github.com/erincatto/box2d.git
    GIT_TAG v2.4.1
  )
  FetchContent_MakeAvailable(box2d)
  ```
- [ ] Fix plugin discovery path (`main.cpp:105-106`) to use proper ROS2 resource paths or ament index

---

## Phase 2: Architecture & Code Quality

These changes address software design issues and should be done after Phase 1 infrastructure is stable.

### 2.1 Fix Threading and Event Loop Architecture

**Priority**: Critical
**Effort**: Large
**Files**: `main.cpp`, `simulator_core.cpp`, `basic_physics.cpp`

The current single-threaded design with `spin_some()` every 30ms causes:
- Message delays for high-frequency topics
- UI blocking during physics/ROS processing
- Inability to handle real-time sensor rates (100Hz+)

- [ ] Design proper multi-threaded architecture:
  - Dedicated thread for ROS2 spinning
  - Physics loop on separate thread or timer
  - UI updates via queued signal/slot connections
- [ ] Implement thread-safe communication between ROS2 callbacks and Qt
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
**Effort**: Medium
**Files**: Sensor and wheel component classes

Components are tightly coupled to ROS2, making unit testing difficult.

- [ ] Abstract ROS2 channel creation behind an interface
- [ ] Allow dependency injection of mock publishers/subscribers for testing
- [ ] Add input validation for ROS2 message values (bounds checking)
- [ ] Add unit tests for component physics behavior

### 2.4 Fix Physics Engine Issues

**Priority**: Medium
**Effort**: Small
**Files**: `basic_physics.cpp`, component files

- [ ] Fix `BasicPhysics::clear()` to ensure all objects release body references before world deletion
- [ ] Make physics iteration parameters configurable (currently hardcoded `8, 3` in `world->Step()`)
- [ ] Add named constants for magic numbers
- [ ] Fix timestamp accumulation overflow risk in `simulator_core.cpp:71-77`:
  ```cpp
  _timestampMsg->data[0] += elapsed;  // Precision loss after hours
  ```
- [ ] Add comments explaining Box2D shape ownership (shapes are copied, safe to delete after fixture creation)

---

## Phase 3: Performance Optimization

These optimizations should be done after architectural issues are resolved.

### 3.1 Optimize Sensor Raycasting

**Priority**: Medium
**Effort**: Medium
**Files**: `lidar_sensor.cpp`, `touch_sensor.cpp`

Current implementation does one Box2D raycast per lidar ray per update.

- [ ] Profile raycast performance with multiple sensors
- [ ] Consider batch raycasting or caching spatial data
- [ ] Implement sensor sleep/wake based on physics activity
- [ ] Add configurable sensor update rates independent of physics tick

### 3.2 Reduce Signal/Slot Overhead

**Priority**: Low
**Effort**: Small
**Files**: `model.h`, `world_object_component.cpp`

- [ ] Add epsilon check before emitting `transformChanged` signal:
  ```cpp
  if(std::abs(dx) > EPSILON || std::abs(dy) > EPSILON || std::abs(dt) > EPSILON)
      transformChanged(this, dx, dy, dt);
  ```
- [ ] Consider batching model updates rather than per-model signals
- [ ] Profile signal emission frequency during typical simulation

### 3.3 Optimize UI Rendering

**Priority**: Low
**Effort**: Medium
**Files**: `qgraphicssimulationviewer.cpp`

- [ ] Replace full model rebuild on change with incremental updates in `modelChanged()`
- [ ] Implement view frustum culling for off-screen objects
- [ ] Consider using `QGraphicsScene::setItemIndexMethod()` for spatial indexing
- [ ] Profile and optimize `_updateColoring()` which recurses through all models

---

## Phase 4: Testing & Documentation

### 4.1 Expand Test Coverage

**Priority**: Medium
**Effort**: Large
**Files**: New test files in `tests/` directories

Current coverage is limited to Property and Model datatypes.

- [ ] Add physics integration tests (body creation, forces, collisions)
- [ ] Add component behavior tests (sensor output, wheel response)
- [ ] Add signal/slot integration tests
- [ ] Add plugin loading tests
- [ ] Add file save/load round-trip tests
- [ ] Set up CI pipeline for automated testing

### 4.2 Fix Property Validator Inconsistency

**Priority**: Low
**Effort**: Small
**Files**: `property.h`, `test_property.cpp`

The `angle_validator` implementation wraps angles to [0, 360), but tests expect rejection of out-of-range values.

- [ ] Clarify intended behavior (wrap vs reject)
- [ ] Fix either implementation or tests to match
- [ ] Add test cases for edge cases (exactly 360, negative angles, large angles)

### 4.3 Clean Up Technical Debt

**Priority**: Low
**Effort**: Small
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
Phase 1.1 (ROS2 Upgrade)
    └── Phase 2.1 (Threading) - Easier with modern ROS2 executors

Phase 1.2 (Qt6 Upgrade)
    └── Phase 3.3 (UI Optimization) - May have different performance characteristics

Phase 1.3 (CMake Modernization)
    └── Phase 4.1 (Testing) - Modern CMake makes test integration easier

Phase 2.2 (Wrappers) + Phase 2.3 (Testability)
    └── Phase 4.1 (Testing) - Cleaner architecture enables better tests
```

### Estimated Effort Scale

- **Small**: 1-2 hours, localized changes
- **Medium**: 1-2 days, multiple files or moderate complexity
- **Large**: 1+ weeks, significant refactoring or cross-cutting changes
