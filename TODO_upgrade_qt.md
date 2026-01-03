# Qt 5 to Qt 6 Upgrade Plan

This document outlines the work required to upgrade Veranda from Qt 5 to Qt 6.

> **Related documents**:
> - [TODO.md](TODO.md) - Phase 1.2 (Bare bones Qt6 upgrade)
> - [TODO_modern_qt6.md](TODO_modern_qt6.md) - Phase 2.5 (Qt6 modernization post-upgrade)
> - [TODO_modernize_cmake.md](TODO_modernize_cmake.md) - CMake modernization required for Qt6

## Current State

- **Current Qt Version**: Qt 5 (5.10+ required, 5.15 LTS recommended)
- **Target Qt Version**: Qt 6.x (6.5 LTS or 6.6+ recommended)
- **Build Systems**: CMake (primary), QMake .pro files (IDE support only)

### Qt Modules Currently Used

| Module | Packages Using It |
|--------|-------------------|
| Qt Core | All packages |
| Qt Gui | Core API, Qt Frontend, Plugins |
| Qt Widgets | Qt Frontend, Image Loader |
| Qt Concurrent | Qt Frontend (main.cpp, mainwindow.cpp) |
| Qt OpenGL | Qt Frontend (JoystickWidget) |

---

## Pre-Migration Steps

### 1. Update to Qt 5.15 First

Before migrating to Qt 6, update to Qt 5.15 LTS. This version provides:
- Deprecation warnings for APIs removed in Qt 6
- Maximum source compatibility with Qt 6
- Ability to fix issues incrementally

Enable deprecation warnings in CMakeLists.txt:
```cmake
add_compile_definitions(QT_DEPRECATED_WARNINGS)
# Or to treat as errors:
add_compile_definitions(QT_DISABLE_DEPRECATED_BEFORE=0x060000)
```

### 2. Consider Build System Migration

Qt 6 strongly recommends CMake over QMake. The project already uses CMake as the primary build system, so this is not a blocker. The .pro files are for IDE support only.

---

## Breaking Changes and Required Updates

### 1. CMake Build System Changes

#### 1.1 Replace `find_package(Qt5 ...)` with Qt 6

**Old (Qt 5):**
```cmake
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
```

**New (Qt 6 with fallback):**
```cmake
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets)
```

**Files affected (13 files):**
- `veranda/Packages/core/pkg-veranda_core_api/CMakeLists.txt`
- `veranda/Packages/core/pkg-veranda_core/CMakeLists.txt`
- `veranda/Packages/builtin_plugin_logic/pkg-veranda_builtin_sensors/CMakeLists.txt`
- `veranda/Packages/builtin_plugin_logic/pkg-veranda_builtin_wheels/CMakeLists.txt`
- `veranda/Packages/builtin_plugin_logic/pkg-veranda_builtin_shapes/CMakeLists.txt`
- `veranda/Packages/qt_frontend/pkg-veranda_qt_frontend/CMakeLists.txt`
- `veranda/Packages/qt_frontend/builtin_qt_frontend_plugins/pkg-veranda_qt_frontend_json_file_handler/CMakeLists.txt`
- `veranda/Packages/qt_frontend/builtin_qt_frontend_plugins/pkg-veranda_qt_frontend_image_file_handler/CMakeLists.txt`
- `veranda/Packages/qt_frontend/builtin_qt_frontend_plugins/pkg-veranda_qt_frontend_default_robot_file_handler/CMakeLists.txt`
- `veranda/Packages/qt_plugins/pkg-veranda_builtin_plugin_qt_wrappers/CMakeLists.txt`
- `veranda/Packages/qt_plugins/pkg-veranda_qt_plugin_api/CMakeLists.txt` (if exists)
- `veranda/Packages/qt_frontend/pkg-veranda_qt_frontend_plugin_api/CMakeLists.txt` (if exists)
- `veranda/Packages/libraries/pkg-veranda_catch2/include/catch2/ament_cmake_add_catch_test.cmake`

#### 1.2 Replace Deprecated `qt5_wrap_cpp()` with AUTOMOC

**Old (Qt 5):**
```cmake
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})
add_library(mylib ${CPP_SRCS} ${MOC_SRCS})
```

**New (Qt 6 - Recommended: Use AUTOMOC):**
```cmake
set(CMAKE_AUTOMOC ON)
add_library(mylib ${CPP_SRCS} ${MOC_HDRS})
```

**Alternative (Qt 6 - Explicit wrap):**
```cmake
qt_wrap_cpp(MOC_SRCS ${MOC_HDRS})
# Or version-specific:
qt6_wrap_cpp(MOC_SRCS ${MOC_HDRS})
```

**Files affected:**
- All CMakeLists.txt files that use `qt5_wrap_cpp()`

#### 1.3 Replace Deprecated `qt5_use_modules()`

**Old (Qt 5 - deprecated in 5.11):**
```cmake
qt5_use_modules(${PROJECT_NAME} Core Gui Widgets)
```

**New (Qt 6):**
```cmake
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
)
```

**Files affected (12 files):**
- `pkg-veranda_core_api/CMakeLists.txt`
- `pkg-veranda_core/CMakeLists.txt`
- `pkg-veranda_builtin_sensors/CMakeLists.txt`
- `pkg-veranda_builtin_wheels/CMakeLists.txt`
- `pkg-veranda_builtin_shapes/CMakeLists.txt`
- `pkg-veranda_qt_frontend/CMakeLists.txt`
- `pkg-veranda_qt_frontend_json_file_handler/CMakeLists.txt`
- `pkg-veranda_qt_frontend_image_file_handler/CMakeLists.txt`
- `pkg-veranda_qt_frontend_default_robot_file_handler/CMakeLists.txt`
- `pkg-veranda_builtin_plugin_qt_wrappers/CMakeLists.txt`
- `ament_cmake_add_catch_test.cmake`

#### 1.4 Replace `qt5_wrap_ui()` and `qt5_add_resources()`

**Old:**
```cmake
qt5_wrap_ui(UI_SRCS ${UI_FILES})
qt5_add_resources(RCC_SRCS ${RCC_FILES})
```

**New:**
```cmake
qt_wrap_ui(UI_SRCS ${UI_FILES})
qt_add_resources(RCC_SRCS ${RCC_FILES})
# Or use AUTOUIC and AUTORCC:
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
```

**Files affected:**
- `pkg-veranda_qt_frontend/CMakeLists.txt`

#### 1.5 C++ Standard Requirement

Qt 6 requires C++17:

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

---

### 2. QRegExp → QRegularExpression Migration

`QRegExp` has been removed from Qt Core in Qt 6 and moved to Qt5Compat module.

**Current usage in codebase:**
```cpp
// File: pkg-veranda_qt_frontend/src/ui/mode_controller.cpp:333
QString strName = item->data(Qt::DisplayRole).toString().section(QRegExp("\\s+"), 0, 0, QString::SectionSkipEmpty);
```

**Option A: Migrate to QRegularExpression (Recommended)**

```cpp
#include <QRegularExpression>

// Old:
QString strName = item->data(Qt::DisplayRole).toString().section(QRegExp("\\s+"), 0, 0, QString::SectionSkipEmpty);

// New:
QString strName = item->data(Qt::DisplayRole).toString().section(QRegularExpression("\\s+"), 0, 0, QString::SectionSkipEmpty);
```

**Option B: Use Qt5Compat module (Temporary)**

Add to CMakeLists.txt:
```cmake
find_package(Qt6 REQUIRED COMPONENTS Core5Compat)
target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Core5Compat)
```

**Note**: Option A is strongly recommended. Qt5Compat is for temporary compatibility only.

---

### 3. QtConcurrent::run() Signature Changes

Qt 6 changed the signature of `QtConcurrent::run()`.

**Current usage:**
```cpp
// File: pkg-veranda_qt_frontend/src/main.cpp:63
rosThread.setFuture(QtConcurrent::run([node](){rclcpp::spin(node);}));

// File: pkg-veranda_qt_frontend/src/ui/mainwindow.cpp:293
QtConcurrent::run([this, path, wl](){ ... });
```

**Qt 6 Changes:**

1. **Lambda usage** (as in the codebase): **Should work without changes** - lambdas with captures continue to work.

2. **Overloaded functions**: Won't compile. Must use lambda or static_cast:
   ```cpp
   // Won't compile in Qt 6:
   QtConcurrent::run(overloadedFunction, arg);

   // Fix with lambda:
   QtConcurrent::run([=]{ overloadedFunction(arg); });
   ```

3. **Member function pointers**: First argument must be the member function, then the object:
   ```cpp
   // Qt 5:
   QtConcurrent::run(object, &MyClass::method, arg1, arg2);

   // Qt 6:
   QtConcurrent::run(&MyClass::method, object, arg1, arg2);
   ```

**Files affected:**
- `pkg-veranda_qt_frontend/src/main.cpp`
- `pkg-veranda_qt_frontend/src/ui/mainwindow.cpp`

**Assessment**: Current lambda-based usage should be compatible. Verify during testing.

---

### 4. QTextStream Changes

Qt 6 removed `endl` manipulator for `QTextStream`. Use `Qt::endl` instead.

**Current usage:**
```cpp
// File: pkg-veranda_qt_frontend/src/main.cpp:42
QTextStream cout(stdout);
```

**Qt 6 Change:**
```cpp
// Old (Qt 5):
stream << "text" << endl;

// New (Qt 6):
stream << "text" << Qt::endl;
```

**Assessment**: Check if `endl` is used with QTextStream anywhere in the codebase. The `cout` variable is created but search shows no Qt-specific `endl` usage with it.

---

### 5. QVariant Type System Changes

Qt 6 simplified `QVariant` type handling.

**Key changes:**
- `QVariant::Type` enum is deprecated; use `QMetaType` instead
- `QVariant::type()` deprecated; use `QVariant::typeId()` or `QVariant::metaType()`

**Example migration:**
```cpp
// Old (Qt 5):
if (variant.type() == QVariant::Int) { ... }

// New (Qt 6):
if (variant.typeId() == QMetaType::Int) { ... }
```

**Files to check:**
- `pkg-veranda_core_api/include/veranda_core/api/property.h`
- Any file using `QVariant::type()`

---

### 6. Signal/Slot Connection Changes

Qt 6 requires more explicit handling for some signal/slot connections.

**Current usage patterns:**
```cpp
// These patterns should continue to work:
connect(&property, &Property::valueSet, this, &MyClass::handler);
connect(&timer, &QTimer::timeout, [&](){ ... });
```

**Potential issues:**
- Overloaded signals may require explicit casting
- Some signal signatures changed (e.g., `QComboBox::currentIndexChanged`)

---

### 7. qRegisterMetaType Changes

Qt 6 changed metatype registration.

**Current usage:**
```cpp
// File: pkg-veranda_builtin_wheels/src/omni_drive.cpp:16
qRegisterMetaType<geometry_msgs::msg::Pose2D::SharedPtr>("geometry_msgs::msg::Pose2D::SharedPtr");

// File: pkg-veranda_core/src/simulator_core.cpp:17
qRegisterMetaType<object_id>("object_id");
```

**Qt 6 Changes:**
- Registration is often automatic for types declared with `Q_DECLARE_METATYPE`
- String-based registration still works but may give warnings
- Consider using `qRegisterMetaType<T>()` without the string argument

**Files affected:**
- `pkg-veranda_builtin_wheels/src/fixed_wheel.cpp`
- `pkg-veranda_builtin_wheels/src/encoder.cpp`
- `pkg-veranda_builtin_wheels/src/omni_drive.cpp`
- `pkg-veranda_builtin_wheels/src/ackermann_steer.cpp`
- `pkg-veranda_core/src/simulator_core.cpp`
- `pkg-veranda_qt_frontend/src/ui/mainwindow.cpp`

---

### 8. QOpenGLWidget Changes

Qt 6 has significant changes to OpenGL handling through the new Rendering Hardware Interface (RHI).

**Current usage:**
```cpp
// File: pkg-veranda_qt_frontend/include/ui/joystickprototype.h:22
class JoystickWidget : public QOpenGLWidget
```

**Qt 6 Considerations:**
- `QOpenGLWidget` still exists and works similarly
- May need to add `OpenGLWidgets` component in CMake:
  ```cmake
  find_package(Qt6 REQUIRED COMPONENTS OpenGLWidgets)
  target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::OpenGLWidgets)
  ```
- Some OpenGL functions may be accessed differently

---

### 9. Container Class Changes

Qt 6 made several changes to container classes.

**Key changes:**
- `QList` and `QVector` are now the same class (both are `QList`)
- `QLinkedList` removed (not used in codebase - OK)
- `QHash`/`QMap` iterator changes
- `QStringRef` removed (use `QStringView`)

**Current usage**: Project uses `QVector`, `QMap`, `QSet`, `QString` extensively.

**Migration notes:**
- `QVector` → `QList` (alias exists, should work)
- Check for `QVector::toList()` or `QList::toVector()` calls (no longer needed)

---

### 10. QString/QByteArray Changes

Qt 6 made some implicit conversions explicit.

**Potential issues:**
- `QString::null` removed (use `QString()` or `isNull()`)
- `QString::sprintf()` removed (use `QString::asprintf()` or `arg()`)
- `QByteArray` may require explicit conversions

**Files to audit:**
- All files using `QString` and `QByteArray` conversions

---

### 11. High DPI Changes

Qt 6 changes the default high DPI scaling behavior.

**Qt 6 default**: `Qt::HighDpiScaleFactorRoundingPolicy::PassThrough`

**Potential issues:**
- UI may look different at non-integer scale factors
- May need explicit scaling policy:
  ```cpp
  QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
      Qt::HighDpiScaleFactorRoundingPolicy::Round);
  ```

---

## Migration Strategy

### Phase 1: Preparation (Qt 5.15)

1. [ ] Update to Qt 5.15 if not already
2. [ ] Enable deprecation warnings
3. [ ] Fix all deprecation warnings
4. [ ] Update CMake to use modern patterns (AUTOMOC, target_link_libraries)

### Phase 2: CMake Updates

1. [ ] Create version-agnostic CMake find_package patterns
2. [ ] Replace all `qt5_*` functions with version-agnostic equivalents
3. [ ] Update all `qt5_use_modules()` to `target_link_libraries()`
4. [ ] Update C++ standard to C++17

### Phase 3: Code Changes

1. [ ] Migrate `QRegExp` to `QRegularExpression`
2. [ ] Verify `QtConcurrent::run()` calls
3. [ ] Audit and update `qRegisterMetaType` calls
4. [ ] Fix any `QVariant::type()` usage
5. [ ] Update any deprecated QString/QByteArray patterns

### Phase 4: Testing

1. [ ] Build with Qt 6
2. [ ] Test all UI functionality
3. [ ] Test plugin loading
4. [ ] Test ROS integration
5. [ ] Test on different DPI settings

---

## File Change Summary

### CMakeLists.txt Files Requiring Updates

| File | Changes |
|------|---------|
| `pkg-veranda_core_api/CMakeLists.txt` | find_package, qt5_wrap_cpp, qt5_use_modules |
| `pkg-veranda_core/CMakeLists.txt` | find_package, qt5_wrap_cpp, qt5_use_modules |
| `pkg-veranda_builtin_sensors/CMakeLists.txt` | find_package, qt5_wrap_cpp, qt5_use_modules |
| `pkg-veranda_builtin_wheels/CMakeLists.txt` | find_package, qt5_wrap_cpp, qt5_use_modules |
| `pkg-veranda_builtin_shapes/CMakeLists.txt` | find_package, qt5_wrap_cpp, qt5_use_modules |
| `pkg-veranda_qt_frontend/CMakeLists.txt` | find_package, qt5_*, all functions |
| `pkg-veranda_qt_frontend_json_file_handler/CMakeLists.txt` | find_package, qt5_wrap_cpp, qt5_use_modules |
| `pkg-veranda_qt_frontend_image_file_handler/CMakeLists.txt` | find_package, qt5_wrap_cpp, qt5_use_modules |
| `pkg-veranda_qt_frontend_default_robot_file_handler/CMakeLists.txt` | find_package, qt5_wrap_cpp, qt5_use_modules |
| `pkg-veranda_builtin_plugin_qt_wrappers/CMakeLists.txt` | find_package, qt5_wrap_cpp, qt5_use_modules |
| `ament_cmake_add_catch_test.cmake` | qt5_wrap_cpp, qt5_use_modules |

### Source Files Requiring Updates

| File | Changes |
|------|---------|
| `mode_controller.cpp` | QRegExp → QRegularExpression |
| `main.cpp` | Verify QtConcurrent usage, QTextStream |
| `mainwindow.cpp` | Verify QtConcurrent usage, qRegisterMetaType |
| `joystickprototype.h/cpp` | QOpenGLWidget (verify compatibility) |
| `omni_drive.cpp` | qRegisterMetaType |
| `fixed_wheel.cpp` | qRegisterMetaType |
| `encoder.cpp` | qRegisterMetaType |
| `ackermann_steer.cpp` | qRegisterMetaType |
| `simulator_core.cpp` | qRegisterMetaType |

### .pro Files (Optional Update)

The .pro files are for IDE support only, but should be updated for completeness:

```qmake
# Add Qt 6 compatibility
greaterThan(QT_MAJOR_VERSION, 5): QT += core5compat  # If using Qt5Compat
```

---

## Platform Requirements

### Qt 6 Supported Platforms

| Platform | Status |
|----------|--------|
| Ubuntu 22.04+ | Supported |
| Ubuntu 20.04 | Not supported (use Qt 5.15) |
| Windows 10+ | Supported |
| Windows 7/8.1 | Not supported |
| macOS 11+ | Supported |

**Note**: 32-bit platforms are not supported by Qt 6.

---

## Implementation Phases

This upgrade should be done in phases:

| Phase | Description |
|-------|-------------|
| 1. CMake updates | All CMakeLists.txt files |
| 2. QRegExp migration | mode_controller.cpp |
| 3. Other code changes | Various files |
| 4. Testing & debugging | Full application |
| 5. .pro file updates | Optional |

---

## References

- [Qt 6 Porting Guide](https://doc.qt.io/qt-6/portingguide.html)
- [Changes to Qt Core](https://doc.qt.io/qt-6/qtcore-changes-qt6.html)
- [Changes to Qt Concurrent](https://doc.qt.io/qt-6/concurrent-changes-qt6.html)
- [Qt5Compat Module](https://www.qt.io/blog/porting-from-qt-5-to-qt-6-using-qt5compat-library)
- [QRegularExpression Class](https://doc.qt.io/qt-6/qregularexpression.html)
- [CMake AUTOMOC](https://cmake.org/cmake/help/latest/prop_tgt/AUTOMOC.html)
- [Qt 6 CMake Integration](https://doc.qt.io/qt-6/cmake-manual.html)

---

## Example: Complete CMakeLists.txt Migration

**Before (Qt 5):**
```cmake
cmake_minimum_required(VERSION 3.5)
project(my_project)

set(CMAKE_CXX_STANDARD 11)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})
qt5_wrap_ui(UI_SRCS ${UI_FILES})

add_executable(${PROJECT_NAME} ${CPP_SRCS} ${MOC_SRCS} ${UI_SRCS})

qt5_use_modules(${PROJECT_NAME} Core Gui Widgets)
```

**After (Qt 5/6 Compatible):**
```cmake
cmake_minimum_required(VERSION 3.16)
project(my_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets)

add_executable(${PROJECT_NAME} ${CPP_SRCS} ${MOC_HDRS} ${UI_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Widgets
)
```
