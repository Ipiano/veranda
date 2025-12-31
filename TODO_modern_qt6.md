# Design Proposal: Qt 6 Modernization

## Overview

This document proposes upgrading Veranda from Qt 5.x to Qt 6, leveraging new features for improved performance, cleaner code, and better maintainability.

## Current State

### Qt 5 Usage Patterns

The codebase uses Qt 5 with several patterns that are deprecated or improved in Qt 6:

1. **CMake Integration**: Uses deprecated `qt5_use_modules()` and `qt5_wrap_cpp()`
2. **Signal/Slot Syntax**: Mix of string-based and pointer-based connections
3. **Container Classes**: Uses Qt containers that have changed behavior
4. **Graphics View**: Heavy use of `QGraphicsView` framework
5. **Meta-Object System**: Relies on MOC for signals/slots

### Current Dependencies

```cpp
// pkg-veranda_qt_frontend/CMakeLists.txt
set(QT_COMPONENTS Core Widgets Gui)
find_package(Qt5 REQUIRED COMPONENTS ${QT_COMPONENTS})
```

## Proposed Solution

### Target Qt Version

**Qt 6.5 LTS** or **Qt 6.6+**

Rationale:
- Qt 6.5 is current LTS with support until 2026
- Mature CMake integration
- Stable API after Qt 5→6 transition period
- Improved graphics performance

### Key Improvements

#### 1. Modern CMake Integration

**Before (Qt 5 style):**
```cmake
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})
qt5_wrap_ui(UI_SRCS ${UI_FILES})
qt5_use_modules(${PROJECT_NAME} ${QT_COMPONENTS})
```

**After (Qt 6 style):**
```cmake
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

# Let CMake handle MOC automatically
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Modern target-based linking
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)
```

#### 2. Qt 5/6 Compatibility Layer

Support both Qt versions during transition:

```cmake
# FindQt.cmake - compatibility wrapper
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets)

# Use version-agnostic targets
if(QT_VERSION_MAJOR EQUAL 6)
    set(QT_LIBRARIES Qt6::Core Qt6::Gui Qt6::Widgets)
else()
    set(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::Widgets)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${QT_LIBRARIES})
```

#### 3. Property Binding System

Qt 6 introduces a powerful property binding system that complements our existing Property class:

**Current approach:**
```cpp
// Manual signal connections for property updates
connect(&radius, &Property::valueSet, this, &Lidar_Sensor::_buildModels);
connect(&scan_points, &Property::valueSet, this, &Lidar_Sensor::_updateDataMessageDimensions);
```

**Enhanced with Qt 6 bindable properties:**
```cpp
#include <QProperty>

class Lidar_Sensor : public WorldObjectComponent {
    Q_OBJECT
    Q_PROPERTY(double radius READ radius WRITE setRadius BINDABLE bindableRadius)

private:
    Q_OBJECT_BINDABLE_PROPERTY(Lidar_Sensor, double, m_radius)

public:
    QBindable<double> bindableRadius() { return &m_radius; }

    void setupBindings() {
        // Automatic updates when dependencies change
        m_scanRange.setBinding([this]() {
            return m_radius.value() * 2.0 * M_PI * (m_angleRange.value() / 360.0);
        });
    }
};
```

#### 4. Improved Signal/Slot Connections

**Current (mixed styles):**
```cpp
// String-based (runtime checked, error-prone)
connect(sender, SIGNAL(valueChanged(int)), receiver, SLOT(onValueChanged(int)));

// Pointer-based (compile-time checked)
connect(&spinTimer, &QTimer::timeout, [&]() { /* ... */ });
```

**Qt 6 best practices:**
```cpp
// Always use pointer-to-member syntax
connect(&spinTimer, &QTimer::timeout, this, &SimulatorCore::onTimeout);

// Lambda with explicit context for proper destruction
connect(&spinTimer, &QTimer::timeout, this, [this]() {
    processTimeout();
});

// Use qOverload for overloaded signals
connect(spinBox, qOverload<int>(&QSpinBox::valueChanged),
        this, &MyClass::onValueChanged);
```

#### 5. QGraphicsView Optimizations

Qt 6 has improved graphics performance. Apply these optimizations:

```cpp
// qgraphicssimulationviewer.cpp

void QGraphicsSimulationViewer::setupOptimizations() {
    // Enable OpenGL rendering for hardware acceleration
    QOpenGLWidget* glWidget = new QOpenGLWidget();
    QSurfaceFormat format;
    format.setSamples(4);  // Anti-aliasing
    glWidget->setFormat(format);
    _viewer->setViewport(glWidget);

    // Optimization flags
    _viewer->setOptimizationFlags(
        QGraphicsView::DontSavePainterState |
        QGraphicsView::DontAdjustForAntialiasing
    );

    // Viewport update mode for animation
    _viewer->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);

    // Scene indexing for large scenes
    _scene->setItemIndexMethod(QGraphicsScene::BspTreeIndex);
    _scene->setBspTreeDepth(8);  // Tune based on object count
}
```

#### 6. Improved Container Usage

Qt 6 changes some container behaviors. Update for compatibility:

```cpp
// Qt 5: QList was a pointer array for large types
// Qt 6: QList is equivalent to QVector

// Replace QVector with QList (they're now the same)
QList<Model*> _models;  // Was QVector<Model*>

// Use ranged-for with references to avoid copies
for (const auto& model : std::as_const(_models)) {
    // ...
}

// Use QStringView for non-owning string references
void processName(QStringView name) {
    // No allocation for string literals or substrings
}
```

#### 7. High-DPI Support

Qt 6 has improved high-DPI handling:

```cpp
int main(int argc, char** argv) {
    // Qt 6 has high-DPI enabled by default
    // Explicitly set policy if needed
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
        Qt::HighDpiScaleFactorRoundingPolicy::PassThrough
    );

    QApplication app(argc, argv);
    // ...
}

// In QGraphicsView, use device-independent pixels
void QGraphicsSimulationViewer::viewZoom(int z) {
    qreal scaleFactor = 1.0 + z * 0.1;
    _viewer->scale(scaleFactor, scaleFactor);
}
```

#### 8. Concurrent Processing with QtConcurrent

Leverage Qt 6's improved concurrent utilities:

```cpp
#include <QtConcurrent>

// Parallel model updates
void SimulatorCore::updateAllModels() {
    QList<WorldObject*> objects = _worldObjects.values();

    // Process in parallel
    QtConcurrent::blockingMap(objects, [](WorldObject* obj) {
        obj->syncModels();
    });
}

// Async file loading with QFuture
QFuture<QVector<WorldObject*>> loadObjectsAsync(const QString& path) {
    return QtConcurrent::run([path]() {
        return FileLoader::loadObjects(path);
    });
}

// Usage with QFutureWatcher
void MainWindow::onLoadClicked() {
    auto future = loadObjectsAsync(filePath);

    auto* watcher = new QFutureWatcher<QVector<WorldObject*>>(this);
    connect(watcher, &QFutureWatcher<QVector<WorldObject*>>::finished,
            this, [this, watcher]() {
                addObjectsToSimulation(watcher->result());
                watcher->deleteLater();
            });
    watcher->setFuture(future);
}
```

#### 9. QML Integration (Optional Future Enhancement)

Qt 6 improves QML/C++ integration. Consider for future UI redesign:

```cpp
// Expose simulator to QML
class SimulatorBridge : public QObject {
    Q_OBJECT
    QML_ELEMENT

    Q_PROPERTY(bool running READ isRunning NOTIFY runningChanged)
    Q_PROPERTY(double simTime READ simTime NOTIFY simTimeChanged)

public:
    Q_INVOKABLE void start() { emit startRequested(); }
    Q_INVOKABLE void stop() { emit stopRequested(); }

signals:
    void runningChanged();
    void simTimeChanged();
    void startRequested();
    void stopRequested();
};

// Register in main.cpp
qmlRegisterType<SimulatorBridge>("Veranda", 1, 0, "Simulator");
```

### File-by-File Changes

#### CMakeLists.txt (all packages)

```cmake
cmake_minimum_required(VERSION 3.16)
project(veranda_core VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Auto-generate MOC, UIC, RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find Qt 6 with fallback to Qt 5
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets)

# Define library
add_library(${PROJECT_NAME} SHARED
    ${CPP_SRCS}
    ${MOC_HDRS}  # AUTOMOC handles these
)

# Modern target-based dependencies
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
)
```

#### property.h - Qt 6 Compatibility

```cpp
#pragma once

#include <QObject>
#include <QVariant>
#include <functional>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QProperty>
#endif

class Property : public QObject {
    Q_OBJECT

    // ... existing implementation ...

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
public:
    // Qt 6 bindable property support
    QBindable<QVariant> bindableValue() {
        return QBindable<QVariant>(&_bindableValue);
    }

private:
    Q_OBJECT_BINDABLE_PROPERTY(Property, QVariant, _bindableValue)
#endif
};
```

#### qgraphicssimulationviewer.cpp - Graphics Optimizations

```cpp
#include "qgraphicssimulationviewer.h"

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QOpenGLWidget>
#endif

QGraphicsSimulationViewer::QGraphicsSimulationViewer(QWidget *parent)
    : Simulator_Visual_If(parent)
{
    setupUi();
    setupOptimizations();
}

void QGraphicsSimulationViewer::setupOptimizations() {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    // Use OpenGL viewport for hardware acceleration
    auto* glWidget = new QOpenGLWidget();
    _viewer->setViewport(glWidget);
#endif

    // Common optimizations
    _viewer->setRenderHint(QPainter::Antialiasing, true);
    _viewer->setRenderHint(QPainter::SmoothPixmapTransform, true);
    _viewer->setCacheMode(QGraphicsView::CacheBackground);

    _scene->setItemIndexMethod(QGraphicsScene::BspTreeIndex);
}
```

### Migration Steps

#### Phase 1: CMake Modernization (Days 1-2)

1. Update all CMakeLists.txt to Qt 6 style
2. Add Qt 5/6 compatibility layer
3. Enable AUTOMOC, AUTOUIC, AUTORCC
4. Test build with both Qt versions

#### Phase 2: API Updates (Days 3-5)

1. Replace deprecated Qt 5 APIs
2. Update container usage (QVector → QList)
3. Fix signal/slot connection syntax
4. Update QVariant conversions (Qt 6 is stricter)

#### Phase 3: Graphics Optimizations (Days 6-7)

1. Add OpenGL viewport option
2. Implement scene indexing
3. Optimize paint events
4. Add high-DPI support

#### Phase 4: Testing (Days 8-10)

1. Verify all UI functionality
2. Performance benchmarks
3. Memory usage comparison
4. Cross-platform testing

### API Compatibility Reference

| Qt 5 API | Qt 6 Replacement | Notes |
|----------|------------------|-------|
| `QVector<T>` | `QList<T>` | Same in Qt 6 |
| `QStringRef` | `QStringView` | Non-owning view |
| `QRegExp` | `QRegularExpression` | Different API |
| `QTextCodec` | `QStringConverter` | Encoding/decoding |
| `qSort()` | `std::sort()` | Use STL algorithms |
| `QDesktopWidget` | `QScreen` | Screen info |
| `qt5_wrap_cpp()` | AUTOMOC | CMake built-in |

### Deprecation Warnings to Address

```cpp
// Warning: QList::toSet() is deprecated
// Old:
QSet<Model*> modelSet = modelList.toSet();
// New:
QSet<Model*> modelSet(modelList.begin(), modelList.end());

// Warning: QVariant::type() is deprecated
// Old:
if (v.type() == QVariant::Double)
// New:
if (v.typeId() == QMetaType::Double)

// Warning: QString::SkipEmptyParts is deprecated
// Old:
str.split(",", QString::SkipEmptyParts)
// New:
str.split(",", Qt::SkipEmptyParts)
```

### Testing Strategy

1. **Build Tests**: Compile with both Qt 5 and Qt 6
2. **Unit Tests**: Run existing tests, verify property behavior
3. **UI Tests**: Manual verification of all UI components
4. **Performance Tests**: Compare frame rates, memory usage
5. **Regression Tests**: Full simulation scenarios

### Rollback Plan

1. Maintain Qt 5 compatibility macros throughout transition
2. Keep CI building against both versions
3. Version-guard any Qt 6-only features
4. Document any behavioral differences

### Success Metrics

| Metric | Current (Qt 5) | Target (Qt 6) |
|--------|----------------|---------------|
| Build time | Baseline | <90% |
| Runtime memory | Baseline | <95% |
| Graphics FPS | Baseline | >110% |
| Startup time | Baseline | <95% |
| Binary size | Baseline | Comparable |

## Future Opportunities

With Qt 6 in place, consider these future enhancements:

1. **QML UI**: Modern, declarative UI alongside widgets
2. **Qt Quick 3D**: 3D visualization option
3. **Qt Graphs**: Data visualization for sensor output
4. **Property Bindings**: Reactive property relationships

## References

- [Qt 6 Documentation](https://doc.qt.io/qt-6/)
- [Porting to Qt 6](https://doc.qt.io/qt-6/portingguide.html)
- [Qt 6 CMake Manual](https://doc.qt.io/qt-6/cmake-manual.html)
- [What's New in Qt 6](https://doc.qt.io/qt-6/whatsnew60.html)
