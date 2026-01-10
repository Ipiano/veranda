# Tag Dispatch: Message Creation Flow

This document details exactly how message instance creation works with the tag dispatch system.

## The Question

When a component calls:
```cpp
auto msg = factory->createMessage<ILaserScan>();
```

How does the factory know to create a `Ros2LaserScan` instance for ROS2 or a `MockLaserScan` for testing?

## The Answer: Compile-Time Template Specialization

The factory delegates to a tag-based method, and each **backend header** provides template specializations that map tags to concrete implementation classes.

---

## Step-by-Step Compilation Flow

### Step 1: Component Code (What You Write)

```cpp
// lidar_sensor.cpp
void Lidar_Sensor::worldTicked(double dt) {
    // Create a message using the factory
    auto msg = _factory->createMessage<ILaserScan>();

    msg->setAngleMin(_minAngle);
    msg->setRanges(_ranges);

    _publisher->publish(*msg);
}
```

### Step 2: Factory Template Method (Generic Interface)

```cpp
// channels/channel_factory.h
template<typename MessageInterface>
std::unique_ptr<MessageInterface>
IChannelFactory::createMessage() {
    // Extract the compile-time tag from the interface
    using Tag = typename MessageInterface::type_tag;

    // Delegate to tag-based method
    return createMessageForTag<Tag>();
}
```

For `createMessage<ILaserScan>()`, the compiler instantiates:

```cpp
std::unique_ptr<ILaserScan>
IChannelFactory::createMessage<ILaserScan>() {
    using Tag = typename ILaserScan::type_tag;  // → LaserScanTag
    return createMessageForTag<LaserScanTag>();
}
```

### Step 3: Tag-Based Method Declaration (Base Class)

```cpp
// channels/channel_factory.h
class IChannelFactory {
protected:
    // Template method - NO implementation in base class!
    template<typename Tag>
    std::unique_ptr<messages::TagToInterface_t<Tag>>
    createMessageForTag();

    // This is a pure template declaration.
    // Implementations come from backend headers via specialization.
};
```

**Key insight**: This method has no implementation in the base class. It's a template that will be specialized by backend headers.

### Step 4: ROS2 Backend Specialization (channels/ros2/factory.h)

When you include the ROS2 backend header, it provides template specializations:

```cpp
// channels/ros2/factory.h
namespace veranda::channels {

// Specialize for LaserScanTag → creates Ros2LaserScan
template<>
inline std::unique_ptr<messages::ILaserScan>
ros2::Ros2ChannelFactory::createMessageForTag<messages::LaserScanTag>()
{
    return std::make_unique<messages::ros2::Ros2LaserScan>();
}

// Specialize for Pose2DTag → creates Ros2Pose2D
template<>
inline std::unique_ptr<messages::IPose2D>
ros2::Ros2ChannelFactory::createMessageForTag<messages::Pose2DTag>()
{
    return std::make_unique<messages::ros2::Ros2Pose2D>();
}

// Specialize for Twist2DTag → creates Ros2Twist2D
template<>
inline std::unique_ptr<messages::ITwist2D>
ros2::Ros2ChannelFactory::createMessageForTag<messages::Twist2DTag>()
{
    return std::make_unique<messages::ros2::Ros2Twist2D>();
}

} // namespace veranda::channels
```

**Key insight**: Each backend header knows the mapping from tag to concrete implementation class.

### Step 5: Compile-Time Resolution

When the compiler sees:
```cpp
factory->createMessage<ILaserScan>()
```

And `factory` is a `Ros2ChannelFactory*`, it:

1. Instantiates `createMessage<ILaserScan>()` which calls `createMessageForTag<LaserScanTag>()`
2. Looks for a specialization of `Ros2ChannelFactory::createMessageForTag<LaserScanTag>()`
3. Finds the specialization in `ros2/factory.h`
4. Generates code equivalent to: `return std::make_unique<Ros2LaserScan>();`

The final compiled code is essentially:
```cpp
std::unique_ptr<ILaserScan> msg = std::make_unique<Ros2LaserScan>();
```

**Zero runtime overhead!**

---

## Comparison: ROS2 vs Mock Backends

### ROS2 Build

```cpp
// main.cpp
#include "lidar_sensor.h"
#include "veranda/channels/ros2/factory.h"  // ← Includes ROS2 specializations

int main() {
    auto factory = std::make_shared<veranda::channels::ros2::Ros2ChannelFactory>(node);

    Lidar_Sensor sensor;
    sensor.setChannelFactory(factory);

    // This creates Ros2LaserScan because ros2/factory.h specializations are included
    sensor.worldTicked(0.016);
}
```

When `factory->createMessage<ILaserScan>()` is called:
- Compiler sees `Ros2ChannelFactory::createMessageForTag<LaserScanTag>()` specialization
- Generates: `std::make_unique<Ros2LaserScan>()`
- Returns as `unique_ptr<ILaserScan>`

### Mock Build (Testing)

```cpp
// test_lidar.cpp
#include "lidar_sensor.h"
#include "veranda/channels/mock/factory.h"  // ← Includes MOCK specializations

TEST_CASE("Lidar test") {
    auto factory = std::make_shared<veranda::channels::mock::MockChannelFactory>();

    Lidar_Sensor sensor;
    sensor.setChannelFactory(factory);

    // This creates MockLaserScan because mock/factory.h specializations are included
    sensor.worldTicked(0.016);
}
```

The **mock backend header** provides different specializations:

```cpp
// channels/mock/factory.h
namespace veranda::channels {

template<>
inline std::unique_ptr<messages::ILaserScan>
mock::MockChannelFactory::createMessageForTag<messages::LaserScanTag>()
{
    return std::make_unique<messages::mock::MockLaserScan>();
    //                       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Different implementation!
}

} // namespace veranda::channels
```

Same component code, different backend → different concrete types created!

---

## The Implementation Types

### ROS2 Implementation (messages/ros2/laser_scan.h)

```cpp
namespace veranda::messages::ros2 {

class Ros2LaserScan : public ILaserScan {
    sensor_msgs::msg::LaserScan _msg;  // Wraps native ROS2 message

public:
    Ros2LaserScan() = default;

    // Implement ILaserScan interface
    double angleMin() const override { return _msg.angle_min; }
    void setAngleMin(double val) override { _msg.angle_min = val; }
    // ... other methods

    // Backend-specific accessor
    sensor_msgs::msg::LaserScan& native() { return _msg; }
};

} // namespace veranda::messages::ros2
```

### Mock Implementation (messages/mock/laser_scan.h)

```cpp
namespace veranda::messages::mock {

class MockLaserScan : public ILaserScan {
    // Plain data members - no ROS2 dependency!
    double _angleMin = 0;
    double _angleMax = 0;
    double _angleIncrement = 0;
    std::vector<float> _ranges;

public:
    MockLaserScan() = default;

    // Implement ILaserScan interface
    double angleMin() const override { return _angleMin; }
    void setAngleMin(double val) override { _angleMin = val; }
    std::span<const float> ranges() const override { return _ranges; }
    void setRanges(std::span<const float> r) override {
        _ranges.assign(r.begin(), r.end());
    }
    // ... other methods

    // Test helper methods
    void clear() { _ranges.clear(); }
    size_t rangeCount() const { return _ranges.size(); }
};

} // namespace veranda::messages::mock
```

---

## Why This Works: The Type System Magic

### The Tag Association

```cpp
// 1. Tag definition (compile-time identifier)
struct LaserScanTag {};

// 2. Interface embeds the tag
class ILaserScan : public MessageInterface<LaserScanTag> {
    // Now: ILaserScan::type_tag = LaserScanTag
};

// 3. Helper trait for reverse lookup
template<> struct TagToInterface<LaserScanTag> {
    using type = ILaserScan;
};
```

### The Specialization Lookup

When the compiler needs to instantiate:
```cpp
Ros2ChannelFactory::createMessageForTag<LaserScanTag>()
```

It searches for matching template specializations in this order:

1. **Exact specialization** for `Ros2ChannelFactory::createMessageForTag<LaserScanTag>`
   - Found in `channels/ros2/factory.h` ✅

2. If not found, **partial specialization** (if any existed)

3. If not found, **primary template** (base class declaration)
   - But the base class has no implementation! → Compile error ❌

This forces you to provide implementations for all message types in each backend.

---

## Complete Example with Multiple Backends

### Project Structure

```
src/
  main.cpp                           # ROS2 production build

tests/
  test_lidar.cpp                     # Mock test build

include/veranda/
  messages/
    laser_scan.h                     # ILaserScan interface (no impl)
    message_interface.h              # Base with tag system
    message_traits.h                 # Tag ↔ Interface mapping

  messages/ros2/
    laser_scan.h                     # Ros2LaserScan implementation

  messages/mock/
    laser_scan.h                     # MockLaserScan implementation

  channels/
    channel_factory.h                # IChannelFactory with createMessage<T>()
    publisher.h                      # IPublisher<T> interface

  channels/ros2/
    factory.h                        # ROS2 template specializations
    publisher.h                      # Ros2Publisher<T> implementation

  channels/mock/
    factory.h                        # Mock template specializations
    publisher.h                      # MockPublisher<T> implementation
```

### Build Configuration (CMake)

```cmake
# Production library - includes ROS2 backend
add_library(veranda_ros2
    src/lidar_sensor.cpp
    # No need to list headers - they're included by source
)

target_include_directories(veranda_ros2 PUBLIC include)
target_link_libraries(veranda_ros2 PUBLIC
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
)

target_compile_definitions(veranda_ros2 PUBLIC
    VERANDA_USE_ROS2_BACKEND  # Optional - could auto-detect by includes
)

# Test library - includes Mock backend
add_library(veranda_mock
    src/lidar_sensor.cpp
    # Same source file! Different backend headers included
)

target_include_directories(veranda_mock PUBLIC include)
# No ROS2 dependencies!

target_compile_definitions(veranda_mock PUBLIC
    VERANDA_USE_MOCK_BACKEND
)
```

### Production Binary

```cpp
// main.cpp
#include "lidar_sensor.h"
#include "veranda/channels/ros2/factory.h"      // ROS2 specializations
#include "veranda/messages/ros2/laser_scan.h"  // Ros2LaserScan
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("veranda");

    // Create ROS2 factory
    auto factory = std::make_shared<veranda::channels::ros2::Ros2ChannelFactory>(node);

    Lidar_Sensor sensor;
    sensor.setChannelFactory(factory);
    sensor.connectChannels();

    // Simulation loop
    while (rclcpp::ok()) {
        sensor.worldTicked(0.016);
        // ↑ Creates Ros2LaserScan, publishes to ROS2
        rclcpp::spin_some(node);
    }
}
```

### Test Binary

```cpp
// test_lidar.cpp
#include "lidar_sensor.h"
#include "veranda/channels/mock/factory.h"      // Mock specializations
#include "veranda/messages/mock/laser_scan.h"  // MockLaserScan
#include <catch2/catch.hpp>

TEST_CASE("Lidar publishes correct scan data") {
    // Create mock factory
    auto factory = std::make_shared<veranda::channels::mock::MockChannelFactory>();

    Lidar_Sensor sensor;
    sensor.setChannelFactory(factory);
    sensor.connectChannels();

    // Tick sensor
    sensor.worldTicked(0.1);
    // ↑ Creates MockLaserScan, stores in mock publisher

    // Verify via mock backend
    auto mockPub = factory->getMockPublisher<veranda::messages::ILaserScan>("/scan");
    REQUIRE(mockPub != nullptr);
    REQUIRE(mockPub->messageCount() == 1);

    // Get the message (returns ILaserScan interface)
    auto msg = mockPub->lastMessage();
    REQUIRE(msg->ranges().size() == 360);
    REQUIRE(msg->angleMin() == Approx(-M_PI));
}
```

---

## Key Insights

### 1. Backend Headers Define the Mapping

Each backend header knows: **Tag → Concrete Class**

```cpp
// ROS2 backend knows:
LaserScanTag → Ros2LaserScan

// Mock backend knows:
LaserScanTag → MockLaserScan
```

### 2. Components Don't Know or Care

The component just uses the interface:

```cpp
auto msg = factory->createMessage<ILaserScan>();  // Don't know/care which impl
msg->setAngleMin(-M_PI);                          // Just use interface methods
publisher->publish(*msg);                          // Polymorphic publish
```

### 3. Compile-Time Selection via Include

Which backend you get is determined by **which headers you include**:

```cpp
#include "veranda/channels/ros2/factory.h"   → Ros2LaserScan
#include "veranda/channels/mock/factory.h"   → MockLaserScan
```

### 4. No Runtime Dispatch

The entire selection happens at compile time through template specialization. The generated code is equivalent to writing:

```cpp
// ROS2 build compiles to:
std::unique_ptr<ILaserScan> msg = std::make_unique<Ros2LaserScan>();

// Mock build compiles to:
std::unique_ptr<ILaserScan> msg = std::make_unique<MockLaserScan>();
```

No virtual tables for message creation, no factory function maps, no runtime type checks!

---

## Adding a New Message Type

To add support for a new message (e.g., `IIMU`):

### 1. Define Tag and Interface

```cpp
// messages/imu.h
struct IMUTag {};

class IIMU : public MessageInterface<IMUTag> {
public:
    virtual double accelX() const = 0;
    virtual void setAccelX(double val) = 0;
    // ... other methods
};
```

### 2. Add Trait Mapping

```cpp
// messages/message_traits.h
template<> struct TagToInterface<IMUTag> { using type = IIMU; };
```

### 3. Implement ROS2 Backend

```cpp
// messages/ros2/imu.h
class Ros2IMU : public IIMU {
    sensor_msgs::msg::Imu _msg;
public:
    double accelX() const override { return _msg.linear_acceleration.x; }
    void setAccelX(double val) override { _msg.linear_acceleration.x = val; }
    // ...
    sensor_msgs::msg::Imu& native() { return _msg; }
};
```

### 4. Add ROS2 Specialization

```cpp
// channels/ros2/factory.h
template<>
inline std::unique_ptr<IIMU>
ros2::Ros2ChannelFactory::createMessageForTag<IMUTag>() {
    return std::make_unique<Ros2IMU>();
}
```

### 5. Use in Component

```cpp
auto msg = factory->createMessage<IIMU>();
msg->setAccelX(9.81);
```

Done! No changes to factory base class, no central registry to update.

---

## Summary

Message creation works through **compile-time template specialization**:

1. **Public API**: `createMessage<ILaserScan>()` (type-safe, interface-based)
2. **Tag extraction**: Compiler gets `ILaserScan::type_tag` = `LaserScanTag`
3. **Tag dispatch**: Calls `createMessageForTag<LaserScanTag>()`
4. **Backend specialization**: ROS2/Mock header provides concrete implementation
5. **Result**: `unique_ptr<ILaserScan>` pointing to `Ros2LaserScan` or `MockLaserScan`

The "magic" is that:
- Each interface embeds its tag as a compile-time property
- Each backend header provides specializations mapping tags to concrete classes
- The compiler selects the right specialization based on which headers are included
- Zero runtime overhead - pure template metaprogramming
