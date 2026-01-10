# Tag Dispatch: Decentralized Specializations

## The Problem with Centralized Specializations

In the previous design, all factory specializations lived in `ros2/factory.h`:

```cpp
// channels/ros2/factory.h
template<>
inline std::unique_ptr<ILaserScan>
Ros2ChannelFactory::createMessageForTag<LaserScanTag>() {
    return std::make_unique<Ros2LaserScan>();
}

template<>
inline std::unique_ptr<IPose2D>
Ros2ChannelFactory::createMessageForTag<Pose2DTag>() {
    return std::make_unique<Ros2Pose2D>();
}

// ... need to add one for EVERY message type!
```

**Problem**: To add a new message, you must modify `ros2/factory.h` ❌

This violates the open/closed principle and makes the factory a central choke point.

---

## Solution: Decentralized Specializations

**Key insight**: Put the factory specialization in the **same header as the message implementation**!

### New Structure

```
include/veranda/
  messages/
    laser_scan.h              # ILaserScan interface only

  messages/ros2/
    laser_scan.h              # Ros2LaserScan + factory specialization
    pose_2d.h                 # Ros2Pose2D + factory specialization
    imu.h                     # Ros2IMU + factory specialization (new!)

  channels/ros2/
    factory.h                 # Just the Ros2ChannelFactory class definition
                              # NO specializations here!
```

### Message Header Contains Both Implementation AND Specialization

```cpp
// messages/ros2/laser_scan.h
#pragma once
#include "veranda/messages/laser_scan.h"
#include "veranda/channels/ros2/factory.h"  // Need factory class definition
#include <sensor_msgs/msg/laser_scan.hpp>

namespace veranda::messages::ros2 {

// 1. The message implementation
class Ros2LaserScan : public ILaserScan {
    sensor_msgs::msg::LaserScan _msg;

public:
    Ros2LaserScan() = default;
    explicit Ros2LaserScan(sensor_msgs::msg::LaserScan msg) : _msg(std::move(msg)) {}

    // Interface implementation
    double angleMin() const override { return _msg.angle_min; }
    void setAngleMin(double val) override { _msg.angle_min = val; }
    // ... other methods

    // Backend-specific access
    sensor_msgs::msg::LaserScan& native() { return _msg; }
    const sensor_msgs::msg::LaserScan& native() const { return _msg; }
};

} // namespace veranda::messages::ros2

// ============================================================================
// 2. Factory specialization - in the SAME HEADER!
// ============================================================================
namespace veranda::channels {

template<>
inline std::unique_ptr<messages::ILaserScan>
ros2::Ros2ChannelFactory::createMessageForTag<messages::LaserScanTag>()
{
    return std::make_unique<messages::ros2::Ros2LaserScan>();
}

template<>
inline std::unique_ptr<IPublisher<messages::ILaserScan>>
ros2::Ros2ChannelFactory::createPublisherForTag<messages::LaserScanTag>(
    const std::string& topic)
{
    return std::make_unique<ros2::Ros2Publisher<messages::ILaserScan>>(this->node(), topic);
}

template<>
inline std::unique_ptr<ISubscriber<messages::ILaserScan>>
ros2::Ros2ChannelFactory::createSubscriberForTag<messages::LaserScanTag>(
    const std::string& topic,
    typename ISubscriber<messages::ILaserScan>::Callback callback)
{
    return std::make_unique<ros2::Ros2Subscriber<messages::ILaserScan>>(
        this->node(), topic, std::move(callback));
}

} // namespace veranda::channels
```

**Key benefit**: All ROS2-specific code for LaserScan is in ONE file!

---

## The Factory Header is Now Minimal

```cpp
// channels/ros2/factory.h
#pragma once
#include "veranda/channels/channel_factory.h"
#include <rclcpp/rclcpp.hpp>

namespace veranda::channels::ros2 {

/**
 * @brief ROS2 implementation of channel factory
 *
 * Template specializations are provided by individual message headers.
 * This class just holds the ROS node.
 */
class Ros2ChannelFactory : public IChannelFactory {
    rclcpp::Node::SharedPtr _node;

public:
    explicit Ros2ChannelFactory(rclcpp::Node::SharedPtr node)
        : _node(std::move(node)) {}

    rclcpp::Node::SharedPtr node() const { return _node; }
};

} // namespace veranda::channels::ros2

// NO specializations here!
// They come from message headers like messages/ros2/laser_scan.h
```

That's it! The factory is just a simple class that holds the ROS node.

---

## Adding a New Message Type

Now adding a new message requires **zero modifications** to existing files:

### Step 1: Define the Interface (new file)

```cpp
// messages/imu.h
#pragma once
#include "message_interface.h"

namespace veranda::messages {

struct IMUTag {};

class IIMU : public MessageInterface<IMUTag> {
public:
    virtual double accelX() const = 0;
    virtual double accelY() const = 0;
    virtual double accelZ() const = 0;
    virtual double gyroX() const = 0;
    virtual double gyroY() const = 0;
    virtual double gyroZ() const = 0;

    virtual void setAccelX(double val) = 0;
    virtual void setAccelY(double val) = 0;
    virtual void setAccelZ(double val) = 0;
    virtual void setGyroX(double val) = 0;
    virtual void setGyroY(double val) = 0;
    virtual void setGyroZ(double val) = 0;
};

} // namespace veranda::messages
```

### Step 2: Add Trait Mapping (modify ONE line in message_traits.h)

```cpp
// messages/message_traits.h
template<> struct TagToInterface<IMUTag> { using type = IIMU; };
```

### Step 3: Implement ROS2 Backend (new file)

```cpp
// messages/ros2/imu.h
#pragma once
#include "veranda/messages/imu.h"
#include "veranda/channels/ros2/factory.h"
#include <sensor_msgs/msg/imu.hpp>

namespace veranda::messages::ros2 {

class Ros2IMU : public IIMU {
    sensor_msgs::msg::Imu _msg;

public:
    Ros2IMU() = default;
    explicit Ros2IMU(sensor_msgs::msg::Imu msg) : _msg(std::move(msg)) {}

    double accelX() const override { return _msg.linear_acceleration.x; }
    double accelY() const override { return _msg.linear_acceleration.y; }
    double accelZ() const override { return _msg.linear_acceleration.z; }
    double gyroX() const override { return _msg.angular_velocity.x; }
    double gyroY() const override { return _msg.angular_velocity.y; }
    double gyroZ() const override { return _msg.angular_velocity.z; }

    void setAccelX(double val) override { _msg.linear_acceleration.x = val; }
    void setAccelY(double val) override { _msg.linear_acceleration.y = val; }
    void setAccelZ(double val) override { _msg.linear_acceleration.z = val; }
    void setGyroX(double val) override { _msg.angular_velocity.x = val; }
    void setGyroY(double val) override { _msg.angular_velocity.y = val; }
    void setGyroZ(double val) override { _msg.angular_velocity.z = val; }

    sensor_msgs::msg::Imu& native() { return _msg; }
    const sensor_msgs::msg::Imu& native() const { return _msg; }
};

} // namespace veranda::messages::ros2

// ============================================================================
// Factory specializations - self-contained in this header!
// ============================================================================
namespace veranda::channels {

template<>
inline std::unique_ptr<messages::IIMU>
ros2::Ros2ChannelFactory::createMessageForTag<messages::IMUTag>()
{
    return std::make_unique<messages::ros2::Ros2IMU>();
}

template<>
inline std::unique_ptr<IPublisher<messages::IIMU>>
ros2::Ros2ChannelFactory::createPublisherForTag<messages::IMUTag>(
    const std::string& topic)
{
    return std::make_unique<ros2::Ros2Publisher<messages::IIMU>>(this->node(), topic);
}

template<>
inline std::unique_ptr<ISubscriber<messages::IIMU>>
ros2::Ros2ChannelFactory::createSubscriberForTag<messages::IMUTag>(
    const std::string& topic,
    typename ISubscriber<messages::IIMU>::Callback callback)
{
    return std::make_unique<ros2::Ros2Subscriber<messages::IIMU>>(
        this->node(), topic, std::move(callback));
}

} // namespace veranda::channels
```

**Done!** No modifications to `ros2/factory.h` or any other existing file.

---

## Selective Inclusion

You only include the message implementations you actually use:

### Minimal ROS2 Build (only uses LaserScan and Pose2D)

```cpp
// main.cpp
#include "veranda/channels/ros2/factory.h"      // Just the factory class
#include "veranda/messages/ros2/laser_scan.h"  // Brings in LaserScan specialization
#include "veranda/messages/ros2/pose_2d.h"     // Brings in Pose2D specialization
// Don't include imu.h - IMU support not linked in!

int main() {
    auto factory = std::make_shared<Ros2ChannelFactory>(node);

    // Works - LaserScan specialization is available
    auto scan = factory->createMessage<ILaserScan>();

    // Works - Pose2D specialization is available
    auto pose = factory->createMessage<IPose2D>();

    // Compile error - IMU specialization not available!
    // auto imu = factory->createMessage<IIMU>();
}
```

### Full ROS2 Build (all messages)

```cpp
// main.cpp
#include "veranda/channels/ros2/factory.h"
#include "veranda/messages/ros2/laser_scan.h"
#include "veranda/messages/ros2/pose_2d.h"
#include "veranda/messages/ros2/imu.h"          // Now IMU is available
#include "veranda/messages/ros2/twist_2d.h"
// ... include only what you need

int main() {
    auto factory = std::make_shared<Ros2ChannelFactory>(node);

    // All message types work because all specializations are included
    auto scan = factory->createMessage<ILaserScan>();
    auto pose = factory->createMessage<IPose2D>();
    auto imu = factory->createMessage<IIMU>();
}
```

---

## How It Works: Include Order

### The Dependency Chain

```
Component source file
    ↓ includes
veranda/messages/ros2/laser_scan.h
    ↓ includes
veranda/channels/ros2/factory.h      (factory class definition)
    ↓ includes
veranda/channels/channel_factory.h   (base class with template methods)
```

When `messages/ros2/laser_scan.h` is included:
1. It gets the `Ros2ChannelFactory` class definition
2. It can now provide specializations of `Ros2ChannelFactory::createMessageForTag<LaserScanTag>`
3. These specializations are inline, so they're available in every translation unit that includes this header

### Template Specialization Mechanics

The base class declares the template method:

```cpp
// channels/channel_factory.h
class IChannelFactory {
protected:
    template<typename Tag>
    std::unique_ptr<messages::TagToInterface_t<Tag>>
    createMessageForTag();
    // No implementation - will be specialized
};
```

Each message header provides a specialization:

```cpp
// messages/ros2/laser_scan.h
template<>
inline std::unique_ptr<ILaserScan>
Ros2ChannelFactory::createMessageForTag<LaserScanTag>() {
    return std::make_unique<Ros2LaserScan>();
}
```

The compiler collects all specializations from all included headers and links them together.

---

## Comparison: Before vs After

### Before (Centralized)

```
messages/ros2/laser_scan.h    → Implementation only
messages/ros2/pose_2d.h       → Implementation only
messages/ros2/imu.h           → Implementation only
                                       ↓
                               All specializations
                                       ↓
channels/ros2/factory.h       → ALL specializations here ❌
```

To add IMU: modify `ros2/factory.h` to add specialization ❌

### After (Decentralized)

```
messages/ros2/laser_scan.h    → Implementation + specialization ✅
messages/ros2/pose_2d.h       → Implementation + specialization ✅
messages/ros2/imu.h           → Implementation + specialization ✅

channels/ros2/factory.h       → Just factory class, NO specializations ✅
```

To add IMU: create `messages/ros2/imu.h` with everything self-contained ✅

---

## Benefits

### ✅ True Extensibility
- Add new message: create ONE new file
- No modifications to factory headers
- No central registry to update

### ✅ Selective Compilation
- Only include message implementations you use
- Smaller binary size
- Faster compilation (don't parse unused messages)

### ✅ Clear Organization
- Each message type is self-contained
- Implementation and factory integration in same file
- Easy to find where a message is defined

### ✅ Plugin-Like Architecture
- Message implementations are like plugins
- Including the header "registers" the message with the factory
- Can conditionally compile message support

### ✅ No Circular Dependencies
- Message headers include factory header (factory class only)
- Factory header doesn't need to know about messages
- Clean dependency graph

---

## Conditional Compilation Example

You can even conditionally compile message support:

```cpp
// messages/ros2/all_messages.h
#pragma once
#include "veranda/channels/ros2/factory.h"

// Always include basic messages
#include "laser_scan.h"
#include "pose_2d.h"

// Optional: advanced sensors
#ifdef VERANDA_WITH_IMU
#include "imu.h"
#endif

#ifdef VERANDA_WITH_CAMERA
#include "camera_image.h"
#endif

#ifdef VERANDA_WITH_GPS
#include "gps.h"
#endif
```

```cmake
# CMakeLists.txt
if(BUILD_WITH_IMU_SUPPORT)
    target_compile_definitions(veranda PUBLIC VERANDA_WITH_IMU)
endif()
```

Now you can enable/disable entire message types at build time!

---

## Complete File Listing

### Core Interfaces (no backend dependency)

```
include/veranda/messages/
    message_interface.h       # MessageInterface<Tag> base
    message_tags.h           # All tag definitions (LaserScanTag, etc.)
    message_traits.h         # Tag ↔ Interface mappings
    laser_scan.h             # ILaserScan interface
    pose_2d.h                # IPose2D interface
    imu.h                    # IIMU interface
```

### ROS2 Backend

```
include/veranda/messages/ros2/
    laser_scan.h             # Ros2LaserScan + factory specializations
    pose_2d.h                # Ros2Pose2D + factory specializations
    imu.h                    # Ros2IMU + factory specializations

include/veranda/channels/ros2/
    factory.h                # Ros2ChannelFactory class only
    publisher.h              # Ros2Publisher<T> template
    subscriber.h             # Ros2Subscriber<T> template
```

### Mock Backend

```
include/veranda/messages/mock/
    laser_scan.h             # MockLaserScan + factory specializations
    pose_2d.h                # MockPose2D + factory specializations
    imu.h                    # MockIMU + factory specializations

include/veranda/channels/mock/
    factory.h                # MockChannelFactory class only
    publisher.h              # MockPublisher<T> template
    subscriber.h             # MockSubscriber<T> template
```

---

## Summary

**Key change**: Move factory specializations from `channels/ros2/factory.h` into individual message headers.

**Benefits**:
- ✅ Adding new message = create ONE new file
- ✅ No modifications to factory code
- ✅ Selective compilation (only include what you use)
- ✅ Self-contained message implementations
- ✅ Plugin-like extensibility

**How it works**:
- Each message header includes the factory header (just class definition)
- Each message header provides template specializations for that message
- Compiler collects all specializations from included headers
- Factory doesn't need to know about messages - messages know about factory

This achieves the original goal: **easy to add new message types without modifying the messaging layer!**
