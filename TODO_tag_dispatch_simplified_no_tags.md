# Simplified Design: Direct Specialization Without Tags

## The Realization

**Tags are unnecessary!** We can specialize template methods directly on the message interface type, eliminating an entire layer of indirection.

### With Tags (More Complex)

```cpp
struct LaserScanTag {};
class ILaserScan : public MessageInterface<LaserScanTag> { ... };

// Factory extracts tag, then dispatches
factory->createMessage<ILaserScan>()
  → ILaserScan::type_tag = LaserScanTag
  → createMessageForTag<LaserScanTag>()
  → specialization
```

### Without Tags (Simpler!)

```cpp
class ILaserScan { ... };  // No tag!

// Factory specializes directly on interface type
factory->createMessage<ILaserScan>()
  → createMessage<ILaserScan>() specialization
```

---

## Complete Simplified Design

### Message Interface (messages/laser_scan.h)

```cpp
#pragma once
#include <span>
#include <memory>

namespace veranda::messages {

/**
 * @brief Interface for laser scan messages
 *
 * No tag needed - the interface type itself is the "tag"!
 */
class ILaserScan {
public:
    virtual ~ILaserScan() = default;

    virtual double angleMin() const = 0;
    virtual double angleMax() const = 0;
    virtual double angleIncrement() const = 0;
    virtual double rangeMin() const = 0;
    virtual double rangeMax() const = 0;
    virtual std::span<const float> ranges() const = 0;

    virtual void setAngleMin(double val) = 0;
    virtual void setAngleMax(double val) = 0;
    virtual void setAngleIncrement(double val) = 0;
    virtual void setRangeMin(double val) = 0;
    virtual void setRangeMax(double val) = 0;
    virtual void setRanges(std::span<const float> ranges) = 0;
};

class IPose2D {
public:
    virtual ~IPose2D() = default;

    virtual double x() const = 0;
    virtual double y() const = 0;
    virtual double theta() const = 0;

    virtual void setX(double val) = 0;
    virtual void setY(double val) = 0;
    virtual void setTheta(double val) = 0;
};

} // namespace veranda::messages
```

No `MessageInterface<Tag>` base class, no tag structs - just simple interfaces!

---

### Factory Base (channels/channel_factory.h)

```cpp
#pragma once
#include <memory>
#include <string>
#include <functional>

namespace veranda::channels {

template<typename MessageInterface>
class IPublisher {
public:
    virtual ~IPublisher() = default;
    virtual void publish(const MessageInterface& msg) = 0;
    virtual std::string topic() const = 0;
    virtual bool isValid() const = 0;
};

template<typename MessageInterface>
class ISubscriber {
public:
    using Callback = std::function<void(std::shared_ptr<const MessageInterface>)>;
    virtual ~ISubscriber() = default;
    virtual std::string topic() const = 0;
    virtual bool isValid() const = 0;
};

/**
 * @brief Abstract factory for creating message channels
 *
 * Concrete factories provide template specializations for each message type
 * they support. Specializations are provided by message implementation headers,
 * not centralized in the factory.
 */
class IChannelFactory {
public:
    virtual ~IChannelFactory() = default;

    /**
     * @brief Create a message instance
     *
     * Example: auto msg = factory->createMessage<ILaserScan>();
     *
     * Template specializations provided by message headers (e.g., messages/ros2/laser_scan.h)
     */
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage();

    /**
     * @brief Create a publisher
     *
     * Example: auto pub = factory->createPublisher<ILaserScan>("/scan");
     */
    template<typename MessageInterface>
    std::unique_ptr<IPublisher<MessageInterface>>
    createPublisher(const std::string& topic);

    /**
     * @brief Create a subscriber
     */
    template<typename MessageInterface>
    std::unique_ptr<ISubscriber<MessageInterface>>
    createSubscriber(const std::string& topic,
                    typename ISubscriber<MessageInterface>::Callback callback);
};

} // namespace veranda::channels
```

**Key point**: These are template method **declarations** with no implementation. Specializations come from message headers.

---

### ROS2 Factory (channels/ros2/factory.h)

```cpp
#pragma once
#include "veranda/channels/channel_factory.h"
#include <rclcpp/rclcpp.hpp>

namespace veranda::channels::ros2 {

/**
 * @brief ROS2 channel factory
 *
 * This is just a simple class that holds the ROS node.
 * Template specializations are provided by individual message headers.
 */
class Ros2ChannelFactory : public IChannelFactory {
    rclcpp::Node::SharedPtr _node;

public:
    explicit Ros2ChannelFactory(rclcpp::Node::SharedPtr node)
        : _node(std::move(node)) {}

    rclcpp::Node::SharedPtr node() const { return _node; }
};

} // namespace veranda::channels::ros2

// NO specializations here! They come from message headers.
```

That's the entire factory - just holds the node!

---

### ROS2 Message Implementation (messages/ros2/laser_scan.h)

```cpp
#pragma once
#include "veranda/messages/laser_scan.h"
#include "veranda/channels/ros2/factory.h"
#include "veranda/channels/ros2/publisher.h"
#include "veranda/channels/ros2/subscriber.h"
#include <sensor_msgs/msg/laser_scan.hpp>

namespace veranda::messages::ros2 {

/**
 * @brief ROS2 implementation of laser scan
 */
class Ros2LaserScan : public ILaserScan {
    sensor_msgs::msg::LaserScan _msg;

public:
    Ros2LaserScan() = default;
    explicit Ros2LaserScan(sensor_msgs::msg::LaserScan msg)
        : _msg(std::move(msg)) {}

    // ILaserScan interface
    double angleMin() const override { return _msg.angle_min; }
    double angleMax() const override { return _msg.angle_max; }
    double angleIncrement() const override { return _msg.angle_increment; }
    double rangeMin() const override { return _msg.range_min; }
    double rangeMax() const override { return _msg.range_max; }

    std::span<const float> ranges() const override {
        return std::span<const float>(_msg.ranges.data(), _msg.ranges.size());
    }

    void setAngleMin(double val) override { _msg.angle_min = val; }
    void setAngleMax(double val) override { _msg.angle_max = val; }
    void setAngleIncrement(double val) override { _msg.angle_increment = val; }
    void setRangeMin(double val) override { _msg.range_min = val; }
    void setRangeMax(double val) override { _msg.range_max = val; }

    void setRanges(std::span<const float> ranges) override {
        _msg.ranges.assign(ranges.begin(), ranges.end());
    }

    // Backend-specific access
    sensor_msgs::msg::LaserScan& native() { return _msg; }
    const sensor_msgs::msg::LaserScan& native() const { return _msg; }
};

} // namespace veranda::messages::ros2

// ============================================================================
// Factory Specializations - Specialize directly on ILaserScan!
// ============================================================================

namespace veranda::channels {

// Message creation
template<>
inline std::unique_ptr<messages::ILaserScan>
ros2::Ros2ChannelFactory::createMessage<messages::ILaserScan>()
{
    return std::make_unique<messages::ros2::Ros2LaserScan>();
}

// Publisher creation
template<>
inline std::unique_ptr<IPublisher<messages::ILaserScan>>
ros2::Ros2ChannelFactory::createPublisher<messages::ILaserScan>(
    const std::string& topic)
{
    return std::make_unique<ros2::Ros2Publisher<messages::ILaserScan>>(
        this->node(), topic);
}

// Subscriber creation
template<>
inline std::unique_ptr<ISubscriber<messages::ILaserScan>>
ros2::Ros2ChannelFactory::createSubscriber<messages::ILaserScan>(
    const std::string& topic,
    typename ISubscriber<messages::ILaserScan>::Callback callback)
{
    return std::make_unique<ros2::Ros2Subscriber<messages::ILaserScan>>(
        this->node(), topic, std::move(callback));
}

} // namespace veranda::channels
```

**Key point**: Specialize `createMessage<ILaserScan>` directly - no tag extraction step!

---

### ROS2 Publisher Template (channels/ros2/publisher.h)

```cpp
#pragma once
#include "veranda/channels/publisher.h"
#include <rclcpp/rclcpp.hpp>

namespace veranda::channels::ros2 {

/**
 * @brief Trait to map message interface to ROS2 message type and implementation
 */
template<typename MessageInterface>
struct Ros2MessageTraits;

// Specialization for LaserScan
template<>
struct Ros2MessageTraits<messages::ILaserScan> {
    using ros2_msg_type = sensor_msgs::msg::LaserScan;
    using impl_type = messages::ros2::Ros2LaserScan;
};

// Specialization for Pose2D
template<>
struct Ros2MessageTraits<messages::IPose2D> {
    using ros2_msg_type = geometry_msgs::msg::Pose2D;
    using impl_type = messages::ros2::Ros2Pose2D;
};

/**
 * @brief ROS2 publisher implementation
 */
template<typename MessageInterface>
class Ros2Publisher : public IPublisher<MessageInterface> {
    using Traits = Ros2MessageTraits<MessageInterface>;
    using Ros2MsgType = typename Traits::ros2_msg_type;
    using ImplType = typename Traits::impl_type;

    typename rclcpp::Publisher<Ros2MsgType>::SharedPtr _pub;
    std::string _topic;

public:
    Ros2Publisher(rclcpp::Node::SharedPtr node, const std::string& topic)
        : _topic(topic)
    {
        _pub = node->create_publisher<Ros2MsgType>(topic, 10);
    }

    void publish(const MessageInterface& msg) override {
        // Safe downcast - we only create ImplType from our factory
        const auto& impl = static_cast<const ImplType&>(msg);
        _pub->publish(impl.native());
    }

    std::string topic() const override { return _topic; }
    bool isValid() const override { return _pub != nullptr; }
};

// Similarly for Ros2Subscriber...

} // namespace veranda::channels::ros2
```

---

## Comparison: With Tags vs Without Tags

### With Tags

```cpp
// Define tag
struct LaserScanTag {};

// Interface embeds tag
class ILaserScan : public MessageInterface<LaserScanTag> { ... };

// Trait mapping
template<> struct TagToInterface<LaserScanTag> { using type = ILaserScan; };

// Factory method extracts tag
template<typename MessageInterface>
std::unique_ptr<MessageInterface> createMessage() {
    using Tag = typename MessageInterface::type_tag;
    return createMessageForTag<Tag>();
}

// Specialization on tag
template<>
std::unique_ptr<ILaserScan>
Ros2ChannelFactory::createMessageForTag<LaserScanTag>() { ... }
```

**Complexity**: 4 layers of indirection!

### Without Tags

```cpp
// Just the interface
class ILaserScan { ... };

// Specialization directly on interface
template<>
std::unique_ptr<ILaserScan>
Ros2ChannelFactory::createMessage<ILaserScan>() { ... }
```

**Simplicity**: Direct specialization!

---

## Component Usage (Unchanged)

```cpp
// lidar_sensor.cpp
void Lidar_Sensor::connectChannels() {
    _publisher = _factory->createPublisher<ILaserScan>("/scan");
}

void Lidar_Sensor::worldTicked(double dt) {
    auto msg = _factory->createMessage<ILaserScan>();
    msg->setAngleMin(_minAngle);
    msg->setRanges(_ranges);
    _publisher->publish(*msg);
}
```

Component code is identical - the simplification is purely in the factory implementation!

---

## Adding a New Message (Even Simpler!)

### Step 1: Define Interface

```cpp
// messages/imu.h
class IIMU {
public:
    virtual double accelX() const = 0;
    virtual void setAccelX(double val) = 0;
    // ...
};
```

### Step 2: Implement for ROS2

```cpp
// messages/ros2/imu.h
#include "veranda/messages/imu.h"
#include "veranda/channels/ros2/factory.h"

namespace veranda::messages::ros2 {
    class Ros2IMU : public IIMU {
        sensor_msgs::msg::Imu _msg;
    public:
        double accelX() const override { return _msg.linear_acceleration.x; }
        void setAccelX(double val) override { _msg.linear_acceleration.x = val; }
        // ...
        sensor_msgs::msg::Imu& native() { return _msg; }
    };
}

// Specializations
namespace veranda::channels {
    template<>
    inline std::unique_ptr<messages::IIMU>
    ros2::Ros2ChannelFactory::createMessage<messages::IIMU>() {
        return std::make_unique<messages::ros2::Ros2IMU>();
    }

    template<>
    inline std::unique_ptr<IPublisher<messages::IIMU>>
    ros2::Ros2ChannelFactory::createPublisher<messages::IIMU>(const std::string& topic) {
        return std::make_unique<ros2::Ros2Publisher<messages::IIMU>>(this->node(), topic);
    }

    template<>
    inline std::unique_ptr<ISubscriber<messages::IIMU>>
    ros2::Ros2ChannelFactory::createSubscriber<messages::IIMU>(
        const std::string& topic,
        typename ISubscriber<messages::IIMU>::Callback callback)
    {
        return std::make_unique<ros2::Ros2Subscriber<messages::IIMU>>(
            this->node(), topic, std::move(callback));
    }
}
```

**Done!** No tag definitions, no trait mappings - just interface and specializations.

---

## Benefits of Eliminating Tags

### ✅ Simpler Code
- No tag structs to define
- No `MessageInterface<Tag>` base class
- No `TagToInterface` traits
- Fewer template layers

### ✅ More Direct
- Specializations directly on interface type
- Easier to understand: "create ILaserScan → Ros2LaserScan"
- Less template metaprogramming magic

### ✅ Fewer Files to Touch
Adding a message:
- With tags: Define tag, define interface, define mapping, define impl, define specializations
- Without tags: Define interface, define impl, define specializations

### ✅ Compile-Time Dispatch (Still!)
```cpp
factory->createMessage<ILaserScan>()
  ↓
Ros2ChannelFactory::createMessage<ILaserScan>() specialization
  ↓
return std::make_unique<Ros2LaserScan>();
```

Still resolved entirely at compile time - no runtime overhead!

### ✅ Same Extensibility
- Message headers still self-contained
- Factory still doesn't need to know about all messages
- Still only include what you need

---

## Why Did We Start With Tags?

Tags were introduced to create a layer of indirection, thinking it would be needed for:
1. Type erasure (eliminated - we use specific interfaces)
2. Avoiding runtime type lookups (unnecessary - template specialization handles it)

But with direct template specialization, tags are just extra complexity with no benefit!

---

## Complete File Structure

```
include/veranda/
  messages/
    laser_scan.h              # ILaserScan interface
    pose_2d.h                 # IPose2D interface
    imu.h                     # IIMU interface

  messages/ros2/
    laser_scan.h              # Ros2LaserScan + 3 factory specializations
    pose_2d.h                 # Ros2Pose2D + 3 factory specializations
    imu.h                     # Ros2IMU + 3 factory specializations

  messages/mock/
    laser_scan.h              # MockLaserScan + 3 factory specializations
    pose_2d.h                 # MockPose2D + 3 factory specializations

  channels/
    publisher.h               # IPublisher<T> template
    subscriber.h              # ISubscriber<T> template
    channel_factory.h         # IChannelFactory with template declarations

  channels/ros2/
    factory.h                 # Ros2ChannelFactory (just holds node)
    publisher.h               # Ros2Publisher<T> template + traits
    subscriber.h              # Ros2Subscriber<T> template

  channels/mock/
    factory.h                 # MockChannelFactory
    publisher.h               # MockPublisher<T> template
    subscriber.h              # MockSubscriber<T> template
```

---

## Summary

**Key simplification**: Specialize directly on the message interface type instead of using intermediate tag types.

**Before (with tags)**:
```cpp
struct LaserScanTag {};
class ILaserScan : public MessageInterface<LaserScanTag> { ... };
template<> ... createMessageForTag<LaserScanTag>() { ... }
```

**After (without tags)**:
```cpp
class ILaserScan { ... };
template<> ... createMessage<ILaserScan>() { ... }
```

**Benefits**:
- ✅ Simpler - fewer layers of indirection
- ✅ More direct - specialize on the actual type
- ✅ Fewer files to touch when adding messages
- ✅ Same compile-time dispatch
- ✅ Same extensibility
- ✅ Same performance

The tag system was unnecessary complexity. Direct template specialization is cleaner!
