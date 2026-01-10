# Design Alternatives: Component-ROS Separation Without Type Erasure

## Overview

This document explores alternative architectures for separating components from ROS that **eliminate type erasure** while maintaining extensibility. The goal is to avoid a generic `IMessage` base class and instead use specific message interfaces with compile-time dispatch.

> **Related**: [TODO_component_ros_separation.md](TODO_component_ros_separation.md) - Original design with type erasure

## Problem with Type Erasure

The original design used `IMessage` as a type-erased base:

```cpp
class IMessage {
    virtual const std::type_info& type() const = 0;
    template<typename T> const T& as() const { ... }  // Runtime cast
};
```

**Issues**:
- Runtime type checking
- Potential casting errors
- Loss of type information
- Performance overhead from virtual calls and RTTI
- Less expressive APIs (generic message instead of specific types)

## Design Goals

1. ✅ **No type erasure**: Each message type has its own interface
2. ✅ **Extensibility**: Easy to add new message types without modifying messaging layer
3. ✅ **Compile-time dispatch**: No runtime type checks
4. ✅ **Selective compilation**: Can enable/disable backends per message type
5. ✅ **Clean dependencies**: Messaging layer doesn't need to know all message types

## Alternative Architectures

### Option 1: Interface Segregation with Template Specialization

Each message gets its own interface, and backends are implemented via template specialization.

#### Core Concept

```cpp
// Each message type has its own interface
class ILaserScan {
public:
    virtual ~ILaserScan() = default;

    virtual double angleMin() const = 0;
    virtual double angleMax() const = 0;
    virtual double angleIncrement() const = 0;
    virtual std::span<const float> ranges() const = 0;

    virtual void setAngleMin(double val) = 0;
    virtual void setAngleMax(double val) = 0;
    virtual void setAngleIncrement(double val) = 0;
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

// Generic publisher interface (no IMessage!)
template<typename MessageInterface>
class IPublisher {
public:
    virtual ~IPublisher() = default;
    virtual void publish(const MessageInterface& msg) = 0;
    virtual std::string topic() const = 0;
};

template<typename MessageInterface>
class ISubscriber {
public:
    using Callback = std::function<void(std::shared_ptr<const MessageInterface>)>;

    virtual ~ISubscriber() = default;
    virtual std::string topic() const = 0;
};
```

#### Channel Factory

```cpp
// Factory uses template methods for each message type
class IChannelFactory {
public:
    virtual ~IChannelFactory() = default;

    // Specific methods for each message type (NO template!)
    virtual std::unique_ptr<IPublisher<ILaserScan>>
        createLaserScanPublisher(const std::string& topic) = 0;

    virtual std::unique_ptr<IPublisher<IPose2D>>
        createPose2DPublisher(const std::string& topic) = 0;

    virtual std::unique_ptr<ISubscriber<IPose2D>>
        createPose2DSubscriber(const std::string& topic,
                              ISubscriber<IPose2D>::Callback cb) = 0;

    // ... one method per message type
};
```

#### Backend Implementations

```cpp
// ROS2 implementation of LaserScan interface
class Ros2LaserScan : public ILaserScan {
    sensor_msgs::msg::LaserScan _msg;

public:
    Ros2LaserScan() = default;
    explicit Ros2LaserScan(const sensor_msgs::msg::LaserScan& msg) : _msg(msg) {}

    double angleMin() const override { return _msg.angle_min; }
    void setAngleMin(double val) override { _msg.angle_min = val; }
    // ... other getters/setters

    const sensor_msgs::msg::LaserScan& native() const { return _msg; }
};

// ROS2 publisher for LaserScan
class Ros2LaserScanPublisher : public IPublisher<ILaserScan> {
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _pub;

public:
    void publish(const ILaserScan& msg) override {
        // Downcast to concrete type (safe - we created it)
        auto& ros2Msg = static_cast<const Ros2LaserScan&>(msg);
        _pub->publish(ros2Msg.native());
    }
};

// ROS2 factory
class Ros2ChannelFactory : public IChannelFactory {
    rclcpp::Node::SharedPtr _node;

public:
    std::unique_ptr<IPublisher<ILaserScan>>
    createLaserScanPublisher(const std::string& topic) override {
        auto pub = std::make_unique<Ros2LaserScanPublisher>();
        pub->_pub = _node->create_publisher<sensor_msgs::msg::LaserScan>(topic, 10);
        return pub;
    }

    // Similar for other message types
};
```

#### Component Usage

```cpp
class Lidar_Sensor {
    std::shared_ptr<IChannelFactory> _factory;
    std::unique_ptr<IPublisher<ILaserScan>> _publisher;

    void connectChannels() {
        _publisher = _factory->createLaserScanPublisher("/scan");
    }

    void worldTicked(double dt) {
        // Create message using the factory's message implementation
        auto msg = _factory->createLaserScanMessage();  // Returns unique_ptr<ILaserScan>

        msg->setAngleMin(_minAngle);
        msg->setAngleMax(_maxAngle);
        // ... configure message

        _publisher->publish(*msg);
    }
};
```

**Pros**:
- No type erasure - each message has typed interface
- Type-safe at component level
- Clear API for each message type

**Cons**:
- Factory must have method for EVERY message type
- Adding new message requires modifying factory interface
- Can't use templates for generic `create<T>()` pattern

---

### Option 2: CRTP with Compile-Time Registration

Use CRTP to allow message types to "register" their backend implementations at compile time.

#### Core Concept

```cpp
// Message interface uses CRTP
template<typename Derived>
class MessageInterface {
public:
    const Derived& derived() const { return static_cast<const Derived&>(*this); }
    Derived& derived() { return static_cast<Derived&>(*this); }
};

// Each message interface is a CRTP base
class ILaserScan : public MessageInterface<ILaserScan> {
public:
    virtual double angleMin() const = 0;
    virtual void setAngleMin(double val) = 0;
    // ... other interface methods
};

// Backend implementations specialize based on message type
template<typename Backend, typename MessageInterface>
class BackendMessage;

// ROS2 specialization for LaserScan
template<>
class BackendMessage<Ros2Backend, ILaserScan> : public ILaserScan {
    sensor_msgs::msg::LaserScan _msg;

public:
    double angleMin() const override { return _msg.angle_min; }
    void setAngleMin(double val) override { _msg.angle_min = val; }

    // Backend-specific accessor
    const sensor_msgs::msg::LaserScan& native() const { return _msg; }
};

// Publisher implementation
template<typename Backend, typename MessageInterface>
class BackendPublisher {
    // Implementation depends on Backend and MessageInterface
};

template<>
class BackendPublisher<Ros2Backend, ILaserScan> : public IPublisher<ILaserScan> {
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _pub;

public:
    void publish(const ILaserScan& msg) override {
        auto& backendMsg = static_cast<const BackendMessage<Ros2Backend, ILaserScan>&>(msg);
        _pub->publish(backendMsg.native());
    }
};
```

#### Channel Factory

```cpp
// Factory is templated on backend
template<typename Backend>
class ChannelFactory : public IChannelFactory {
    Backend _backend;  // Holds backend-specific state (e.g., rclcpp::Node)

public:
    // Generic creation method - dispatches to specialization
    template<typename MessageInterface>
    std::unique_ptr<IPublisher<MessageInterface>> createPublisher(const std::string& topic) {
        return std::make_unique<BackendPublisher<Backend, MessageInterface>>(_backend, topic);
    }

    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage() {
        return std::make_unique<BackendMessage<Backend, MessageInterface>>();
    }
};

// Backend tag types
struct Ros2Backend {
    rclcpp::Node::SharedPtr node;
};

struct MockBackend {
    // Mock-specific state
};
```

#### Component Usage

```cpp
class Lidar_Sensor {
    std::shared_ptr<IChannelFactory> _factory;
    std::unique_ptr<IPublisher<ILaserScan>> _publisher;

    void connectChannels() {
        // Type-safe, compile-time dispatch
        _publisher = _factory->createPublisher<ILaserScan>("/scan");
    }

    void worldTicked(double dt) {
        auto msg = _factory->createMessage<ILaserScan>();
        msg->setAngleMin(_minAngle);
        _publisher->publish(*msg);
    }
};
```

**Pros**:
- Generic `create<T>()` API
- Compile-time dispatch via template specialization
- Easy to add new messages - just specialize the template
- Factory doesn't need to know all message types

**Cons**:
- Factory interface still needs virtual methods (can't have virtual template methods)
- Still need one method per message type in abstract factory interface
- Complex template metaprogramming

---

### Option 3: Header-Only Backend Selection (Recommended!)

Each message interface is defined independently. Backend implementations are in separate headers that you include to "enable" that backend for that message.

#### Project Structure

```
include/veranda/messages/
    laser_scan.h          # ILaserScan interface only
    pose_2d.h             # IPose2D interface only
    twist_2d.h            # ITwist2D interface only

include/veranda/messages/ros2/
    laser_scan.h          # ROS2 implementation of ILaserScan
    pose_2d.h             # ROS2 implementation of IPose2D
    twist_2d.h            # ROS2 implementation of ITwist2D

include/veranda/messages/mock/
    laser_scan.h          # Mock implementation of ILaserScan
    pose_2d.h             # Mock implementation of IPose2D
```

#### Message Interface (messages/laser_scan.h)

```cpp
#pragma once
#include <span>
#include <memory>

namespace veranda::messages {

class ILaserScan {
public:
    virtual ~ILaserScan() = default;

    virtual double angleMin() const = 0;
    virtual double angleMax() const = 0;
    virtual double angleIncrement() const = 0;
    virtual std::span<const float> ranges() const = 0;

    virtual void setAngleMin(double val) = 0;
    virtual void setAngleMax(double val) = 0;
    virtual void setAngleIncrement(double val) = 0;
    virtual void setRanges(std::span<const float> ranges) = 0;

    // Factory method - implemented by backend
    static std::unique_ptr<ILaserScan> create();
};

} // namespace veranda::messages
```

#### Backend Implementation (messages/ros2/laser_scan.h)

```cpp
#pragma once
#include "veranda/messages/laser_scan.h"
#include <sensor_msgs/msg/laser_scan.hpp>

namespace veranda::messages::ros2 {

class Ros2LaserScan : public ILaserScan {
    sensor_msgs::msg::LaserScan _msg;
    std::vector<float> _rangesCache;  // For span storage

public:
    Ros2LaserScan() = default;
    explicit Ros2LaserScan(sensor_msgs::msg::LaserScan msg)
        : _msg(std::move(msg)) {}

    double angleMin() const override { return _msg.angle_min; }
    double angleMax() const override { return _msg.angle_max; }
    double angleIncrement() const override { return _msg.angle_increment; }
    std::span<const float> ranges() const override { return _msg.ranges; }

    void setAngleMin(double val) override { _msg.angle_min = val; }
    void setAngleMax(double val) override { _msg.angle_max = val; }
    void setAngleIncrement(double val) override { _msg.angle_increment = val; }
    void setRanges(std::span<const float> ranges) override {
        _msg.ranges.assign(ranges.begin(), ranges.end());
    }

    // Backend-specific access
    sensor_msgs::msg::LaserScan& native() { return _msg; }
    const sensor_msgs::msg::LaserScan& native() const { return _msg; }
};

} // namespace veranda::messages::ros2

// Provide implementation of factory method when this header is included
namespace veranda::messages {
    inline std::unique_ptr<ILaserScan> ILaserScan::create() {
        return std::make_unique<ros2::Ros2LaserScan>();
    }
}
```

#### Publisher/Subscriber (channels/ros2/publisher.h)

```cpp
#pragma once
#include "veranda/channels/publisher.h"
#include <rclcpp/rclcpp.hpp>

namespace veranda::channels::ros2 {

// Trait to get ROS2 message type from interface
template<typename MessageInterface>
struct Ros2MessageType;

// Specialization for each message type
template<>
struct Ros2MessageType<messages::ILaserScan> {
    using type = sensor_msgs::msg::LaserScan;
    using impl_type = messages::ros2::Ros2LaserScan;
};

template<typename MessageInterface>
class Ros2Publisher : public IPublisher<MessageInterface> {
    using Ros2Msg = typename Ros2MessageType<MessageInterface>::type;
    using ImplType = typename Ros2MessageType<MessageInterface>::impl_type;

    typename rclcpp::Publisher<Ros2Msg>::SharedPtr _pub;
    std::string _topic;

public:
    Ros2Publisher(rclcpp::Node::SharedPtr node, const std::string& topic)
        : _topic(topic)
    {
        _pub = node->create_publisher<Ros2Msg>(topic, 10);
    }

    void publish(const MessageInterface& msg) override {
        // Safe downcast - we know the implementation type
        auto& impl = static_cast<const ImplType&>(msg);
        _pub->publish(impl.native());
    }

    std::string topic() const override { return _topic; }
};

} // namespace veranda::channels::ros2
```

#### Generic Channel Factory (channels/channel_factory.h)

```cpp
#pragma once
#include <memory>
#include <string>
#include <type_traits>

namespace veranda::channels {

template<typename MessageInterface>
class IPublisher {
public:
    virtual ~IPublisher() = default;
    virtual void publish(const MessageInterface& msg) = 0;
    virtual std::string topic() const = 0;
};

template<typename MessageInterface>
class ISubscriber {
public:
    using Callback = std::function<void(std::shared_ptr<const MessageInterface>)>;
    virtual ~ISubscriber() = default;
    virtual std::string topic() const = 0;
};

// Channel factory uses SFINAE to detect available implementations
class IChannelFactory {
public:
    virtual ~IChannelFactory() = default;

    // Generic template methods - enabled only if backend provides implementation
    template<typename MessageInterface>
    std::unique_ptr<IPublisher<MessageInterface>>
    createPublisher(const std::string& topic);

    template<typename MessageInterface>
    std::unique_ptr<ISubscriber<MessageInterface>>
    createSubscriber(const std::string& topic,
                    typename ISubscriber<MessageInterface>::Callback callback);

    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage() {
        return MessageInterface::create();
    }

protected:
    // Backends implement these
    virtual void* createPublisherImpl(const std::type_info& msgType,
                                     const std::string& topic) = 0;
    virtual void* createSubscriberImpl(const std::type_info& msgType,
                                      const std::string& topic,
                                      void* callback) = 0;
};

} // namespace veranda::channels
```

#### ROS2 Factory Implementation (channels/ros2/factory.h)

```cpp
#pragma once
#include "veranda/channels/channel_factory.h"
#include "veranda/channels/ros2/publisher.h"
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <functional>

namespace veranda::channels::ros2 {

class Ros2ChannelFactory : public IChannelFactory {
    rclcpp::Node::SharedPtr _node;

    // Type-erased factory functions registered at compile time
    using PublisherFactory = std::function<void*(rclcpp::Node::SharedPtr, const std::string&)>;
    std::unordered_map<std::type_index, PublisherFactory> _publisherFactories;

public:
    explicit Ros2ChannelFactory(rclcpp::Node::SharedPtr node)
        : _node(std::move(node))
    {
        // Register all known message types
        registerMessageType<messages::ILaserScan>();
        registerMessageType<messages::IPose2D>();
        // ... register others
    }

protected:
    template<typename MessageInterface>
    void registerMessageType() {
        // Publisher factory
        _publisherFactories[typeid(MessageInterface)] =
            [](rclcpp::Node::SharedPtr node, const std::string& topic) -> void* {
                return new Ros2Publisher<MessageInterface>(node, topic);
            };

        // Similar for subscribers
    }

    void* createPublisherImpl(const std::type_info& msgType,
                             const std::string& topic) override
    {
        auto it = _publisherFactories.find(std::type_index(msgType));
        if (it == _publisherFactories.end()) {
            throw std::runtime_error("Message type not registered with ROS2 backend");
        }
        return it->second(_node, topic);
    }
};

} // namespace veranda::channels::ros2

// Provide implementation for template methods
namespace veranda::channels {

template<typename MessageInterface>
std::unique_ptr<IPublisher<MessageInterface>>
IChannelFactory::createPublisher(const std::string& topic) {
    void* ptr = createPublisherImpl(typeid(MessageInterface), topic);
    return std::unique_ptr<IPublisher<MessageInterface>>(
        static_cast<IPublisher<MessageInterface>*>(ptr));
}

} // namespace veranda::channels
```

#### Component Usage

```cpp
// lidar_sensor.h
#pragma once
#include "veranda/messages/laser_scan.h"
#include "veranda/channels/channel_factory.h"

class Lidar_Sensor {
    std::shared_ptr<veranda::channels::IChannelFactory> _factory;
    std::unique_ptr<veranda::channels::IPublisher<veranda::messages::ILaserScan>> _publisher;

public:
    void connectChannels() {
        _publisher = _factory->createPublisher<veranda::messages::ILaserScan>("/scan");
    }

    void worldTicked(double dt) {
        auto msg = _factory->createMessage<veranda::messages::ILaserScan>();
        msg->setAngleMin(_minAngle);
        // ... configure
        _publisher->publish(*msg);
    }
};
```

```cpp
// For ROS2 build, include:
#include "veranda/messages/ros2/laser_scan.h"
#include "veranda/channels/ros2/factory.h"

// For testing, include:
#include "veranda/messages/mock/laser_scan.h"
#include "veranda/channels/mock/factory.h"
```

**Pros**:
- ✅ No type erasure - specific interfaces
- ✅ Generic `create<T>()` API works
- ✅ Messaging layer doesn't need to know all message types upfront
- ✅ Selective compilation - only include backends you need
- ✅ Easy to add new messages - no central registry to update
- ✅ Clean separation - message interface doesn't depend on backend
- ✅ Backends know about messages (OK per requirements)

**Cons**:
- Need to register message types with each backend factory
- Factory still needs runtime type dispatch (type_info map)
- Slight boilerplate for each message+backend combination

---

### Option 4: Concepts + Static Polymorphism (C++20)

Use C++20 concepts to define message requirements, eliminating virtual functions entirely.

#### Core Concept

```cpp
// Concept defining what a LaserScan message must provide
template<typename T>
concept LaserScanLike = requires(T msg) {
    { msg.angleMin() } -> std::convertible_to<double>;
    { msg.angleMax() } -> std::convertible_to<double>;
    { msg.setAngleMin(0.0) } -> std::same_as<void>;
    { msg.ranges() } -> std::convertible_to<std::span<const float>>;
};

// No virtual interface - just concrete types
namespace veranda::messages::ros2 {
    class LaserScan {
        sensor_msgs::msg::LaserScan _msg;
    public:
        double angleMin() const { return _msg.angle_min; }
        void setAngleMin(double val) { _msg.angle_min = val; }
        // ... satisfies LaserScanLike concept

        sensor_msgs::msg::LaserScan& native() { return _msg; }
    };
}

namespace veranda::messages::mock {
    class LaserScan {
        double _angleMin = 0;
        std::vector<float> _ranges;
    public:
        double angleMin() const { return _angleMin; }
        void setAngleMin(double val) { _angleMin = val; }
        // ... satisfies LaserScanLike concept
    };
}

// Publisher template works with any LaserScanLike type
template<LaserScanLike MsgType>
class Publisher {
    // Implementation specific to message type
};

template<>
class Publisher<veranda::messages::ros2::LaserScan> {
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _pub;
public:
    void publish(const veranda::messages::ros2::LaserScan& msg) {
        _pub->publish(msg.native());
    }
};
```

#### Component Implementation

```cpp
// Component is templated on message types
template<LaserScanLike LaserScanMsg>
class Lidar_Sensor {
    Publisher<LaserScanMsg> _publisher;

public:
    void worldTicked(double dt) {
        LaserScanMsg msg;
        msg.setAngleMin(_minAngle);
        _publisher.publish(msg);
    }
};

// Instantiate for specific backend
using Ros2LidarSensor = Lidar_Sensor<veranda::messages::ros2::LaserScan>;
using MockLidarSensor = Lidar_Sensor<veranda::messages::mock::LaserScan>;
```

**Pros**:
- ✅ Zero runtime overhead - all compile-time
- ✅ No virtual functions
- ✅ No type erasure
- ✅ Very explicit type requirements

**Cons**:
- ❌ Components must be templates - can't use polymorphism
- ❌ Entire component hierarchy becomes templated
- ❌ Can't swap backends at runtime
- ❌ Code bloat from template instantiation
- ❌ Requires C++20

---

### Option 5: Type-Indexed Factory with Explicit Registration

Combine the best of Option 3 with explicit compile-time registration to eliminate runtime type-info lookups.

#### Message Type Tags

```cpp
// Each message interface has a unique type tag
namespace veranda::messages {

struct LaserScanTag {};
struct Pose2DTag {};
struct Twist2DTag {};

// Base interface with tag
template<typename Tag>
class MessageInterface {
public:
    using type_tag = Tag;
    virtual ~MessageInterface() = default;
};

// Specific interfaces
class ILaserScan : public MessageInterface<LaserScanTag> {
public:
    virtual double angleMin() const = 0;
    virtual void setAngleMin(double val) = 0;
    // ...
};

class IPose2D : public MessageInterface<Pose2DTag> {
public:
    virtual double x() const = 0;
    virtual void setX(double val) = 0;
    // ...
};

} // namespace veranda::messages
```

#### Backend Implementation

```cpp
// Backend provides specializations for each tag
namespace veranda::messages::ros2 {

template<typename Tag>
class Ros2Message;

template<>
class Ros2Message<LaserScanTag> : public ILaserScan {
    sensor_msgs::msg::LaserScan _msg;
public:
    double angleMin() const override { return _msg.angle_min; }
    void setAngleMin(double val) override { _msg.angle_min = val; }
    // ...
    sensor_msgs::msg::LaserScan& native() { return _msg; }
};

template<>
class Ros2Message<Pose2DTag> : public IPose2D {
    geometry_msgs::msg::Pose2D _msg;
public:
    double x() const override { return _msg.x; }
    void setX(double val) override { _msg.x = val; }
    // ...
    geometry_msgs::msg::Pose2D& native() { return _msg; }
};

} // namespace veranda::messages::ros2
```

#### Factory with Tag Dispatch

```cpp
namespace veranda::channels {

// Generic publisher
template<typename MessageInterface>
class IPublisher {
public:
    virtual ~IPublisher() = default;
    virtual void publish(const MessageInterface& msg) = 0;
};

// Factory base
class IChannelFactory {
public:
    virtual ~IChannelFactory() = default;

    // Generic method - tag dispatch at compile time
    template<typename MessageInterface>
    std::unique_ptr<IPublisher<MessageInterface>>
    createPublisher(const std::string& topic) {
        return createPublisherForTag<typename MessageInterface::type_tag>(topic);
    }

    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage() {
        return createMessageForTag<typename MessageInterface::type_tag>();
    }

protected:
    // Backends implement these for each tag
    template<typename Tag>
    std::unique_ptr<IPublisher<typename TagToInterface<Tag>::type>>
    createPublisherForTag(const std::string& topic);

    template<typename Tag>
    std::unique_ptr<typename TagToInterface<Tag>::type>
    createMessageForTag();
};

// Helper trait
template<typename Tag> struct TagToInterface;
template<> struct TagToInterface<messages::LaserScanTag> { using type = messages::ILaserScan; };
template<> struct TagToInterface<messages::Pose2DTag> { using type = messages::IPose2D; };

} // namespace veranda::channels
```

#### ROS2 Factory

```cpp
namespace veranda::channels::ros2 {

class Ros2ChannelFactory : public IChannelFactory {
    rclcpp::Node::SharedPtr _node;

public:
    explicit Ros2ChannelFactory(rclcpp::Node::SharedPtr node) : _node(node) {}
};

} // namespace veranda::channels::ros2

// Provide implementations via explicit specialization
namespace veranda::channels {

template<>
inline std::unique_ptr<IPublisher<messages::ILaserScan>>
ros2::Ros2ChannelFactory::createPublisherForTag<messages::LaserScanTag>(
    const std::string& topic)
{
    return std::make_unique<ros2::Ros2Publisher<messages::ILaserScan>>(_node, topic);
}

template<>
inline std::unique_ptr<messages::ILaserScan>
ros2::Ros2ChannelFactory::createMessageForTag<messages::LaserScanTag>()
{
    return std::make_unique<messages::ros2::Ros2Message<messages::LaserScanTag>>();
}

} // namespace veranda::channels
```

**Pros**:
- ✅ No type erasure
- ✅ Generic `create<T>()` API
- ✅ Compile-time dispatch via tags
- ✅ No runtime type_info lookups
- ✅ Easy to add new messages

**Cons**:
- Still need explicit specialization for each message+backend
- Tag system adds slight complexity

---

## Comparison Matrix

| Feature | Option 1<br/>Interface Segregation | Option 2<br/>CRTP | Option 3<br/>Header Selection | Option 4<br/>Concepts | Option 5<br/>Tag Dispatch |
|---------|------------|------|-----------------|-----------|---------------|
| No type erasure | ✅ | ✅ | ✅ | ✅ | ✅ |
| Generic `create<T>()` | ❌ | ⚠️ | ✅ | ✅ | ✅ |
| Easy to add messages | ❌ | ✅ | ✅ | ✅ | ✅ |
| Runtime backend swap | ✅ | ✅ | ✅ | ❌ | ✅ |
| Selective compilation | ⚠️ | ✅ | ✅ | ✅ | ✅ |
| No virtual functions | ❌ | ❌ | ❌ | ✅ | ❌ |
| Simple implementation | ✅ | ❌ | ⚠️ | ❌ | ⚠️ |
| C++17 compatible | ✅ | ✅ | ✅ | ❌ | ✅ |

## Recommendation

**Option 3 (Header-Only Backend Selection)** is recommended as the best balance:

1. ✅ Achieves primary goal: eliminates type erasure
2. ✅ Clean API: `factory->create<ILaserScan>()`
3. ✅ Extensible: add messages without modifying factory interface
4. ✅ Flexible: can swap backends at runtime (important for tool)
5. ✅ Practical: works with C++17, reasonable complexity
6. ✅ Testable: easy to provide mock implementations

### Refinement: Eliminate Runtime Type Dispatch

To avoid the `std::type_info` map in Option 3, we can use a hybrid with tags from Option 5:

```cpp
// Each interface has a compile-time tag
class ILaserScan : public MessageInterface<LaserScanTag> {
    // ... interface methods
};

// Factory uses tag-based dispatch
template<typename MessageInterface>
std::unique_ptr<IPublisher<MessageInterface>>
IChannelFactory::createPublisher(const std::string& topic) {
    // Compile-time dispatch via tag
    return createPublisherForTag<typename MessageInterface::type_tag>(topic);
}

// Implementations provided via template specialization in backend headers
```

This gives us:
- No type erasure (specific interfaces)
- No runtime type dispatch (compile-time tags)
- Generic API (`create<ILaserScan>()`)
- Easy extensibility (add specialization in backend header)
- Selective compilation (only include needed backend headers)

## Implementation Strategy

### Phase 1: Define Message Interfaces

```
include/veranda/messages/
    message_interface.h    # Base with tag system
    laser_scan.h          # ILaserScan interface
    pose_2d.h             # IPose2D interface
    twist_2d.h            # ITwist2D interface
```

### Phase 2: ROS2 Backend

```
include/veranda/messages/ros2/
    laser_scan.h          # Ros2Message<LaserScanTag> implementation
    pose_2d.h
    twist_2d.h

include/veranda/channels/ros2/
    publisher.h           # Ros2Publisher<T> template
    subscriber.h
    factory.h             # Ros2ChannelFactory with specializations
```

### Phase 3: Mock Backend

```
include/veranda/messages/mock/
    laser_scan.h          # MockMessage<LaserScanTag>
    pose_2d.h

include/veranda/channels/mock/
    publisher.h
    factory.h
```

### Phase 4: Component Migration

Migrate components to use new interfaces, one at a time.

## Open Questions

1. **Message creation**: Should components create messages via factory, or can they stack-allocate?
   - Factory: More flexible, supports custom allocators
   - Stack: Better performance, simpler code
   - **Proposal**: Factory for published messages, stack for local use

2. **Timestamp handling**: Should message interfaces include timestamp accessors?
   - Some backends (ROS2) have header with timestamp
   - Pure physics messages might not need it
   - **Proposal**: Separate `ITimestamped` mixin interface

3. **Message copying**: Should interfaces support clone()?
   - Useful for recording
   - Adds complexity
   - **Proposal**: Optional, can add later

4. **Const correctness**: Should publish() take const ref or unique_ptr?
   - Const ref: Simpler, allows reuse
   - Unique_ptr: Clearer ownership, zero-copy potential
   - **Proposal**: Start with const ref, optimize later

## Next Steps

1. Review this document and select preferred approach
2. Prototype selected approach with single message type (LaserScan)
3. Validate with one component (Lidar_Sensor) including tests
4. Iterate on API based on feedback
5. Expand to remaining message types and components
