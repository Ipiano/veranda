# Design Proposal: Component-ROS Infrastructure Separation

## Overview

This document proposes decoupling Veranda's component implementations from direct ROS2 dependencies, enabling unit testing without ROS2 infrastructure and supporting alternative communication backends.

**Key Design Principles**:
- ✅ **No type erasure**: Each message has specific typed interface (e.g., `ILaserScan` with `angleMin()` methods)
- ✅ **Compile-time dispatch**: Zero runtime overhead via template specialization
- ✅ **Extensible**: Add new messages without modifying factory or existing code
- ✅ **Decentralized**: Message implementations are self-contained in their own headers
- ✅ **Selective compilation**: Only include backends and messages you need
- ✅ **Optional message support**: Backends don't need to support all message types

> **Related documents**:
> - [TODO.md](TODO.md) - Phase 2.3 (Improve component testability)
> - [TODO_update_ros.md](TODO_update_ros.md) - ROS2 Jazzy upgrade (complete this first)
> - [TODO_modern_ros.md](TODO_modern_ros.md) - Modern ROS2 threading patterns
>
> **Note**: This refactor should be done **after** completing the ROS2 Jazzy upgrade to avoid doing work twice.

---

## Current State

### Tight Coupling Problem

Components directly create and manage ROS2 publishers and subscribers:

```cpp
// lidar_sensor.cpp
void Lidar_Sensor::_connectChannels() {
    disconnectChannels();

    if(_rosNode) {
        _outputChannel = output_channel.get().toString();
        if(_outputChannel.size()) {
            // Direct ROS2 dependency
            _sendChannel = _rosNode->create_publisher<sensor_msgs::msg::LaserScan>(
                _outputChannel.toStdString(), 7);
        }
    }
}

void Lidar_Sensor::_worldTicked(const double dt) {
    // ... scan logic ...
    if(_sendChannel) {
        _sendChannel->publish(data);  // Direct ROS2 call
    }
}
```

### Problems

1. **Untestable**: Cannot unit test component behavior without ROS2 runtime
2. **Tight Coupling**: Components know about `rclcpp::Node`, `rclcpp::Publisher`, etc.
3. **No Abstraction**: Cannot swap communication backend (e.g., for replay, recording)
4. **Thread Safety**: ROS2 threading concerns leak into component code
5. **Message Type Lock-in**: Components hardcode ROS2 message types

### Current Component Dependencies

```
WorldObjectComponent
    ├── rclcpp::Node (shared_ptr)
    ├── rclcpp::Publisher<T>
    ├── rclcpp::Subscription<T>
    └── sensor_msgs::msg::*, geometry_msgs::msg::*
```

---

## Proposed Solution

### Architecture Overview

Introduce a **Channel Abstraction Layer** with specific message interfaces (no type erasure):

```
┌─────────────────────────────────────────────────────────────────┐
│                        Components                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ Lidar       │  │ Omni_Drive  │  │ GPS_Sensor              │  │
│  └──────┬──────┘  └──────┬──────┘  └────────────┬────────────┘  │
│         │                │                      │                │
│         ▼                ▼                      ▼                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │         Specific Message Interfaces (No Erasure!)        │   │
│  │   ILaserScan, IPose2D, ITwist2D with typed methods       │   │
│  └──────────────────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │         Channel Interfaces (Template-based)              │   │
│  │   IPublisher<T>, ISubscriber<T>, IChannelFactory         │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
        ┌──────────┐   ┌──────────┐   ┌──────────────┐
        │ ROS2     │   │ Mock     │   │ Recording    │
        │ Backend  │   │ Backend  │   │ Backend      │
        └──────────┘   └──────────┘   └──────────────┘
```

---

## Core Interfaces

### Message Interfaces (No Type Erasure!)

Each message type has its own specific interface with typed methods:

```cpp
// include/veranda/messages/laser_scan.h
#pragma once
#include <span>

namespace veranda::messages {

/**
 * @brief Interface for laser scan messages
 *
 * Provides specific typed methods - no generic IMessage base!
 */
class ILaserScan {
public:
    virtual ~ILaserScan() = default;

    // Specific typed getters
    virtual double angleMin() const = 0;
    virtual double angleMax() const = 0;
    virtual double angleIncrement() const = 0;
    virtual double rangeMin() const = 0;
    virtual double rangeMax() const = 0;
    virtual std::span<const float> ranges() const = 0;

    // Specific typed setters
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

class ITwist2D {
public:
    virtual ~ITwist2D() = default;

    virtual double linearX() const = 0;
    virtual double linearY() const = 0;
    virtual double angularZ() const = 0;

    virtual void setLinearX(double val) = 0;
    virtual void setLinearY(double val) = 0;
    virtual void setAngularZ(double val) = 0;
};

} // namespace veranda::messages
```

**Key insight**: No `IMessage` base class, no type erasure! Each message interface is specific and type-safe.

---

### Channel Interfaces

```cpp
// include/veranda/channels/publisher.h
#pragma once
#include <memory>
#include <string>
#include <functional>

namespace veranda::channels {

/**
 * @brief Abstract publisher interface
 *
 * Templated on specific message interface (e.g., ILaserScan)
 */
template<typename MessageInterface>
class IPublisher {
public:
    virtual ~IPublisher() = default;

    virtual void publish(const MessageInterface& msg) = 0;
    virtual void publish(std::shared_ptr<const MessageInterface> msg) {
        publish(*msg);
    }

    virtual std::string topic() const = 0;
    virtual bool isValid() const = 0;
};

/**
 * @brief Abstract subscriber interface
 */
template<typename MessageInterface>
class ISubscriber {
public:
    using Callback = std::function<void(std::shared_ptr<const MessageInterface>)>;

    virtual ~ISubscriber() = default;
    virtual std::string topic() const = 0;
    virtual bool isValid() const = 0;
};

} // namespace veranda::channels
```

---

### Channel Factory

```cpp
// include/veranda/channels/channel_factory.h
#pragma once
#include "publisher.h"
#include <memory>
#include <string>

namespace veranda::channels {

/**
 * @brief Abstract factory for creating message channels
 *
 * Uses template specialization for compile-time dispatch.
 * Concrete factories provide specializations via message implementation headers.
 *
 * KEY DESIGN: This base class only declares template methods.
 * Implementations come from backend-specific message headers (e.g., messages/ros2/laser_scan.h)
 */
class IChannelFactory {
public:
    virtual ~IChannelFactory() = default;

    /**
     * @brief Create a message instance
     *
     * Example: auto msg = factory->createMessage<ILaserScan>();
     *
     * Template specializations provided by message headers.
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

**Critical design point**: These are template method **declarations** with no implementation in the base class. Specializations come from message implementation headers, not from the factory!

---

### Optional Message Support

Not all backends need to support all message types. The design handles this gracefully:

#### Support Detection Trait

```cpp
// include/veranda/channels/backend_traits.h
#pragma once
#include <type_traits>

namespace veranda::channels {

/**
 * @brief Trait to check if a backend supports a message type
 *
 * Specialize this in message implementation headers.
 * Default is false (not supported).
 */
template<typename Backend, typename MessageInterface>
struct SupportsMessage : std::false_type {};

template<typename Backend, typename MessageInterface>
inline constexpr bool SupportsMessage_v =
    SupportsMessage<Backend, MessageInterface>::value;

} // namespace veranda::channels
```

#### Updated Factory with Support Checking

```cpp
// include/veranda/channels/channel_factory.h

namespace veranda::channels {

/**
 * @brief Exception thrown when message type not supported
 */
class UnsupportedMessageException : public std::runtime_error {
public:
    UnsupportedMessageException(const std::string& backend,
                               const std::string& message)
        : std::runtime_error("Backend '" + backend +
                            "' does not support message type '" + message + "'") {}
};

class IChannelFactory {
public:
    virtual ~IChannelFactory() = default;

    /**
     * @brief Create a message instance
     *
     * Throws UnsupportedMessageException if backend doesn't support this message.
     * This indicates a programming error (forgot to include implementation header)
     * or intentional lack of support.
     */
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage() {
        return createMessageImpl<MessageInterface>();
    }

    /**
     * @brief Create a publisher
     */
    template<typename MessageInterface>
    std::unique_ptr<IPublisher<MessageInterface>>
    createPublisher(const std::string& topic) {
        return createPublisherImpl<MessageInterface>(topic);
    }

    /**
     * @brief Create a subscriber
     */
    template<typename MessageInterface>
    std::unique_ptr<ISubscriber<MessageInterface>>
    createSubscriber(const std::string& topic,
                    typename ISubscriber<MessageInterface>::Callback callback) {
        return createSubscriberImpl<MessageInterface>(topic, std::move(callback));
    }

    /**
     * @brief Check if backend supports a message type
     *
     * Use this for optional features. For required messages, just call
     * createMessage() and let it throw if unsupported.
     */
    template<typename MessageInterface>
    static constexpr bool supportsMessage() noexcept {
        return SupportsMessage_v<std::remove_pointer_t<decltype(this)>,
                                 MessageInterface>;
    }

protected:
    // Default implementations throw - overridden by specializations
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessageImpl() {
        throw UnsupportedMessageException(
            typeid(*this).name(),
            typeid(MessageInterface).name()
        );
    }

    template<typename MessageInterface>
    std::unique_ptr<IPublisher<MessageInterface>>
    createPublisherImpl(const std::string& topic) {
        throw UnsupportedMessageException(
            typeid(*this).name(),
            typeid(MessageInterface).name()
        );
    }

    template<typename MessageInterface>
    std::unique_ptr<ISubscriber<MessageInterface>>
    createSubscriberImpl(const std::string& topic,
                        typename ISubscriber<MessageInterface>::Callback callback) {
        throw UnsupportedMessageException(
            typeid(*this).name(),
            typeid(MessageInterface).name()
        );
    }
};

} // namespace veranda::channels
```

#### Usage Patterns

**For required messages (most common)**:
```cpp
void Lidar::connectChannels() {
    // LaserScan is required - just create it
    // Will throw UnsupportedMessageException if not supported
    _publisher = _factory->createPublisher<ILaserScan>("/scan");
}
```

**For optional features**:
```cpp
void Robot::configure(std::shared_ptr<IChannelFactory> factory) {
    // Required sensors
    setupLidar(factory);
    setupOdometry(factory);

    // Optional IMU - check support first
    if (factory->supportsMessage<IIMU>()) {
        setupIMU(factory);
    } else {
        useDeadReckoning();  // Fallback without IMU
    }
}
```

**In test code**:
```cpp
TEST_F(IMUTest, ReadsAcceleration) {
    auto factory = std::make_shared<MockChannelFactory>();

    // Skip test if mock backend doesn't support IMU
    if (!factory->supportsMessage<IIMU>()) {
        GTEST_SKIP() << "IMU not supported by mock backend";
    }

    auto imu = factory->createMessage<IIMU>();
    testIMU(imu);
}
```

> **See also**: [TODO_optional_message_support.md](TODO_optional_message_support.md) for detailed strategies

---

## ROS2 Backend Implementation

### ROS2 Factory (Minimal!)

```cpp
// include/veranda/channels/ros2/factory.h
#pragma once
#include "veranda/channels/channel_factory.h"
#include <rclcpp/rclcpp.hpp>

namespace veranda::channels::ros2 {

/**
 * @brief ROS2 channel factory
 *
 * This is just a simple class that holds the ROS node.
 * Template specializations for createMessage/createPublisher/createSubscriber
 * are provided by individual message headers (e.g., messages/ros2/laser_scan.h).
 *
 * This factory does NOT need to know about all message types!
 */
class Ros2ChannelFactory : public IChannelFactory {
    rclcpp::Node::SharedPtr _node;

public:
    explicit Ros2ChannelFactory(rclcpp::Node::SharedPtr node)
        : _node(std::move(node)) {}

    rclcpp::Node::SharedPtr node() const { return _node; }
};

} // namespace veranda::channels::ros2

// NO template specializations here!
// They come from message implementation headers.
```

**Key point**: The factory is tiny! Just holds the ROS node. All the specializations live in message headers.

---

### ROS2 Message Implementation (Self-Contained!)

```cpp
// include/veranda/messages/ros2/laser_scan.h
#pragma once
#include "veranda/messages/laser_scan.h"
#include "veranda/channels/ros2/factory.h"
#include "veranda/channels/ros2/publisher.h"
#include "veranda/channels/ros2/subscriber.h"
#include <sensor_msgs/msg/laser_scan.hpp>

namespace veranda::messages::ros2 {

/**
 * @brief ROS2 implementation of laser scan interface
 *
 * Wraps sensor_msgs::msg::LaserScan and implements ILaserScan.
 */
class Ros2LaserScan : public ILaserScan {
    sensor_msgs::msg::LaserScan _msg;

public:
    Ros2LaserScan() = default;
    explicit Ros2LaserScan(sensor_msgs::msg::LaserScan msg)
        : _msg(std::move(msg)) {}

    // Implement ILaserScan interface
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

    // Backend-specific access to native ROS2 message
    sensor_msgs::msg::LaserScan& native() { return _msg; }
    const sensor_msgs::msg::LaserScan& native() const { return _msg; }
};

} // namespace veranda::messages::ros2

// ============================================================================
// Support Trait and Factory Specializations - Self-contained in this header!
// ============================================================================

namespace veranda::channels {

// Declare that ROS2 backend supports LaserScan
template<>
struct SupportsMessage<ros2::Ros2ChannelFactory, messages::ILaserScan>
    : std::true_type {};

// Message creation specialization
template<>
inline std::unique_ptr<messages::ILaserScan>
ros2::Ros2ChannelFactory::createMessageImpl<messages::ILaserScan>()
{
    return std::make_unique<messages::ros2::Ros2LaserScan>();
}

// Publisher creation specialization
template<>
inline std::unique_ptr<IPublisher<messages::ILaserScan>>
ros2::Ros2ChannelFactory::createPublisherImpl<messages::ILaserScan>(
    const std::string& topic)
{
    return std::make_unique<ros2::Ros2Publisher<messages::ILaserScan>>(
        this->node(), topic);
}

// Subscriber creation specialization
template<>
inline std::unique_ptr<ISubscriber<messages::ILaserScan>>
ros2::Ros2ChannelFactory::createSubscriberImpl<messages::ILaserScan>(
    const std::string& topic,
    typename ISubscriber<messages::ILaserScan>::Callback callback)
{
    return std::make_unique<ros2::Ros2Subscriber<messages::ILaserScan>>(
        this->node(), topic, std::move(callback));
}

} // namespace veranda::channels
```

**Critical insight**: All ROS2-specific code for LaserScan (implementation + factory specializations) is in ONE file! This is the key to extensibility.

---

### ROS2 Publisher/Subscriber Templates

```cpp
// include/veranda/channels/ros2/publisher.h
#pragma once
#include "veranda/channels/publisher.h"
#include <rclcpp/rclcpp.hpp>

namespace veranda::channels::ros2 {

/**
 * @brief Trait to map message interface to ROS2 types
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
        // Safe downcast - we only create ImplType messages from our factory
        const auto& impl = static_cast<const ImplType&>(msg);
        _pub->publish(impl.native());
    }

    std::string topic() const override { return _topic; }
    bool isValid() const override { return _pub != nullptr; }
};

/**
 * @brief ROS2 subscriber implementation
 */
template<typename MessageInterface>
class Ros2Subscriber : public ISubscriber<MessageInterface> {
    using Traits = Ros2MessageTraits<MessageInterface>;
    using Ros2MsgType = typename Traits::ros2_msg_type;
    using ImplType = typename Traits::impl_type;

    typename rclcpp::Subscription<Ros2MsgType>::SharedPtr _sub;
    std::string _topic;

public:
    Ros2Subscriber(rclcpp::Node::SharedPtr node,
                   const std::string& topic,
                   typename ISubscriber<MessageInterface>::Callback callback)
        : _topic(topic)
    {
        auto ros2Callback = [cb = std::move(callback)](
            typename Ros2MsgType::SharedPtr rosMsg)
        {
            // Convert ROS2 message to our implementation
            auto msg = std::make_shared<const ImplType>(*rosMsg);
            cb(msg);
        };

        _sub = node->create_subscription<Ros2MsgType>(topic, 10, ros2Callback);
    }

    std::string topic() const override { return _topic; }
    bool isValid() const override { return _sub != nullptr; }
};

} // namespace veranda::channels::ros2
```

---

## Mock Backend Implementation

### Mock Factory

```cpp
// include/veranda/channels/mock/factory.h
#pragma once
#include "veranda/channels/channel_factory.h"
#include <map>
#include <memory>

namespace veranda::channels::mock {

/**
 * @brief Mock channel factory for testing
 *
 * Like ROS2 factory, this is minimal. Specializations come from message headers.
 */
class MockChannelFactory : public IChannelFactory {
    std::map<std::string, std::shared_ptr<void>> _publishers;
    std::map<std::string, std::shared_ptr<void>> _subscribers;

public:
    MockChannelFactory() = default;

    // Test helper: get mock publisher for verification
    template<typename MessageInterface>
    std::shared_ptr<MockPublisher<MessageInterface>>
    getMockPublisher(const std::string& topic) {
        auto it = _publishers.find(topic);
        if (it != _publishers.end()) {
            return std::static_pointer_cast<MockPublisher<MessageInterface>>(it->second);
        }
        return nullptr;
    }

    // Test helper: get mock subscriber for message injection
    template<typename MessageInterface>
    std::shared_ptr<MockSubscriber<MessageInterface>>
    getMockSubscriber(const std::string& topic) {
        auto it = _subscribers.find(topic);
        if (it != _subscribers.end()) {
            return std::static_pointer_cast<MockSubscriber<MessageInterface>>(it->second);
        }
        return nullptr;
    }

    // Internal: register publisher (called by specializations)
    template<typename MessageInterface>
    void registerPublisher(const std::string& topic,
                          std::shared_ptr<MockPublisher<MessageInterface>> pub) {
        _publishers[topic] = pub;
    }

    template<typename MessageInterface>
    void registerSubscriber(const std::string& topic,
                           std::shared_ptr<MockSubscriber<MessageInterface>> sub) {
        _subscribers[topic] = sub;
    }
};

} // namespace veranda::channels::mock
```

---

### Mock Message Implementation

```cpp
// include/veranda/messages/mock/laser_scan.h
#pragma once
#include "veranda/messages/laser_scan.h"
#include "veranda/channels/mock/factory.h"
#include "veranda/channels/mock/publisher.h"
#include "veranda/channels/mock/subscriber.h"

namespace veranda::messages::mock {

/**
 * @brief Mock implementation of laser scan
 *
 * Uses plain data members - no ROS2 dependency!
 */
class MockLaserScan : public ILaserScan {
    double _angleMin = 0;
    double _angleMax = 0;
    double _angleIncrement = 0;
    double _rangeMin = 0;
    double _rangeMax = 0;
    std::vector<float> _ranges;

public:
    MockLaserScan() = default;

    double angleMin() const override { return _angleMin; }
    double angleMax() const override { return _angleMax; }
    double angleIncrement() const override { return _angleIncrement; }
    double rangeMin() const override { return _rangeMin; }
    double rangeMax() const override { return _rangeMax; }

    std::span<const float> ranges() const override { return _ranges; }

    void setAngleMin(double val) override { _angleMin = val; }
    void setAngleMax(double val) override { _angleMax = val; }
    void setAngleIncrement(double val) override { _angleIncrement = val; }
    void setRangeMin(double val) override { _rangeMin = val; }
    void setRangeMax(double val) override { _rangeMax = val; }

    void setRanges(std::span<const float> ranges) override {
        _ranges.assign(ranges.begin(), ranges.end());
    }

    // Test helpers
    void clear() { _ranges.clear(); }
    size_t rangeCount() const { return _ranges.size(); }
};

} // namespace veranda::messages::mock

// ============================================================================
// Factory Specializations
// ============================================================================

namespace veranda::channels {

template<>
inline std::unique_ptr<messages::ILaserScan>
mock::MockChannelFactory::createMessage<messages::ILaserScan>()
{
    return std::make_unique<messages::mock::MockLaserScan>();
}

template<>
inline std::unique_ptr<IPublisher<messages::ILaserScan>>
mock::MockChannelFactory::createPublisher<messages::ILaserScan>(
    const std::string& topic)
{
    auto pub = std::make_unique<mock::MockPublisher<messages::ILaserScan>>(topic);
    auto pubPtr = std::shared_ptr<mock::MockPublisher<messages::ILaserScan>>(pub.get());
    this->registerPublisher(topic, pubPtr);
    return pub;
}

template<>
inline std::unique_ptr<ISubscriber<messages::ILaserScan>>
mock::MockChannelFactory::createSubscriber<messages::ILaserScan>(
    const std::string& topic,
    typename ISubscriber<messages::ILaserScan>::Callback callback)
{
    auto sub = std::make_unique<mock::MockSubscriber<messages::ILaserScan>>(
        topic, std::move(callback));
    auto subPtr = std::shared_ptr<mock::MockSubscriber<messages::ILaserScan>>(sub.get());
    this->registerSubscriber(topic, subPtr);
    return sub;
}

} // namespace veranda::channels
```

---

## Component Usage

### Updated Component Implementation

```cpp
// lidar_sensor.h
#pragma once
#include "veranda/messages/laser_scan.h"
#include "veranda/channels/channel_factory.h"
#include "veranda/channels/publisher.h"
#include <veranda_core/api/world_object_component.h>

class Lidar_Sensor : public WorldObjectComponent {
    Q_OBJECT

    // Channel factory (injected, polymorphic)
    std::shared_ptr<veranda::channels::IChannelFactory> _factory;

    // Publisher (type-safe - ILaserScan, not generic IMessage!)
    std::unique_ptr<veranda::channels::IPublisher<veranda::messages::ILaserScan>> _publisher;

    // Scan configuration
    double _minAngle = -M_PI;
    double _maxAngle = M_PI;
    int _numRays = 360;
    double _maxRange = 10.0;
    std::vector<float> _ranges;

public:
    Lidar_Sensor(const QString& pluginIID, QObject* parent = nullptr);

    // Dependency injection
    void setChannelFactory(std::shared_ptr<veranda::channels::IChannelFactory> factory) {
        _factory = std::move(factory);
    }

    void _connectChannels() override {
        disconnectChannels();

        if (_factory) {
            QString topic = output_channel.get().toString();
            if (!topic.isEmpty()) {
                // Type-safe creation - compiler knows this is ILaserScan
                _publisher = _factory->createPublisher<veranda::messages::ILaserScan>(
                    topic.toStdString()
                );
            }
        }
    }

    void _disconnectChannels() override {
        _publisher.reset();
    }

    void _worldTicked(const double dt) override {
        // Perform raycasting
        performScan(_ranges);

        if (_publisher && _publisher->isValid()) {
            // Create message via factory
            auto msg = _factory->createMessage<veranda::messages::ILaserScan>();

            // Configure using typed interface methods
            msg->setAngleMin(_minAngle);
            msg->setAngleMax(_maxAngle);
            msg->setAngleIncrement((_maxAngle - _minAngle) / _numRays);
            msg->setRangeMin(0.0);
            msg->setRangeMax(_maxRange);
            msg->setRanges(_ranges);

            // Publish
            _publisher->publish(*msg);
        }
    }

private:
    void performScan(std::vector<float>& ranges) {
        // Box2D raycasting logic...
    }
};
```

**Key improvements**:
- No ROS2 dependencies in header
- Type-safe: `IPublisher<ILaserScan>` not generic
- Testable: Can inject mock factory
- Clean: Uses specific message interface methods

---

### Production Usage (ROS2)

```cpp
// main.cpp
#include "lidar_sensor.h"
#include "veranda/channels/ros2/factory.h"
#include "veranda/messages/ros2/laser_scan.h"  // Brings in ROS2 specializations
#include "veranda/messages/ros2/pose_2d.h"
#include "veranda/messages/ros2/twist_2d.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("veranda");

    // Create ROS2 factory
    auto factory = std::make_shared<veranda::channels::ros2::Ros2ChannelFactory>(node);

    // Create sensor and inject factory
    Lidar_Sensor sensor;
    sensor.setChannelFactory(factory);
    sensor.connectChannels();

    // Simulation loop
    while (rclcpp::ok()) {
        sensor.worldTicked(0.016);
        rclcpp::spin_some(node);
    }

    return 0;
}
```

---

### Test Usage (Mock)

```cpp
// test_lidar.cpp
#include "lidar_sensor.h"
#include "veranda/channels/mock/factory.h"
#include "veranda/messages/mock/laser_scan.h"  // Brings in Mock specializations
#include <catch2/catch.hpp>

TEST_CASE("Lidar publishes correct scan data") {
    // Create mock factory - no ROS2 runtime needed!
    auto factory = std::make_shared<veranda::channels::mock::MockChannelFactory>();

    Lidar_Sensor sensor;
    sensor.setChannelFactory(factory);
    sensor.connectChannels();

    // Tick sensor
    sensor.worldTicked(0.1);

    // Verify via mock backend
    auto mockPub = factory->getMockPublisher<veranda::messages::ILaserScan>("/scan");
    REQUIRE(mockPub != nullptr);
    REQUIRE(mockPub->messageCount() == 1);

    // Get the message - returns ILaserScan interface
    auto msg = mockPub->lastMessage();
    REQUIRE(msg->ranges().size() == 360);
    REQUIRE(msg->angleMin() == Approx(-M_PI));
    REQUIRE(msg->angleMax() == Approx(M_PI));
}

TEST_CASE("Omni drive responds to velocity commands") {
    auto factory = std::make_shared<veranda::channels::mock::MockChannelFactory>();

    Omni_Drive drive;
    drive.setChannelFactory(factory);
    drive.connectChannels();

    // Get mock subscriber to inject commands
    auto mockSub = factory->getMockSubscriber<veranda::messages::ITwist2D>("/cmd_vel");
    REQUIRE(mockSub != nullptr);

    // Create and inject a velocity command
    auto cmd = factory->createMessage<veranda::messages::ITwist2D>();
    cmd->setLinearX(1.0);
    cmd->setLinearY(0.0);
    cmd->setAngularZ(0.0);
    mockSub->injectMessage(cmd);

    // Tick physics and verify motion
    for (int i = 0; i < 10; ++i) {
        drive.worldTicked(1.0/30.0);
    }

    // Verify body moved in positive x direction
    b2Vec2 pos = drive.getBody()->GetPosition();
    REQUIRE(pos.x > 0);
}
```

---

## Adding a New Message Type

The key benefit: **Add new message without modifying factory or existing code!**

### Step 1: Define Interface

```cpp
// include/veranda/messages/imu.h
#pragma once

namespace veranda::messages {

class IIMU {
public:
    virtual ~IIMU() = default;

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

### Step 2: Implement ROS2 Backend (New File - No Modifications to Existing Code!)

```cpp
// include/veranda/messages/ros2/imu.h
#pragma once
#include "veranda/messages/imu.h"
#include "veranda/channels/ros2/factory.h"
#include "veranda/channels/ros2/publisher.h"
#include <sensor_msgs/msg/imu.hpp>

namespace veranda::messages::ros2 {

// 1. Implementation
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

// 2. Add trait specialization
namespace veranda::channels::ros2 {
    template<>
    struct Ros2MessageTraits<messages::IIMU> {
        using ros2_msg_type = sensor_msgs::msg::Imu;
        using impl_type = messages::ros2::Ros2IMU;
    };
}

// 3. Factory specializations (self-contained!)
namespace veranda::channels {

template<>
inline std::unique_ptr<messages::IIMU>
ros2::Ros2ChannelFactory::createMessage<messages::IIMU>()
{
    return std::make_unique<messages::ros2::Ros2IMU>();
}

template<>
inline std::unique_ptr<IPublisher<messages::IIMU>>
ros2::Ros2ChannelFactory::createPublisher<messages::IIMU>(const std::string& topic)
{
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

} // namespace veranda::channels
```

**Done!** No modifications to `ros2/factory.h` or any other existing file.

### Step 3: Use in Component

```cpp
// imu_sensor.cpp
#include "veranda/messages/ros2/imu.h"  // Include to enable ROS2 backend for IMU

void IMU_Sensor::_worldTicked(double dt) {
    auto msg = _factory->createMessage<veranda::messages::IIMU>();
    msg->setAccelX(computedAccelX);
    msg->setGyroZ(computedGyroZ);
    _publisher->publish(*msg);
}
```

---

## Implementation Phases

### Phase 1: Core Infrastructure (Week 1-2)

1. **Define message interfaces** (no backend dependencies)
   - `messages/laser_scan.h` - ILaserScan
   - `messages/pose_2d.h` - IPose2D
   - `messages/twist_2d.h` - ITwist2D
   - `messages/joy.h` - IJoy

2. **Define channel interfaces**
   - `channels/publisher.h` - IPublisher<T>
   - `channels/subscriber.h` - ISubscriber<T>
   - `channels/channel_factory.h` - IChannelFactory

3. **Create mock backend** (simplest, for validation)
   - `channels/mock/factory.h` - MockChannelFactory
   - `channels/mock/publisher.h` - MockPublisher<T>
   - `channels/mock/subscriber.h` - MockSubscriber<T>
   - `messages/mock/laser_scan.h` - MockLaserScan + specializations
   - `messages/mock/pose_2d.h` - MockPose2D + specializations

4. **Write tests using mock backend**
   - Validates interface design
   - Ensures message interfaces are sufficient
   - No ROS2 dependency yet

### Phase 2: ROS2 Backend (Week 3-4)

1. **Implement ROS2 factory**
   - `channels/ros2/factory.h` - Ros2ChannelFactory (minimal!)
   - `channels/ros2/publisher.h` - Ros2Publisher<T> + traits
   - `channels/ros2/subscriber.h` - Ros2Subscriber<T>

2. **Implement ROS2 message wrappers**
   - `messages/ros2/laser_scan.h` - Ros2LaserScan + specializations
   - `messages/ros2/pose_2d.h` - Ros2Pose2D + specializations
   - `messages/ros2/twist_2d.h` - Ros2Twist2D + specializations
   - `messages/ros2/joy.h` - Ros2Joy + specializations

3. **Integration testing**
   - Verify ROS2 messages publish correctly
   - Test with actual ROS2 tools (ros2 topic echo, etc.)

### Phase 3: Component Migration (Week 5-8)

Migrate components one at a time to minimize risk:

1. **Lidar_Sensor** (most complex - validates design)
   - Add `setChannelFactory()` method
   - Replace `_rosNode` with `_factory`
   - Replace direct ROS2 calls with interface calls
   - Write unit tests with mock factory
   - Verify integration with ROS2 factory

2. **GPS_Sensor** (simpler sensor)
3. **Touch_Sensor** (simplest sensor)
4. **Omni_Drive** (subscriber example)
5. **Fixed_Wheel** (another wheel type)
6. **Ackermann_Steer** (complex wheel)

For each component:
- Keep old implementation temporarily (`_rosNode` as fallback)
- Add new factory-based implementation
- Test both paths
- Remove old implementation once validated

### Phase 4: Integration and Cleanup (Week 9-10)

1. **Update SimulatorCore**
   - Create and inject channel factory
   - Remove `setROSNode()` pattern

2. **Update main.cpp**
   - Create appropriate factory based on runtime mode
   - Pass to SimulatorCore

3. **Remove deprecated code**
   - Remove `setROSNode()` methods
   - Remove direct ROS2 dependencies from component headers

4. **Documentation**
   - Update component development guide
   - Add examples for adding new message types
   - Document testing patterns

---

## Project Structure

```
include/veranda/
  messages/
    laser_scan.h              # ILaserScan interface only
    pose_2d.h                 # IPose2D interface only
    twist_2d.h                # ITwist2D interface only
    joy.h                     # IJoy interface only
    imu.h                     # IIMU interface only

  messages/ros2/
    laser_scan.h              # Ros2LaserScan + 3 specializations
    pose_2d.h                 # Ros2Pose2D + 3 specializations
    twist_2d.h                # Ros2Twist2D + 3 specializations
    joy.h                     # Ros2Joy + 3 specializations
    imu.h                     # Ros2IMU + 3 specializations

  messages/mock/
    laser_scan.h              # MockLaserScan + 3 specializations
    pose_2d.h                 # MockPose2D + 3 specializations
    twist_2d.h                # MockTwist2D + 3 specializations

  channels/
    publisher.h               # IPublisher<T> template interface
    subscriber.h              # ISubscriber<T> template interface
    channel_factory.h         # IChannelFactory base (template declarations)

  channels/ros2/
    factory.h                 # Ros2ChannelFactory (minimal - just holds node)
    publisher.h               # Ros2Publisher<T> template + traits
    subscriber.h              # Ros2Subscriber<T> template

  channels/mock/
    factory.h                 # MockChannelFactory + test helpers
    publisher.h               # MockPublisher<T> template
    subscriber.h              # MockSubscriber<T> template
```

---

## Key Design Decisions

### Why No Type Erasure?

**Problem with type erasure**:
```cpp
class IMessage { virtual const std::type_info& type() const = 0; };
auto msg = factory->createMessage(...);
auto& scan = msg->as<LaserScanData>();  // Runtime cast, error-prone
```

**Our solution**:
```cpp
auto msg = factory->createMessage<ILaserScan>();  // Compile-time type
msg->setAngleMin(-M_PI);  // Type-safe, no casting
```

### Why Decentralized Specializations?

**Centralized (bad)**:
```cpp
// channels/ros2/factory.h needs specialization for EVERY message
template<> ... createMessage<ILaserScan>() { ... }
template<> ... createMessage<IPose2D>() { ... }
template<> ... createMessage<IIMU>() { ... }  // Must modify factory for new messages!
```

**Decentralized (good)**:
```cpp
// messages/ros2/imu.h is self-contained
class Ros2IMU : public IIMU { ... }
template<> ... createMessage<IIMU>() { return make_unique<Ros2IMU>(); }
// Factory doesn't need modification!
```

### Why Template Specialization Instead of Virtual Methods?

**Virtual methods would require**:
```cpp
class IChannelFactory {
    virtual unique_ptr<ILaserScan> createLaserScanMessage() = 0;  // One per message!
    virtual unique_ptr<IPose2D> createPose2DMessage() = 0;
    virtual unique_ptr<IIMU> createIMUMessage() = 0;
    // Must add method for every new message type
};
```

**Template specialization**:
```cpp
class IChannelFactory {
    template<typename T> unique_ptr<T> createMessage();  // Generic!
    // Specializations come from message headers
};
```

### Why Headers-Only for Specializations?

**Compile-time benefits**:
- Only include messages you actually use
- Smaller binaries (unused messages not compiled in)
- Faster compilation (don't parse unused message headers)
- Conditional compilation possible (`#ifdef VERANDA_WITH_IMU`)

**Extensibility benefits**:
- Add message = add one new header file
- No modifications to existing code
- No central registry to maintain

---

## Benefits Summary

### ✅ Testability
- Components can be unit tested without ROS2 runtime
- Mock backend provides full control for testing
- Can inject specific message values
- Can verify exact messages published

### ✅ No Type Erasure
- Each message has specific typed interface
- Compile-time type safety
- IDE autocomplete works
- No runtime casting errors

### ✅ Extensibility
- Add new message: create ONE new file
- No modifications to factory code
- No modifications to existing message code
- Self-contained message implementations

### ✅ Flexibility
- Can swap communication backends at runtime
- Can record/replay messages
- Can test without network
- Can profile without ROS2 overhead

### ✅ Performance
- Zero runtime overhead from abstraction
- All dispatch happens at compile time
- No virtual calls for message creation
- Same performance as direct ROS2 usage

### ✅ Isolation
- Components don't depend on ROS2 headers
- Clean separation of concerns
- Easier to port to non-ROS platforms
- Reduced compile times for components

### ✅ Maintainability
- Clear architecture
- Self-documenting code (typed interfaces)
- Easy to understand message flow
- Plugin-like extensibility

---

## API Changes Summary

### Deprecated APIs

| Old API | Status |
|---------|--------|
| `WorldObjectComponent::setROSNode()` | Deprecated, use `setChannelFactory()` |
| `WorldObjectComponent::_rosNode` | Deprecated, use `_factory` |
| Direct `rclcpp::Publisher` usage | Use `IPublisher<T>` |
| Direct `rclcpp::Subscription` usage | Use `ISubscriber<T>` |
| ROS2 message types in component headers | Use message interfaces |

### New APIs

| API | Purpose |
|-----|---------|
| `IChannelFactory` | Abstract factory for creating channels |
| `IPublisher<T>` | Abstract publisher for specific message type |
| `ISubscriber<T>` | Abstract subscriber for specific message type |
| `ILaserScan`, `IPose2D`, etc. | Specific message interfaces |
| `Ros2ChannelFactory` | ROS2 implementation of factory |
| `MockChannelFactory` | Mock implementation for testing |

---

## Success Metrics

- [x] Design complete and documented
- [ ] All component unit tests pass without ROS2 runtime
- [ ] No direct ROS2 dependencies in component headers
- [ ] Integration tests verify ROS2 communication works
- [ ] Test coverage increased by 50%+
- [ ] No performance regression in message throughput
- [ ] New message type can be added with single file
- [ ] Documentation complete with examples

---

## Future Enhancements

### Recording Backend
```cpp
class RecordingChannelFactory : public IChannelFactory {
    // Records all messages to file for later replay
};
```

### Replay Backend
```cpp
class ReplayChannelFactory : public IChannelFactory {
    // Plays back previously recorded messages
};
```

### Multi-Backend Support
```cpp
class MultiChannelFactory : public IChannelFactory {
    // Forwards to multiple backends (e.g., ROS2 + Recording)
};
```

### Performance Monitoring
```cpp
class InstrumentedChannelFactory : public IChannelFactory {
    // Wraps another factory, adds timing/profiling
};
```
