# Design Proposal: Component-ROS Infrastructure Separation

## Overview

This document proposes decoupling Veranda's component implementations from direct ROS2 dependencies, enabling unit testing without ROS2 infrastructure and supporting alternative communication backends.

> **Related documents**:
> - [TODO.md](TODO.md) - Phase 2.3 (Improve component testability)
> - [TODO_update_ros.md](TODO_update_ros.md) - ROS2 Jazzy upgrade (complete this first)
> - [TODO_modern_ros.md](TODO_modern_ros.md) - Modern ROS2 threading patterns
>
> **Note**: This refactor should be done **after** completing the ROS2 Jazzy upgrade to avoid doing work twice.

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

## Proposed Solution

### Architecture Overview

Introduce a **Channel Abstraction Layer** between components and ROS2:

```
┌─────────────────────────────────────────────────────────────────┐
│                        Components                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ Lidar       │  │ Omni_Drive  │  │ GPS_Sensor              │  │
│  └──────┬──────┘  └──────┬──────┘  └────────────┬────────────┘  │
│         │                │                      │                │
│         ▼                ▼                      ▼                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              Channel Interface Layer                      │   │
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

### Interface Definitions

#### Core Channel Interfaces

```cpp
// include/veranda_core/api/channels/channel_interfaces.h

#pragma once
#include <functional>
#include <memory>
#include <string>
#include <any>

namespace veranda::channels {

/**
 * @brief Type-erased message interface
 *
 * Allows components to work with messages without knowing ROS2 types
 */
class IMessage {
public:
    virtual ~IMessage() = default;
    virtual const std::type_info& type() const = 0;

    template<typename T>
    const T& as() const {
        return *static_cast<const T*>(data());
    }

    template<typename T>
    T& as() {
        return *static_cast<T*>(data());
    }

protected:
    virtual const void* data() const = 0;
    virtual void* data() = 0;
};

/**
 * @brief Typed message wrapper
 */
template<typename T>
class Message : public IMessage {
    T _data;

public:
    Message() = default;
    explicit Message(T data) : _data(std::move(data)) {}

    const std::type_info& type() const override { return typeid(T); }
    T& get() { return _data; }
    const T& get() const { return _data; }

protected:
    const void* data() const override { return &_data; }
    void* data() override { return &_data; }
};

/**
 * @brief Abstract publisher interface
 */
template<typename MsgType>
class IPublisher {
public:
    virtual ~IPublisher() = default;
    virtual void publish(const MsgType& msg) = 0;
    virtual void publish(std::shared_ptr<MsgType> msg) = 0;
    virtual std::string topic() const = 0;
    virtual bool isValid() const = 0;
};

/**
 * @brief Abstract subscriber interface
 */
template<typename MsgType>
class ISubscriber {
public:
    using Callback = std::function<void(std::shared_ptr<const MsgType>)>;

    virtual ~ISubscriber() = default;
    virtual std::string topic() const = 0;
    virtual bool isValid() const = 0;
};

/**
 * @brief Quality of Service settings (backend-agnostic)
 */
struct QoSProfile {
    enum class Reliability { BestEffort, Reliable };
    enum class Durability { Volatile, TransientLocal };

    size_t depth = 10;
    Reliability reliability = Reliability::Reliable;
    Durability durability = Durability::Volatile;

    static QoSProfile sensorData() {
        return {.depth = 5, .reliability = Reliability::BestEffort};
    }

    static QoSProfile commands() {
        return {.depth = 10, .reliability = Reliability::Reliable};
    }
};

/**
 * @brief Factory for creating publishers and subscribers
 */
class IChannelFactory {
public:
    virtual ~IChannelFactory() = default;

    template<typename MsgType>
    std::shared_ptr<IPublisher<MsgType>> createPublisher(
        const std::string& topic,
        const QoSProfile& qos = {})
    {
        return std::static_pointer_cast<IPublisher<MsgType>>(
            createPublisherImpl(topic, typeid(MsgType), qos));
    }

    template<typename MsgType>
    std::shared_ptr<ISubscriber<MsgType>> createSubscriber(
        const std::string& topic,
        typename ISubscriber<MsgType>::Callback callback,
        const QoSProfile& qos = {})
    {
        auto wrappedCallback = [cb = std::move(callback)](std::shared_ptr<const IMessage> msg) {
            cb(std::make_shared<const MsgType>(msg->as<MsgType>()));
        };
        return std::static_pointer_cast<ISubscriber<MsgType>>(
            createSubscriberImpl(topic, typeid(MsgType), std::move(wrappedCallback), qos));
    }

protected:
    using TypeErasedCallback = std::function<void(std::shared_ptr<const IMessage>)>;

    virtual std::shared_ptr<void> createPublisherImpl(
        const std::string& topic,
        const std::type_info& msgType,
        const QoSProfile& qos) = 0;

    virtual std::shared_ptr<void> createSubscriberImpl(
        const std::string& topic,
        const std::type_info& msgType,
        TypeErasedCallback callback,
        const QoSProfile& qos) = 0;
};

} // namespace veranda::channels
```

#### Native Message Types

Define Veranda-native message types that can be converted to/from ROS2:

```cpp
// include/veranda_core/api/channels/message_types.h

#pragma once
#include <vector>
#include <cstdint>

namespace veranda::messages {

/**
 * @brief 2D pose (position + orientation)
 */
struct Pose2D {
    double x = 0;
    double y = 0;
    double theta = 0;  // radians
};

/**
 * @brief 2D velocity command
 */
struct Twist2D {
    double linear_x = 0;
    double linear_y = 0;
    double angular_z = 0;
};

/**
 * @brief Laser scan data
 */
struct LaserScan {
    double angle_min = 0;
    double angle_max = 0;
    double angle_increment = 0;
    double time_increment = 0;
    double scan_time = 0;
    double range_min = 0;
    double range_max = 0;
    std::vector<float> ranges;
    std::vector<float> intensities;
};

/**
 * @brief Joystick state
 */
struct Joy {
    std::vector<float> axes;
    std::vector<int32_t> buttons;
};

} // namespace veranda::messages
```

### ROS2 Backend Implementation

```cpp
// src/channels/ros2_channel_factory.h

#pragma once
#include "veranda_core/api/channels/channel_interfaces.h"
#include <rclcpp/rclcpp.hpp>

namespace veranda::channels::ros2 {

/**
 * @brief Convert Veranda QoS to ROS2 QoS
 */
inline rclcpp::QoS toRos2Qos(const QoSProfile& profile) {
    rclcpp::QoS qos(profile.depth);

    if (profile.reliability == QoSProfile::Reliability::BestEffort) {
        qos.best_effort();
    } else {
        qos.reliable();
    }

    if (profile.durability == QoSProfile::Durability::TransientLocal) {
        qos.transient_local();
    }

    return qos;
}

/**
 * @brief ROS2 publisher wrapper
 */
template<typename VerandaMsg, typename Ros2Msg>
class Ros2Publisher : public IPublisher<VerandaMsg> {
    typename rclcpp::Publisher<Ros2Msg>::SharedPtr _pub;
    std::string _topic;

public:
    Ros2Publisher(rclcpp::Node::SharedPtr node,
                  const std::string& topic,
                  const QoSProfile& qos)
        : _topic(topic)
    {
        _pub = node->create_publisher<Ros2Msg>(topic, toRos2Qos(qos));
    }

    void publish(const VerandaMsg& msg) override {
        auto rosMsg = toRos2(msg);
        _pub->publish(rosMsg);
    }

    void publish(std::shared_ptr<VerandaMsg> msg) override {
        publish(*msg);
    }

    std::string topic() const override { return _topic; }
    bool isValid() const override { return _pub != nullptr; }

private:
    // Conversion function (specialize for each message type)
    static Ros2Msg toRos2(const VerandaMsg& msg);
};

/**
 * @brief ROS2 subscriber wrapper
 */
template<typename VerandaMsg, typename Ros2Msg>
class Ros2Subscriber : public ISubscriber<VerandaMsg> {
    typename rclcpp::Subscription<Ros2Msg>::SharedPtr _sub;
    std::string _topic;

public:
    Ros2Subscriber(rclcpp::Node::SharedPtr node,
                   const std::string& topic,
                   typename ISubscriber<VerandaMsg>::Callback callback,
                   const QoSProfile& qos)
        : _topic(topic)
    {
        auto rosCallback = [cb = std::move(callback)](typename Ros2Msg::SharedPtr msg) {
            auto verandaMsg = std::make_shared<const VerandaMsg>(fromRos2(*msg));
            cb(verandaMsg);
        };

        _sub = node->create_subscription<Ros2Msg>(topic, toRos2Qos(qos), rosCallback);
    }

    std::string topic() const override { return _topic; }
    bool isValid() const override { return _sub != nullptr; }

private:
    // Conversion function (specialize for each message type)
    static VerandaMsg fromRos2(const Ros2Msg& msg);
};

/**
 * @brief ROS2 implementation of channel factory
 */
class Ros2ChannelFactory : public IChannelFactory {
    rclcpp::Node::SharedPtr _node;

public:
    explicit Ros2ChannelFactory(rclcpp::Node::SharedPtr node)
        : _node(std::move(node)) {}

protected:
    std::shared_ptr<void> createPublisherImpl(
        const std::string& topic,
        const std::type_info& msgType,
        const QoSProfile& qos) override;

    std::shared_ptr<void> createSubscriberImpl(
        const std::string& topic,
        const std::type_info& msgType,
        TypeErasedCallback callback,
        const QoSProfile& qos) override;
};

// Message conversions
template<>
inline geometry_msgs::msg::Pose2D
Ros2Publisher<messages::Pose2D, geometry_msgs::msg::Pose2D>::toRos2(
    const messages::Pose2D& msg)
{
    geometry_msgs::msg::Pose2D ros;
    ros.x = msg.x;
    ros.y = msg.y;
    ros.theta = msg.theta;
    return ros;
}

template<>
inline messages::Pose2D
Ros2Subscriber<messages::Pose2D, geometry_msgs::msg::Pose2D>::fromRos2(
    const geometry_msgs::msg::Pose2D& ros)
{
    return {ros.x, ros.y, ros.theta};
}

// Similar for LaserScan, Joy, etc.

} // namespace veranda::channels::ros2
```

### Mock Backend for Testing

```cpp
// include/veranda_core/api/channels/mock_channel_factory.h

#pragma once
#include "veranda_core/api/channels/channel_interfaces.h"
#include <deque>
#include <mutex>

namespace veranda::channels::mock {

/**
 * @brief Mock publisher that records all published messages
 */
template<typename MsgType>
class MockPublisher : public IPublisher<MsgType> {
    std::string _topic;
    mutable std::mutex _mutex;
    std::deque<MsgType> _publishedMessages;
    size_t _maxHistory = 100;

public:
    explicit MockPublisher(const std::string& topic) : _topic(topic) {}

    void publish(const MsgType& msg) override {
        std::lock_guard lock(_mutex);
        _publishedMessages.push_back(msg);
        while (_publishedMessages.size() > _maxHistory) {
            _publishedMessages.pop_front();
        }
    }

    void publish(std::shared_ptr<MsgType> msg) override {
        publish(*msg);
    }

    std::string topic() const override { return _topic; }
    bool isValid() const override { return true; }

    // Test helpers
    size_t messageCount() const {
        std::lock_guard lock(_mutex);
        return _publishedMessages.size();
    }

    MsgType lastMessage() const {
        std::lock_guard lock(_mutex);
        if (_publishedMessages.empty()) {
            throw std::runtime_error("No messages published");
        }
        return _publishedMessages.back();
    }

    std::vector<MsgType> allMessages() const {
        std::lock_guard lock(_mutex);
        return {_publishedMessages.begin(), _publishedMessages.end()};
    }

    void clear() {
        std::lock_guard lock(_mutex);
        _publishedMessages.clear();
    }
};

/**
 * @brief Mock subscriber that allows injecting messages
 */
template<typename MsgType>
class MockSubscriber : public ISubscriber<MsgType> {
    std::string _topic;
    typename ISubscriber<MsgType>::Callback _callback;

public:
    MockSubscriber(const std::string& topic,
                   typename ISubscriber<MsgType>::Callback callback)
        : _topic(topic), _callback(std::move(callback)) {}

    std::string topic() const override { return _topic; }
    bool isValid() const override { return true; }

    // Test helper: inject a message as if received
    void injectMessage(const MsgType& msg) {
        if (_callback) {
            _callback(std::make_shared<const MsgType>(msg));
        }
    }

    void injectMessage(std::shared_ptr<const MsgType> msg) {
        if (_callback) {
            _callback(msg);
        }
    }
};

/**
 * @brief Mock channel factory for testing
 */
class MockChannelFactory : public IChannelFactory {
    std::map<std::string, std::shared_ptr<void>> _publishers;
    std::map<std::string, std::shared_ptr<void>> _subscribers;

public:
    template<typename MsgType>
    std::shared_ptr<MockPublisher<MsgType>> getMockPublisher(const std::string& topic) {
        auto it = _publishers.find(topic);
        if (it != _publishers.end()) {
            return std::static_pointer_cast<MockPublisher<MsgType>>(it->second);
        }
        return nullptr;
    }

    template<typename MsgType>
    std::shared_ptr<MockSubscriber<MsgType>> getMockSubscriber(const std::string& topic) {
        auto it = _subscribers.find(topic);
        if (it != _subscribers.end()) {
            return std::static_pointer_cast<MockSubscriber<MsgType>>(it->second);
        }
        return nullptr;
    }

protected:
    std::shared_ptr<void> createPublisherImpl(
        const std::string& topic,
        const std::type_info& msgType,
        const QoSProfile& qos) override
    {
        // Create mock publisher based on type
        // Implementation depends on type registration
        auto pub = std::make_shared<MockPublisher<messages::Pose2D>>(topic);
        _publishers[topic] = pub;
        return pub;
    }

    std::shared_ptr<void> createSubscriberImpl(
        const std::string& topic,
        const std::type_info& msgType,
        TypeErasedCallback callback,
        const QoSProfile& qos) override
    {
        // Create mock subscriber
        auto wrappedCallback = [callback](std::shared_ptr<const messages::Pose2D> msg) {
            callback(std::make_shared<Message<messages::Pose2D>>(*msg));
        };
        auto sub = std::make_shared<MockSubscriber<messages::Pose2D>>(topic, wrappedCallback);
        _subscribers[topic] = sub;
        return sub;
    }
};

} // namespace veranda::channels::mock
```

### Updated Component Implementation

```cpp
// Updated lidar_sensor.h

#pragma once
#include <veranda_core/api/world_object_component.h>
#include <veranda_core/api/channels/channel_interfaces.h>
#include <veranda_core/api/channels/message_types.h>

class Lidar_Sensor : public WorldObjectComponent {
    Q_OBJECT

    // Channel factory (injected)
    std::shared_ptr<veranda::channels::IChannelFactory> _channelFactory;

    // Publisher (created via factory)
    std::shared_ptr<veranda::channels::IPublisher<veranda::messages::LaserScan>> _publisher;

    // Scan data
    veranda::messages::LaserScan _scanData;

public:
    Lidar_Sensor(const QString& pluginIID, QObject* parent = nullptr);

    // Dependency injection for channel factory
    void setChannelFactory(std::shared_ptr<veranda::channels::IChannelFactory> factory) {
        _channelFactory = std::move(factory);
    }

    void _connectChannels() override {
        disconnectChannels();

        if (_channelFactory) {
            QString topic = output_channel.get().toString();
            if (!topic.isEmpty()) {
                _publisher = _channelFactory->createPublisher<veranda::messages::LaserScan>(
                    topic.toStdString(),
                    veranda::channels::QoSProfile::sensorData()
                );
            }
        }
    }

    void _disconnectChannels() override {
        _publisher.reset();
    }

    void _worldTicked(const double dt) override {
        // ... scan logic unchanged ...

        if (_publisher && _publisher->isValid()) {
            _publisher->publish(_scanData);
        }
    }

    // For testing: get the publisher to verify behavior
    auto getPublisher() const { return _publisher; }
};
```

### Updated SimulatorCore

```cpp
// simulator_core.cpp

SimulatorCore::SimulatorCore(
    Simulator_Physics_If* physics,
    Simulator_Ui_If* ui,
    std::shared_ptr<veranda::channels::IChannelFactory> channelFactory,
    QObject* parent)
    : QObject(parent)
    , _physicsEngine(physics)
    , _userInterface(ui)
    , _channelFactory(std::move(channelFactory))
{
    // ... existing setup ...
}

void SimulatorCore::addSimObjects(QVector<WorldObject*> objs, bool makeCopies) {
    for (WorldObject* obj : objs) {
        // Clone and setup
        WorldObject* newObj = qobject_cast<WorldObject*>(obj->clone());

        // Inject channel factory instead of ROS node
        newObj->setChannelFactory(_channelFactory);

        // ... rest of setup ...
    }
}
```

### Example Unit Tests

```cpp
// tests/test_lidar_sensor.cpp

#include <catch2/catch.hpp>
#include "veranda_sensors/lidar_sensor.h"
#include "veranda_core/api/channels/mock_channel_factory.h"

using namespace veranda::channels;
using namespace veranda::messages;

TEST_CASE("Lidar sensor publishes scan data") {
    // Setup with mock factory
    auto mockFactory = std::make_shared<mock::MockChannelFactory>();

    Lidar_Sensor sensor("test.lidar", nullptr);
    sensor.setChannelFactory(mockFactory);

    // Configure sensor
    sensor.getProperties()["channels/output_ranges"]->set("/test/scan");
    sensor.getProperties()["scan_points"]->set(10);
    sensor.getProperties()["scan_radius"]->set(5.0);
    sensor.getProperties()["scan_rate"]->set(10.0);  // 10 Hz

    // Setup physics
    b2World world(b2Vec2(0, 0));
    b2BodyDef bodyDef;
    b2Body* anchor = world.CreateBody(&bodyDef);
    sensor.generateBodies(&world, 1, anchor);

    // Connect channels
    sensor.connectChannels();

    // Get mock publisher
    auto mockPub = mockFactory->getMockPublisher<LaserScan>("/test/scan");
    REQUIRE(mockPub != nullptr);

    // Tick enough for one scan (100ms at 10Hz)
    sensor.worldTicked(0.1);

    // Verify message published
    REQUIRE(mockPub->messageCount() == 1);

    LaserScan scan = mockPub->lastMessage();
    REQUIRE(scan.ranges.size() == 10);
    REQUIRE(scan.range_max <= 5.0);
}

TEST_CASE("Omni drive responds to velocity commands") {
    auto mockFactory = std::make_shared<mock::MockChannelFactory>();

    Omni_Drive drive("test.omni", nullptr);
    drive.setChannelFactory(mockFactory);

    // Configure
    drive.getProperties()["channels/input_velocity"]->set("/test/cmd_vel");
    drive.getProperties()["driven"]->set(true);

    // Setup physics
    b2World world(b2Vec2(0, 0));
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    b2Body* anchor = world.CreateBody(&bodyDef);
    drive.generateBodies(&world, 1, anchor);
    drive.connectChannels();

    // Get mock subscriber
    auto mockSub = mockFactory->getMockSubscriber<Pose2D>("/test/cmd_vel");
    REQUIRE(mockSub != nullptr);

    // Inject a velocity command
    Pose2D cmd{.x = 1.0, .y = 0.0, .theta = 0.0};
    mockSub->injectMessage(cmd);

    // Tick physics
    for (int i = 0; i < 10; ++i) {
        world.Step(1.0/30.0, 8, 3);
        drive.worldTicked(1.0/30.0);
    }

    // Verify body moved
    b2Vec2 pos = drive.getBody()->GetPosition();
    REQUIRE(pos.x > 0);  // Moved in positive x direction
}

TEST_CASE("Sensor does not crash without channel factory") {
    Lidar_Sensor sensor("test.lidar", nullptr);
    // No channel factory set

    sensor.connectChannels();  // Should not crash
    sensor.worldTicked(0.1);   // Should not crash
    sensor.disconnectChannels();  // Should not crash
}
```

## Implementation Phases

### Phase 1: Interface Definition

1. Create `channel_interfaces.h` with abstract interfaces
2. Define Veranda-native message types
3. Create mock implementation for testing

### Phase 2: ROS2 Backend

1. Implement `Ros2ChannelFactory`
2. Add message conversion functions
3. Ensure API compatibility with existing behavior

### Phase 3: Component Migration

Migrate components one at a time:

1. `Lidar_Sensor` - most complex, validates design
2. `GPS_Sensor` - simpler sensor
3. `Touch_Sensor` - simplest sensor
4. `Omni_Drive` - subscriber example
5. `Fixed_Wheel` - another wheel type
6. `Ackermann_Steer` - complex wheel

For each component:
1. Add `setChannelFactory()` method
2. Replace direct ROS2 calls with interface calls
3. Write unit tests with mock factory
4. Verify integration still works

### Phase 4: Integration

1. Update `SimulatorCore` to create and inject factory
2. Update `main.cpp` to create appropriate factory
3. Remove deprecated `setROSNode()` pattern
4. Integration testing

### Phase 5: Additional Backends (Optional)

1. Recording backend (save messages to file)
2. Replay backend (playback recorded messages)
3. Direct memory backend (for in-process testing)

## API Changes Summary

### Deprecated APIs

| Old API | Status |
|---------|--------|
| `WorldObjectComponent::setROSNode()` | Deprecated, use `setChannelFactory()` |
| `WorldObjectComponent::_rosNode` | Deprecated, use factory |
| Direct `rclcpp::Publisher` creation | Use `IChannelFactory::createPublisher()` |
| Direct `rclcpp::Subscription` creation | Use `IChannelFactory::createSubscriber()` |

### New APIs

| API | Purpose |
|-----|---------|
| `IChannelFactory` | Abstract factory for creating channels |
| `IPublisher<T>` | Abstract publisher interface |
| `ISubscriber<T>` | Abstract subscriber interface |
| `veranda::messages::*` | Native message types |
| `MockChannelFactory` | Testing helper |
| `Ros2ChannelFactory` | ROS2 implementation |

## Benefits

1. **Testability**: Components can be unit tested without ROS2
2. **Flexibility**: Can swap communication backends
3. **Isolation**: Components don't depend on ROS2 headers
4. **Maintainability**: Clear separation of concerns
5. **Performance Testing**: Can measure component performance without network
6. **Recording/Replay**: Easy to add message recording

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Performance overhead from abstraction | Use templates, inline where possible |
| Message conversion cost | Zero-copy where possible, batch conversions |
| API complexity increase | Good documentation, migration guide |
| Breaking existing code | Maintain compatibility shim during transition |

## Success Metrics

- [ ] All component unit tests pass without ROS2 runtime
- [ ] No direct ROS2 dependencies in component headers
- [ ] Integration tests verify ROS2 communication still works
- [ ] Test coverage increased by 50%+
- [ ] No performance regression in message throughput
