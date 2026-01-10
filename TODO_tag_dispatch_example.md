# Tag Dispatch Mechanism - Detailed Example

This document provides a complete working example of the tag-based dispatch system for eliminating type erasure without runtime type lookups.

## Core Concept

The tag dispatch system uses **empty tag structs** as compile-time identifiers for message types. The compiler uses template specialization to select the correct implementation at compile time - no runtime `type_info` lookups needed!

```
ILaserScan::create()
    ↓ (has type_tag = LaserScanTag)
    ↓
Factory::createMessageForTag<LaserScanTag>()
    ↓ (template specialization)
    ↓
Backend-specific implementation (e.g., Ros2Message<LaserScanTag>)
```

## Complete Example

### Step 1: Define Tags (messages/message_tags.h)

```cpp
#pragma once

namespace veranda::messages {

// Empty structs used only as compile-time type identifiers
struct LaserScanTag {};
struct Pose2DTag {};
struct Twist2DTag {};
struct JoyTag {};
struct IMUTag {};

} // namespace veranda::messages
```

**Key point**: These are just empty types. They carry no data and generate no code - they exist only for the type system.

---

### Step 2: Message Interface Base (messages/message_interface.h)

```cpp
#pragma once
#include "message_tags.h"
#include <type_traits>

namespace veranda::messages {

/**
 * @brief Base class for all message interfaces
 *
 * The Tag template parameter associates each message interface with
 * a unique compile-time identifier used for dispatch.
 */
template<typename Tag>
class MessageInterface {
public:
    using type_tag = Tag;  // Expose tag for template metaprogramming
    virtual ~MessageInterface() = default;
};

} // namespace veranda::messages
```

**Key point**: The `type_tag` type alias allows us to extract the tag from any message interface type at compile time.

---

### Step 3: Specific Message Interfaces (messages/laser_scan.h)

```cpp
#pragma once
#include "message_interface.h"
#include <span>
#include <memory>

namespace veranda::messages {

/**
 * @brief Interface for laser scan messages
 *
 * Tagged with LaserScanTag for compile-time dispatch.
 */
class ILaserScan : public MessageInterface<LaserScanTag> {
public:
    // Specific typed interface - no type erasure!
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

class IPose2D : public MessageInterface<Pose2DTag> {
public:
    virtual double x() const = 0;
    virtual double y() const = 0;
    virtual double theta() const = 0;

    virtual void setX(double val) = 0;
    virtual void setY(double val) = 0;
    virtual void setTheta(double val) = 0;
};

class ITwist2D : public MessageInterface<Twist2DTag> {
public:
    virtual double linearX() const = 0;
    virtual double linearY() const = 0;
    virtual double angularZ() const = 0;

    virtual void setLinearX(double val) = 0;
    virtual void setLinearY(double val) = 0;
    virtual void setAngularZ(double val) = 0;
};

} // namespace veranda::messages
```

**Key point**: Each interface inherits from `MessageInterface<SpecificTag>`, which embeds the tag as a compile-time property.

---

### Step 4: Tag-to-Interface Mapping (messages/message_traits.h)

```cpp
#pragma once
#include "laser_scan.h"
#include "pose_2d.h"
#include "twist_2d.h"

namespace veranda::messages {

/**
 * @brief Trait to map from Tag type to Interface type
 *
 * This allows us to write: TagToInterface<LaserScanTag>::type → ILaserScan
 */
template<typename Tag>
struct TagToInterface;

template<>
struct TagToInterface<LaserScanTag> {
    using type = ILaserScan;
};

template<>
struct TagToInterface<Pose2DTag> {
    using type = IPose2D;
};

template<>
struct TagToInterface<Twist2DTag> {
    using type = ITwist2D;
};

// Helper alias for convenience
template<typename Tag>
using TagToInterface_t = typename TagToInterface<Tag>::type;

/**
 * @brief Trait to map from Interface type to Tag type
 *
 * This allows us to write: InterfaceToTag<ILaserScan>::type → LaserScanTag
 */
template<typename Interface>
struct InterfaceToTag {
    using type = typename Interface::type_tag;
};

template<typename Interface>
using InterfaceToTag_t = typename InterfaceToTag<Interface>::type;

} // namespace veranda::messages
```

**Key point**: These trait structs enable bidirectional compile-time lookup between tags and interfaces.

---

### Step 5: Channel Interfaces (channels/publisher.h)

```cpp
#pragma once
#include <memory>
#include <string>
#include <functional>

namespace veranda::channels {

/**
 * @brief Abstract publisher interface
 *
 * Templated on the message INTERFACE (e.g., ILaserScan), not on the
 * concrete implementation. This allows polymorphism.
 */
template<typename MessageInterface>
class IPublisher {
public:
    virtual ~IPublisher() = default;

    virtual void publish(const MessageInterface& msg) = 0;
    virtual void publish(std::shared_ptr<const MessageInterface> msg) {
        publish(*msg);  // Default implementation
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

### Step 6: Factory Base with Tag Dispatch (channels/channel_factory.h)

```cpp
#pragma once
#include "publisher.h"
#include "veranda/messages/message_traits.h"
#include <memory>
#include <string>

namespace veranda::channels {

/**
 * @brief Abstract factory for creating message channels
 *
 * Uses tag-based dispatch to route creation requests to the correct
 * backend-specific implementation at compile time.
 */
class IChannelFactory {
public:
    virtual ~IChannelFactory() = default;

    /**
     * @brief Create a publisher for the given message interface type
     *
     * This is the public API that components use. It extracts the tag
     * from the MessageInterface and delegates to createPublisherForTag.
     *
     * Example: factory->createPublisher<ILaserScan>("/scan")
     */
    template<typename MessageInterface>
    std::unique_ptr<IPublisher<MessageInterface>>
    createPublisher(const std::string& topic) {
        // Extract the tag from the message interface at compile time
        using Tag = typename MessageInterface::type_tag;

        // Delegate to tag-based method (compile-time dispatch!)
        return createPublisherForTag<Tag>(topic);
    }

    /**
     * @brief Create a subscriber for the given message interface type
     */
    template<typename MessageInterface>
    std::unique_ptr<ISubscriber<MessageInterface>>
    createSubscriber(const std::string& topic,
                    typename ISubscriber<MessageInterface>::Callback callback) {
        using Tag = typename MessageInterface::type_tag;
        return createSubscriberForTag<Tag>(topic, std::move(callback));
    }

    /**
     * @brief Create a message instance of the given interface type
     *
     * Example: auto msg = factory->createMessage<ILaserScan>();
     */
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage() {
        using Tag = typename MessageInterface::type_tag;
        return createMessageForTag<Tag>();
    }

protected:
    /**
     * @brief Backend-specific creation methods
     *
     * These are implemented by concrete factory classes (e.g., Ros2ChannelFactory)
     * via template specialization. Each backend provides specializations for the
     * tags it supports.
     *
     * These methods are protected because they're not called directly by users.
     */
    template<typename Tag>
    std::unique_ptr<IPublisher<messages::TagToInterface_t<Tag>>>
    createPublisherForTag(const std::string& topic);

    template<typename Tag>
    std::unique_ptr<ISubscriber<messages::TagToInterface_t<Tag>>>
    createSubscriberForTag(const std::string& topic,
                          typename ISubscriber<messages::TagToInterface_t<Tag>>::Callback callback);

    template<typename Tag>
    std::unique_ptr<messages::TagToInterface_t<Tag>>
    createMessageForTag();
};

} // namespace veranda::channels
```

**Key point**: The public API (`createPublisher<ILaserScan>`) extracts the tag and delegates to `createPublisherForTag<LaserScanTag>`. This tag-based method gets specialized by backend implementations.

---

### Step 7: ROS2 Message Implementation (messages/ros2/laser_scan.h)

```cpp
#pragma once
#include "veranda/messages/laser_scan.h"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

namespace veranda::messages::ros2 {

/**
 * @brief ROS2 implementation of LaserScan interface
 *
 * Wraps sensor_msgs::msg::LaserScan and implements ILaserScan interface.
 */
class Ros2LaserScan : public ILaserScan {
    sensor_msgs::msg::LaserScan _msg;
    mutable std::vector<float> _rangesView;  // For returning span

public:
    Ros2LaserScan() = default;

    // Allow constructing from native ROS2 message (for subscribers)
    explicit Ros2LaserScan(sensor_msgs::msg::LaserScan msg)
        : _msg(std::move(msg)) {}

    // ILaserScan interface implementation
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

    // Backend-specific access to native message
    sensor_msgs::msg::LaserScan& native() { return _msg; }
    const sensor_msgs::msg::LaserScan& native() const { return _msg; }
};

// Similarly for other messages
class Ros2Pose2D : public IPose2D {
    geometry_msgs::msg::Pose2D _msg;

public:
    Ros2Pose2D() = default;
    explicit Ros2Pose2D(geometry_msgs::msg::Pose2D msg) : _msg(std::move(msg)) {}

    double x() const override { return _msg.x; }
    double y() const override { return _msg.y; }
    double theta() const override { return _msg.theta; }

    void setX(double val) override { _msg.x = val; }
    void setY(double val) override { _msg.y = val; }
    void setTheta(double val) override { _msg.theta = val; }

    geometry_msgs::msg::Pose2D& native() { return _msg; }
    const geometry_msgs::msg::Pose2D& native() const { return _msg; }
};

} // namespace veranda::messages::ros2
```

---

### Step 8: ROS2 Publisher Implementation (channels/ros2/publisher.h)

```cpp
#pragma once
#include "veranda/channels/publisher.h"
#include "veranda/messages/ros2/laser_scan.h"  // Get Ros2LaserScan
#include "veranda/messages/ros2/pose_2d.h"     // Get Ros2Pose2D
#include <rclcpp/rclcpp.hpp>

namespace veranda::channels::ros2 {

/**
 * @brief Trait to map message interface to ROS2 message type
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
 *
 * Templated on the message INTERFACE, uses traits to determine
 * the concrete ROS2 message type and implementation class.
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
        // Safe downcast - we know ImplType implements MessageInterface
        // and we only create ImplType messages from this factory
        const auto& impl = static_cast<const ImplType&>(msg);

        // Publish the native ROS2 message
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
        auto ros2Callback = [cb = std::move(callback)](typename Ros2MsgType::SharedPtr rosMsg) {
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

### Step 9: ROS2 Factory Implementation (channels/ros2/factory.h)

```cpp
#pragma once
#include "veranda/channels/channel_factory.h"
#include "veranda/channels/ros2/publisher.h"
#include <rclcpp/rclcpp.hpp>

namespace veranda::channels::ros2 {

/**
 * @brief ROS2 implementation of channel factory
 */
class Ros2ChannelFactory : public IChannelFactory {
    rclcpp::Node::SharedPtr _node;

public:
    explicit Ros2ChannelFactory(rclcpp::Node::SharedPtr node)
        : _node(std::move(node)) {}

    rclcpp::Node::SharedPtr node() const { return _node; }
};

} // namespace veranda::channels::ros2

// ============================================================================
// TEMPLATE SPECIALIZATIONS - This is where the magic happens!
// ============================================================================

namespace veranda::channels {

// Provide implementation for LaserScan publisher creation
template<>
inline std::unique_ptr<IPublisher<messages::ILaserScan>>
ros2::Ros2ChannelFactory::createPublisherForTag<messages::LaserScanTag>(
    const std::string& topic)
{
    return std::make_unique<ros2::Ros2Publisher<messages::ILaserScan>>(_node, topic);
}

// Provide implementation for LaserScan message creation
template<>
inline std::unique_ptr<messages::ILaserScan>
ros2::Ros2ChannelFactory::createMessageForTag<messages::LaserScanTag>()
{
    return std::make_unique<messages::ros2::Ros2LaserScan>();
}

// Provide implementation for LaserScan subscriber creation
template<>
inline std::unique_ptr<ISubscriber<messages::ILaserScan>>
ros2::Ros2ChannelFactory::createSubscriberForTag<messages::LaserScanTag>(
    const std::string& topic,
    typename ISubscriber<messages::ILaserScan>::Callback callback)
{
    return std::make_unique<ros2::Ros2Subscriber<messages::ILaserScan>>(
        _node, topic, std::move(callback));
}

// ============================================================================
// Repeat for Pose2D
// ============================================================================

template<>
inline std::unique_ptr<IPublisher<messages::IPose2D>>
ros2::Ros2ChannelFactory::createPublisherForTag<messages::Pose2DTag>(
    const std::string& topic)
{
    return std::make_unique<ros2::Ros2Publisher<messages::IPose2D>>(_node, topic);
}

template<>
inline std::unique_ptr<messages::IPose2D>
ros2::Ros2ChannelFactory::createMessageForTag<messages::Pose2DTag>()
{
    return std::make_unique<messages::ros2::Ros2Pose2D>();
}

template<>
inline std::unique_ptr<ISubscriber<messages::IPose2D>>
ros2::Ros2ChannelFactory::createSubscriberForTag<messages::Pose2DTag>(
    const std::string& topic,
    typename ISubscriber<messages::IPose2D>::Callback callback)
{
    return std::make_unique<ros2::Ros2Subscriber<messages::IPose2D>>(
        _node, topic, std::move(callback));
}

// ============================================================================
// Repeat for Twist2D, Joy, etc.
// ============================================================================

} // namespace veranda::channels
```

**Key point**: These template specializations are what make the tag dispatch work! When you include this header, the compiler knows how to create ROS2 publishers for specific tag types.

---

### Step 10: Component Usage

```cpp
// lidar_sensor.h
#pragma once
#include "veranda/messages/laser_scan.h"
#include "veranda/channels/channel_factory.h"
#include "veranda/channels/publisher.h"

class Lidar_Sensor {
    std::shared_ptr<veranda::channels::IChannelFactory> _factory;
    std::unique_ptr<veranda::channels::IPublisher<veranda::messages::ILaserScan>> _publisher;

    double _minAngle = -M_PI;
    double _maxAngle = M_PI;
    int _numRays = 360;
    double _maxRange = 10.0;
    std::vector<float> _ranges;

public:
    void setChannelFactory(std::shared_ptr<veranda::channels::IChannelFactory> factory) {
        _factory = std::move(factory);
    }

    void connectChannels() {
        if (_factory) {
            // Generic API - looks simple!
            _publisher = _factory->createPublisher<veranda::messages::ILaserScan>("/scan");
        }
    }

    void worldTicked(double dt) {
        // Perform raycasting...
        performScan(_ranges);

        if (_publisher && _publisher->isValid()) {
            // Create message via factory
            auto msg = _factory->createMessage<veranda::messages::ILaserScan>();

            // Configure using typed interface
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

---

### Step 11: Putting It All Together

```cpp
// main.cpp - ROS2 build
#include "lidar_sensor.h"
#include "veranda/channels/ros2/factory.h"  // Includes all ROS2 specializations
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("veranda");

    // Create ROS2 factory
    auto factory = std::make_shared<veranda::channels::ros2::Ros2ChannelFactory>(node);

    // Create sensor
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

```cpp
// test_lidar.cpp - Test build
#include "lidar_sensor.h"
#include "veranda/channels/mock/factory.h"  // Includes mock specializations
#include <catch2/catch.hpp>

TEST_CASE("Lidar publishes scan data") {
    // Create mock factory
    auto factory = std::make_shared<veranda::channels::mock::MockChannelFactory>();

    Lidar_Sensor sensor;
    sensor.setChannelFactory(factory);
    sensor.connectChannels();

    // Tick sensor
    sensor.worldTicked(0.1);

    // Verify message was published
    auto mockPub = factory->getMockPublisher<veranda::messages::ILaserScan>("/scan");
    REQUIRE(mockPub != nullptr);
    REQUIRE(mockPub->messageCount() == 1);

    auto msg = mockPub->lastMessage();
    // Verify message contents...
}
```

---

## Compilation Flow Example

Let's trace what happens when you call `factory->createPublisher<ILaserScan>("/scan")`:

### 1. Template Instantiation

```cpp
// User writes:
factory->createPublisher<ILaserScan>("/scan")

// Compiler instantiates IChannelFactory::createPublisher<ILaserScan>:
template<>
std::unique_ptr<IPublisher<ILaserScan>>
IChannelFactory::createPublisher<ILaserScan>(const std::string& topic) {
    using Tag = typename ILaserScan::type_tag;  // → LaserScanTag
    return createPublisherForTag<LaserScanTag>(topic);  // Call tag-based method
}
```

### 2. Tag Extraction

```cpp
// The compiler extracts the tag at compile time:
typename ILaserScan::type_tag
    = typename MessageInterface<LaserScanTag>::type_tag
    = LaserScanTag
```

### 3. Tag-Based Dispatch

```cpp
// The call becomes:
factory->createPublisherForTag<LaserScanTag>("/scan")

// For ROS2 factory, this resolves to the specialization:
template<>
std::unique_ptr<IPublisher<ILaserScan>>
Ros2ChannelFactory::createPublisherForTag<LaserScanTag>(const std::string& topic) {
    return std::make_unique<ros2::Ros2Publisher<ILaserScan>>(_node, topic);
}
```

### 4. Concrete Type Creation

```cpp
// Creates Ros2Publisher<ILaserScan>, which uses:
Ros2MessageTraits<ILaserScan>::impl_type
    = Ros2LaserScan  // The concrete ROS2 implementation
```

### 5. Result

The factory returns a `unique_ptr<IPublisher<ILaserScan>>` that points to a `Ros2Publisher<ILaserScan>` instance. When you call `publish()`, it downcasts to `Ros2LaserScan` and publishes the native `sensor_msgs::msg::LaserScan`.

**All type resolution happens at compile time!** No runtime type checks, no `std::type_info` lookups, no dynamic casting.

---

## Key Advantages

### 1. Type Safety
```cpp
// Won't compile - ILaserScan and IPose2D are different types!
IPublisher<ILaserScan>* pub = factory->createPublisher<IPose2D>("/scan");
```

### 2. No Runtime Overhead
```cpp
// Both resolve to direct function calls at compile time:
factory->createPublisher<ILaserScan>("/scan");   // → Ros2Publisher<ILaserScan>
factory->createMessage<ILaserScan>();            // → Ros2LaserScan
```

### 3. Compile-Time Errors for Missing Implementations
```cpp
// If you forget to provide a specialization for a tag:
factory->createPublisher<INewMessage>("/topic");
// Compiler error: no matching function for
// Ros2ChannelFactory::createPublisherForTag<NewMessageTag>
```

### 4. Easy to Extend
To add a new message type:
1. Define tag: `struct NewMessageTag {};`
2. Define interface: `class INewMessage : public MessageInterface<NewMessageTag> { ... };`
3. Implement backend: `class Ros2NewMessage : public INewMessage { ... };`
4. Add specializations in `ros2/factory.h`

No modifications to base factory or existing code!

---

## Comparison: With vs Without Tags

### Without Tags (Runtime Dispatch)
```cpp
// Factory needs runtime type lookup
std::shared_ptr<void> createPublisherImpl(const std::type_info& msgType, ...) {
    auto it = _publisherFactories.find(std::type_index(msgType));  // Runtime map lookup
    if (it == _publisherFactories.end()) throw ...;
    return it->second(...);  // Function pointer call
}
```

### With Tags (Compile-Time Dispatch)
```cpp
// Factory uses template specialization
template<>
std::unique_ptr<IPublisher<ILaserScan>>
createPublisherForTag<LaserScanTag>(const std::string& topic) {
    return std::make_unique<Ros2Publisher<ILaserScan>>(...);  // Direct call
}
```

The tag version:
- ✅ No runtime map lookup
- ✅ No runtime type checks
- ✅ Compiler can inline everything
- ✅ Compile-time error if implementation missing
- ✅ No runtime overhead vs. calling constructor directly

---

## Summary

The tag dispatch system provides:

1. **Compile-time dispatch**: Tags are used as template parameters, resolved at compile time
2. **No type erasure**: Each message has specific typed interface
3. **No runtime overhead**: All dispatch happens during compilation
4. **Type safety**: Mismatched types caught by compiler
5. **Extensibility**: Add messages without modifying factory base class
6. **Clean API**: Components just call `create<ILaserScan>()`

The "magic" is that template specializations in backend headers teach the factory how to create instances for specific tags, and the tag is embedded in each message interface as a compile-time property.
