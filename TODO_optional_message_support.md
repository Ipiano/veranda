# Optional Message Support Per Backend

## The Problem

With pure template specialization, if a backend doesn't provide a specialization for a message type, you get a **linker error**:

```cpp
// Only include LaserScan, NOT IMU
#include "veranda/messages/ros2/laser_scan.h"

auto factory = std::make_shared<Ros2ChannelFactory>(node);

// Works - specialization included
auto scan = factory->createMessage<ILaserScan>();

// LINKER ERROR - no specialization for IIMU!
auto imu = factory->createMessage<IIMU>();
```

**Error**: `undefined reference to Ros2ChannelFactory::createMessage<IIMU>()`

### Questions This Raises

1. **Must all messages implement all backends?**
   - Example: Mock backend might not need complex IMU simulation
   - Example: ROS2 backend might not support custom proprietary messages

2. **How to disable a message for a backend?**
   - Don't create the implementation file?
   - Conditional compilation?

3. **How to detect support at runtime?**
   - Check before calling `createMessage`?
   - Get a list of supported messages?

---

## Solution Strategies

### Strategy 1: Feature Traits (Compile-Time Detection)

Define traits that declare which messages each backend supports.

#### Implementation

```cpp
// channels/backend_traits.h
#pragma once
#include <type_traits>

namespace veranda::channels {

/**
 * @brief Trait to check if a backend supports a message type
 *
 * Specialize this for each backend+message combination.
 * Default is false (not supported).
 */
template<typename Backend, typename MessageInterface>
struct SupportsMessage : std::false_type {};

/**
 * @brief Helper to check message support at compile time
 */
template<typename Backend, typename MessageInterface>
inline constexpr bool SupportsMessage_v =
    SupportsMessage<Backend, MessageInterface>::value;

} // namespace veranda::channels
```

#### Backend Declares Support

```cpp
// messages/ros2/laser_scan.h
#include "veranda/channels/backend_traits.h"

// Implementation
class Ros2LaserScan : public ILaserScan { ... };

// Declare support
namespace veranda::channels {
    template<>
    struct SupportsMessage<ros2::Ros2ChannelFactory, messages::ILaserScan>
        : std::true_type {};
}

// Factory specialization (as before)
template<>
inline std::unique_ptr<messages::ILaserScan>
ros2::Ros2ChannelFactory::createMessage<messages::ILaserScan>() {
    return std::make_unique<messages::ros2::Ros2LaserScan>();
}
```

#### Factory Checks Support with Static Assert

```cpp
// channels/channel_factory.h

template<typename MessageInterface>
std::unique_ptr<MessageInterface> IChannelFactory::createMessage() {
    // Compile-time check - fails with clear error message
    static_assert(
        SupportsMessage_v<std::remove_pointer_t<decltype(this)>, MessageInterface>,
        "This backend does not support the requested message type. "
        "Include the appropriate message implementation header or use a different backend."
    );

    return createMessageImpl<MessageInterface>();
}

protected:
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessageImpl();
    // Specializations as before
```

#### Usage

```cpp
// Only include LaserScan support
#include "veranda/messages/ros2/laser_scan.h"

auto factory = std::make_shared<Ros2ChannelFactory>(node);

// Works
auto scan = factory->createMessage<ILaserScan>();

// COMPILE ERROR with clear message:
// "This backend does not support the requested message type..."
auto imu = factory->createMessage<IIMU>();
```

**Pros**:
- Clear compile-time error with helpful message
- Can check support with `if constexpr` before calling
- Zero runtime overhead

**Cons**:
- Still fails to compile if you try to use unsupported message
- Can't discover supported messages at runtime

---

### Strategy 2: Optional Return (Runtime Detection)

Factory methods return `std::optional` or `nullptr` when message not supported.

#### Implementation

```cpp
// channels/channel_factory.h

namespace veranda::channels {

/**
 * @brief Factory with optional message creation
 */
class IChannelFactory {
public:
    /**
     * @brief Create message, returns nullptr if not supported
     */
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage() {
        if constexpr (SupportsMessage_v<std::remove_pointer_t<decltype(this)>,
                                        MessageInterface>) {
            return createMessageImpl<MessageInterface>();
        } else {
            return nullptr;  // Not supported
        }
    }

    /**
     * @brief Check if message type is supported
     */
    template<typename MessageInterface>
    bool supportsMessage() const {
        return SupportsMessage_v<std::remove_pointer_t<decltype(this)>,
                                 MessageInterface>;
    }

protected:
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessageImpl();
    // Specializations only for supported messages
};

} // namespace veranda::channels
```

#### Usage

```cpp
#include "veranda/messages/ros2/laser_scan.h"
// Don't include IMU

auto factory = std::make_shared<Ros2ChannelFactory>(node);

// Check support before creating
if (factory->supportsMessage<IIMU>()) {
    auto imu = factory->createMessage<IIMU>();
    // Won't execute - returns false
} else {
    // Handle unsupported message
    std::cerr << "IMU not supported by this backend\n";
}

// Or just check return value
auto imu = factory->createMessage<IIMU>();
if (!imu) {
    // Handle unsupported message
}

// LaserScan works
auto scan = factory->createMessage<ILaserScan>();
assert(scan != nullptr);
```

**Pros**:
- Graceful runtime handling
- Can check support before creating
- Code compiles even if message not supported

**Cons**:
- Need to check return value
- Slight runtime overhead (if constexpr is compile-time, but still need nullptr check)
- Doesn't prevent accidentally trying to use unsupported message

---

### Strategy 3: Fallback Implementation (Throws Exception)

Provide a default implementation that throws when message not supported.

#### Implementation

```cpp
// channels/channel_factory.h

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
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage() {
        return createMessageImpl<MessageInterface>();
    }

protected:
    // Default implementation - throws
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessageImpl() {
        throw UnsupportedMessageException(
            typeid(*this).name(),
            typeid(MessageInterface).name()
        );
    }

    // Specializations override this default
};

} // namespace veranda::channels
```

#### Backend Specializations Override Default

```cpp
// messages/ros2/laser_scan.h

// Specialization for supported message
template<>
inline std::unique_ptr<messages::ILaserScan>
ros2::Ros2ChannelFactory::createMessageImpl<messages::ILaserScan>() {
    return std::make_unique<messages::ros2::Ros2LaserScan>();
}

// If IMU not supported, just don't provide specialization
// The default implementation will throw
```

#### Usage

```cpp
#include "veranda/messages/ros2/laser_scan.h"

auto factory = std::make_shared<Ros2ChannelFactory>(node);

// Works
auto scan = factory->createMessage<ILaserScan>();

// Throws UnsupportedMessageException at runtime
try {
    auto imu = factory->createMessage<IIMU>();
} catch (const UnsupportedMessageException& e) {
    std::cerr << e.what() << "\n";
    // "Backend 'Ros2ChannelFactory' does not support message type 'IIMU'"
}
```

**Pros**:
- Code compiles even without specialization
- Clear runtime error message
- Don't need to check return value

**Cons**:
- Runtime error instead of compile-time
- Need exception handling
- Might not discover unsupported message until runtime

---

### Strategy 4: Simplified Approach (Recommended)

Combine traits for compile-time checking with a single create method that throws if unsupported.

#### Implementation

```cpp
// channels/channel_factory.h

namespace veranda::channels {

/**
 * @brief Factory with trait-based support detection
 */
class IChannelFactory {
public:
    /**
     * @brief Create message (throws if not supported)
     *
     * If the backend doesn't support this message type, throws
     * UnsupportedMessageException. This indicates a programming error
     * (forgot to include implementation header) or misconfiguration.
     */
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage() {
        return createMessageImpl<MessageInterface>();
    }

    /**
     * @brief Check if message type is supported
     *
     * Use this for optional features or when you need to check support
     * before attempting to create. For required messages, just call
     * createMessage() and let it throw if there's a problem.
     */
    template<typename MessageInterface>
    static constexpr bool supportsMessage() noexcept {
        return SupportsMessage_v<std::remove_pointer_t<decltype(this)>,
                                 MessageInterface>;
    }

protected:
    // Default implementation throws
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessageImpl() {
        throw UnsupportedMessageException(
            typeid(*this).name(),
            typeid(MessageInterface).name()
        );
    }
};

} // namespace veranda::channels
```

#### Usage Patterns

```cpp
#include "veranda/messages/ros2/laser_scan.h"
// IMU not included - not supported

auto factory = std::make_shared<Ros2ChannelFactory>(node);

// Pattern 1: Required message - just create (throws if missing)
auto scan = factory->createMessage<ILaserScan>();  // Must exist
_publisher->publish(*scan);

// Pattern 2: Optional feature - check support first
if (factory->supportsMessage<IIMU>()) {
    auto imu = factory->createMessage<IIMU>();
    setupIMU(imu);
} else {
    // IMU not supported, use alternative navigation
    useAlternativeNavigation();
}

// Pattern 3: Test code - skip if not supported
if (!factory->supportsMessage<IIMU>()) {
    GTEST_SKIP() << "IMU not supported by this backend";
}
auto imu = factory->createMessage<IIMU>();
testIMU(imu);

// Pattern 4: Compile-time check (zero runtime overhead)
if constexpr (Ros2ChannelFactory::supportsMessage<ILaserScan>()) {
    auto scan = factory->createMessage<ILaserScan>();
    // Compiler knows this branch will execute
}
```

**Pros**:
- ✅ Simple API: just `createMessage()` and `supportsMessage()`
- ✅ Clear semantics: unsupported message = error
- ✅ No nullable returns to check everywhere
- ✅ Use `supportsMessage()` only when you need optional behavior
- ✅ Compile-time checks available with `if constexpr`

**Cons**:
- Need exception handling for unsupported messages (but that's a bug anyway)

**Design philosophy**: If you're calling `createMessage<IIMU>()` but didn't include the IMU implementation header, that's a **programming error**, not a condition to gracefully handle. Use `supportsMessage()` only for genuinely optional features.

---

## Conditional Compilation

You can conditionally compile message support using preprocessor:

### In Message Header

```cpp
// messages/ros2/imu.h
#pragma once

#ifdef VERANDA_ENABLE_IMU_SUPPORT

#include "veranda/messages/imu.h"
#include "veranda/channels/ros2/factory.h"

namespace veranda::messages::ros2 {
    class Ros2IMU : public IIMU { ... };
}

namespace veranda::channels {
    template<>
    struct SupportsMessage<ros2::Ros2ChannelFactory, messages::IIMU>
        : std::true_type {};

    template<>
    inline std::unique_ptr<messages::IIMU>
    ros2::Ros2ChannelFactory::createMessageImpl<messages::IIMU>() {
        return std::make_unique<messages::ros2::Ros2IMU>();
    }
}

#endif // VERANDA_ENABLE_IMU_SUPPORT
```

### In CMake

```cmake
option(VERANDA_ENABLE_IMU_SUPPORT "Enable IMU sensor support" ON)

if(VERANDA_ENABLE_IMU_SUPPORT)
    target_compile_definitions(veranda PUBLIC VERANDA_ENABLE_IMU_SUPPORT)
    target_sources(veranda PRIVATE
        src/sensors/imu_sensor.cpp
    )
endif()
```

### Usage

```cpp
#include "veranda/messages/ros2/imu.h"  // Only defines support if enabled

auto factory = std::make_shared<Ros2ChannelFactory>(node);

#ifdef VERANDA_ENABLE_IMU_SUPPORT
    auto imu = factory->createMessage<IIMU>();
#else
    // IMU support not compiled in
#endif

// Or runtime check (works regardless of compilation)
if (factory->supportsMessage<IIMU>()) {
    auto imu = factory->createMessage<IIMU>();
}
```

---

## Partial Backend Support Example

### Scenario: Mock Backend Doesn't Need All Messages

```cpp
// messages/mock/laser_scan.h - Implemented
class MockLaserScan : public ILaserScan { ... };

template<>
struct SupportsMessage<mock::MockChannelFactory, messages::ILaserScan>
    : std::true_type {};

template<>
inline std::unique_ptr<messages::ILaserScan>
mock::MockChannelFactory::createMessageImpl<messages::ILaserScan>() {
    return std::make_unique<messages::mock::MockLaserScan>();
}
```

```cpp
// messages/mock/imu.h - NOT implemented
// File doesn't exist, or exists but doesn't provide specialization
```

### Usage

```cpp
#include "veranda/channels/mock/factory.h"
#include "veranda/messages/mock/laser_scan.h"
// Don't include messages/mock/imu.h - doesn't exist

auto mockFactory = std::make_shared<MockChannelFactory>();

// Works - LaserScan is supported
auto scan = mockFactory->createMessage<ILaserScan>();

// Runtime check prevents error
if (mockFactory->supportsMessage<IIMU>()) {
    auto imu = mockFactory->createMessage<IIMU>();
} else {
    // Mock backend doesn't need realistic IMU - skip test
    SKIP_TEST("IMU not supported by mock backend");
}
```

---

## Discovering Supported Messages at Runtime

You can't iterate over all possible message types, but you can query specific ones:

```cpp
// component.cpp

void Component::configure(std::shared_ptr<IChannelFactory> factory) {
    // Check what this factory supports
    struct {
        const char* name;
        bool supported;
    } messageSupport[] = {
        {"LaserScan", factory->supportsMessage<ILaserScan>()},
        {"Pose2D", factory->supportsMessage<IPose2D>()},
        {"IMU", factory->supportsMessage<IIMU>()},
        {"GPS", factory->supportsMessage<IGPS>()},
    };

    std::cout << "Factory supports:\n";
    for (auto [name, supported] : messageSupport) {
        std::cout << "  " << name << ": "
                  << (supported ? "YES" : "NO") << "\n";
    }

    // Configure based on available messages
    if (factory->supportsMessage<ILaserScan>()) {
        setupLidar(factory);
    }

    if (factory->supportsMessage<IIMU>()) {
        setupIMU(factory);
    } else {
        std::cout << "Skipping IMU - not supported\n";
    }
}
```

---

## Recommended Approach

**Use the Simplified Strategy (Strategy 4)** with these guidelines:

### For Required Messages (Most Common)
```cpp
void Lidar::connectChannels() {
    // LaserScan is required - just create it
    // Will throw UnsupportedMessageException if not supported (programming error)
    _publisher = _factory->createPublisher<ILaserScan>("/scan");
}

void Lidar::worldTicked(double dt) {
    auto msg = _factory->createMessage<ILaserScan>();
    msg->setAngleMin(_minAngle);
    _publisher->publish(*msg);
}
```

### For Optional Features
```cpp
void Robot::configure(std::shared_ptr<IChannelFactory> factory) {
    // Required sensors - will throw if missing
    setupLidar(factory);
    setupOdometry(factory);

    // Optional IMU - check support first
    if (factory->supportsMessage<IIMU>()) {
        setupIMU(factory);
    } else {
        // Use alternative navigation without IMU
        useDeadReckoning();
    }
}
```

### For Test Code
```cpp
TEST_F(IMUTest, ReadsAcceleration) {
    auto factory = std::make_shared<MockChannelFactory>();

    // Skip test if this backend doesn't support IMU
    if (!factory->supportsMessage<IIMU>()) {
        GTEST_SKIP() << "IMU not supported by mock backend";
    }

    auto imu = factory->createMessage<IIMU>();
    testIMU(imu);
}
```

### For Compile-Time Optimization
```cpp
// Only compile GPS code if ROS2 backend supports it
if constexpr (Ros2ChannelFactory::supportsMessage<IGPS>()) {
    #include "veranda/messages/ros2/gps.h"
    // GPS code only compiled if supported
    setupGPS();
}
```

**Design Philosophy**:
- **Required messages**: Just call `createMessage()` - let it throw if there's a problem
- **Optional messages**: Check `supportsMessage()` before creating
- No need for nullable returns or error handling everywhere

---

## Summary

### Question: Must all messages implement all backends?

**No!** You can:
1. Not create implementation file for unsupported messages
2. Use conditional compilation (`#ifdef`)
3. Provide partial backend support

### Question: How to disable a message for a backend?

1. **Don't create the implementation file** (e.g., no `messages/mock/imu.h`)
2. **Use conditional compilation** with `#ifdef`
3. **Don't provide trait specialization** - `SupportsMessage` defaults to `false`

### Question: How to detect support?

1. **Compile-time**: `if constexpr (factory->supportsMessage<IIMU>())`
2. **Runtime**: `if (factory->supportsMessage<IIMU>())`
3. **Try create**: `auto msg = factory->tryCreateMessage<IIMU>(); if (msg) { ... }`

### Recommended Pattern

```cpp
// channels/channel_factory.h
class IChannelFactory {
public:
    // Check if backend supports a message type
    template<typename MessageInterface>
    static constexpr bool supportsMessage() noexcept;

    // Create message (throws if unsupported - indicates programming error)
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage();

    // Create publisher (throws if unsupported)
    template<typename MessageInterface>
    std::unique_ptr<IPublisher<MessageInterface>>
    createPublisher(const std::string& topic);

    // Create subscriber (throws if unsupported)
    template<typename MessageInterface>
    std::unique_ptr<ISubscriber<MessageInterface>>
    createSubscriber(const std::string& topic,
                    typename ISubscriber<MessageInterface>::Callback callback);
};
```

**Simple and clear**:
- For required messages: just call `createMessage()` - throws if there's a problem
- For optional messages: check `supportsMessage()` first
- No nullable returns or error handling everywhere
