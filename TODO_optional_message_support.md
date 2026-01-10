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

### Strategy 4: Hybrid Approach (Recommended)

Combine traits for compile-time checking with runtime support detection.

#### Implementation

```cpp
// channels/channel_factory.h

namespace veranda::channels {

/**
 * @brief Factory with both compile-time and runtime checks
 */
class IChannelFactory {
public:
    /**
     * @brief Create message (throws if not supported)
     */
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> createMessage() {
        // For development: compile-time warning if not supported
        if constexpr (!SupportsMessage_v<std::remove_pointer_t<decltype(this)>,
                                         MessageInterface>) {
            // This generates a warning but still compiles
            [[deprecated("Message type not supported by this backend")]]
            auto warning = 0;
            (void)warning;
        }

        return createMessageImpl<MessageInterface>();
    }

    /**
     * @brief Try to create message, returns nullptr if not supported
     */
    template<typename MessageInterface>
    std::unique_ptr<MessageInterface> tryCreateMessage() noexcept {
        if constexpr (SupportsMessage_v<std::remove_pointer_t<decltype(this)>,
                                        MessageInterface>) {
            return createMessageImpl<MessageInterface>();
        } else {
            return nullptr;
        }
    }

    /**
     * @brief Check if message type is supported
     */
    template<typename MessageInterface>
    constexpr bool supportsMessage() const noexcept {
        return SupportsMessage_v<std::remove_pointer_t<decltype(this)>,
                                 MessageInterface>;
    }

protected:
    // Default throws
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

// Pattern 1: Check support first (runtime)
if (factory->supportsMessage<IIMU>()) {
    auto imu = factory->createMessage<IIMU>();
} else {
    // Fall back to different sensor
}

// Pattern 2: Try create (runtime, no exceptions)
auto imu = factory->tryCreateMessage<IIMU>();
if (imu) {
    // Use it
} else {
    // Not supported
}

// Pattern 3: Compile-time check
if constexpr (factory->supportsMessage<ILaserScan>()) {
    auto scan = factory->createMessage<ILaserScan>();
    // Compiler knows this branch will execute
}

// Pattern 4: Just create (throws if not supported)
try {
    auto scan = factory->createMessage<ILaserScan>();  // OK
    auto imu = factory->createMessage<IIMU>();          // Throws
} catch (const UnsupportedMessageException& e) {
    // Handle error
}
```

**Pros**:
- ✅ Flexibility: choose compile-time or runtime checking
- ✅ `tryCreateMessage()` for graceful handling
- ✅ `supportsMessage()` for querying support
- ✅ `createMessage()` throws with clear error
- ✅ Compile-time warnings in debug builds

**Cons**:
- Slightly more complex API
- Need to choose which method to use

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

**Use the Hybrid Strategy (Strategy 4)** with these guidelines:

### For Production Code
```cpp
// Prefer runtime checks for robustness
if (factory->supportsMessage<IIMU>()) {
    auto imu = factory->createMessage<IIMU>();
    setupIMU(imu);
} else {
    // Gracefully degrade or use alternative
    useAlternativeNavigation();
}
```

### For Test Code
```cpp
// Use tryCreateMessage for optional features
auto imu = factory->tryCreateMessage<IIMU>();
if (imu) {
    // Test IMU functionality
    testIMU(imu);
} else {
    // Skip IMU tests with mock backend
    GTEST_SKIP() << "IMU not supported by this backend";
}
```

### For Components with Required Messages
```cpp
// Use createMessage and let it throw if required message missing
void Lidar::connectChannels() {
    // LaserScan is required - throw if not supported
    _publisher = _factory->createPublisher<ILaserScan>("/scan");
}
```

### For Optional Features
```cpp
// Use compile-time check for optional features
if constexpr (SupportsMessage_v<Ros2ChannelFactory, IGPS>) {
    #include "veranda/messages/ros2/gps.h"
    // GPS code only compiled if supported
}
```

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
    // Check support
    template<typename T> constexpr bool supportsMessage() const noexcept;

    // Try create (returns nullptr if unsupported)
    template<typename T> std::unique_ptr<T> tryCreateMessage() noexcept;

    // Create (throws if unsupported)
    template<typename T> std::unique_ptr<T> createMessage();

    // Try create publisher (returns nullptr if unsupported)
    template<typename T> std::unique_ptr<IPublisher<T>>
        tryCreatePublisher(const std::string& topic) noexcept;

    // Create publisher (throws if unsupported)
    template<typename T> std::unique_ptr<IPublisher<T>>
        createPublisher(const std::string& topic);
};
```

This gives maximum flexibility - choose the right method based on whether the message is required or optional.
