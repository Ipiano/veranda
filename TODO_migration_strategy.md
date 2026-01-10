# Migration Strategy: Component-ROS Separation Implementation

## Overview

This document outlines strategies for migrating existing components from direct ROS2 dependencies to the new abstraction layer with minimal risk and without requiring both systems to coexist long-term.

> **Related**: [TODO_component_ros_separation.md](TODO_component_ros_separation.md) - Design details

---

## Goal

**Migrate all components to use the abstraction layer without having both old (direct ROS2) and new (factory-based) systems coexisting indefinitely.**

---

## Key Challenges

1. **All-or-nothing problem**: Components can't easily support both old and new APIs simultaneously
2. **Testing confidence**: Need to validate each component still works correctly
3. **Integration risk**: Changing all components at once is high-risk
4. **Rollback complexity**: If something breaks, hard to revert partial migration
5. **Build system**: Need to manage dependencies and includes cleanly

---

## Recommended Strategy: Staged Bottom-Up Migration

Migrate in stages, completing each layer fully before moving to the next. This avoids long-term coexistence while managing risk.

### Phase 0: Infrastructure (1-2 weeks)

**Goal**: Build the abstraction layer without touching existing components.

#### Tasks

1. **Create message interfaces** (no backend dependencies)
   ```
   include/veranda/messages/
     laser_scan.h       - ILaserScan interface
     pose_2d.h          - IPose2D interface
     twist_2d.h         - ITwist2D interface
     joy.h              - IJoy interface
   ```

2. **Create channel interfaces**
   ```
   include/veranda/channels/
     publisher.h        - IPublisher<T>
     subscriber.h       - ISubscriber<T>
     channel_factory.h  - IChannelFactory
     backend_traits.h   - SupportsMessage trait
   ```

3. **Implement mock backend** (simplest, for testing infrastructure)
   ```
   include/veranda/messages/mock/
     laser_scan.h       - MockLaserScan + specializations
     pose_2d.h          - MockPose2D + specializations

   include/veranda/channels/mock/
     factory.h          - MockChannelFactory
     publisher.h        - MockPublisher<T>
     subscriber.h       - MockSubscriber<T>
   ```

4. **Write infrastructure tests**
   - Test factory creation
   - Test message creation
   - Test publisher/subscriber mock functionality
   - Validate trait system works

**Deliverable**: Fully functional abstraction layer with mock backend, tested independently.

**Risk**: Low - no existing code touched

---

### Phase 1: ROS2 Backend (1 week)

**Goal**: Implement ROS2 backend for all existing message types.

#### Tasks

1. **Create ROS2 factory**
   ```
   include/veranda/channels/ros2/
     factory.h          - Ros2ChannelFactory (minimal)
     publisher.h        - Ros2Publisher<T> + traits
     subscriber.h       - Ros2Subscriber<T>
   ```

2. **Implement ROS2 message wrappers** (one per existing message type)
   ```
   include/veranda/messages/ros2/
     laser_scan.h       - Ros2LaserScan + trait + specializations
     pose_2d.h          - Ros2Pose2D + trait + specializations
     twist_2d.h         - Ros2Twist2D + trait + specializations
     joy.h              - Ros2Joy + trait + specializations
   ```

3. **Integration tests** (no component changes yet!)
   - Create standalone test program that uses ROS2 factory
   - Publish messages, verify with `ros2 topic echo`
   - Subscribe to messages from `ros2 topic pub`
   - Validate message conversion is correct

**Deliverable**: Fully functional ROS2 backend, tested independently against real ROS2.

**Risk**: Low - still no existing components touched

---

### Phase 2: Migrate SimulatorCore (1 week)

**Goal**: Change how components receive their messaging infrastructure.

#### Current State

```cpp
// simulator_core.cpp
void SimulatorCore::addSimObjects(...) {
    WorldObject* newObj = ...;
    newObj->setROSNode(_rosNode);  // Old way
}
```

#### New State

```cpp
// simulator_core.cpp
void SimulatorCore::addSimObjects(...) {
    WorldObject* newObj = ...;
    newObj->setChannelFactory(_channelFactory);  // New way
}
```

#### Migration Steps

1. **Add factory to SimulatorCore**
   ```cpp
   class SimulatorCore {
       std::shared_ptr<veranda::channels::IChannelFactory> _channelFactory;
       rclcpp::Node::SharedPtr _rosNode;  // Keep temporarily
   public:
       void setChannelFactory(std::shared_ptr<veranda::channels::IChannelFactory> factory) {
           _channelFactory = factory;
       }
       // Keep setROSNode temporarily
   };
   ```

2. **Update WorldObjectComponent base class**
   ```cpp
   class WorldObjectComponent : public QObject {
       std::shared_ptr<veranda::channels::IChannelFactory> _channelFactory;
       rclcpp::Node::SharedPtr _rosNode;  // Keep temporarily

   public:
       void setChannelFactory(std::shared_ptr<veranda::channels::IChannelFactory> factory) {
           _channelFactory = factory;
       }
       // Keep setROSNode temporarily

   protected:
       std::shared_ptr<veranda::channels::IChannelFactory> channelFactory() const {
           return _channelFactory;
       }
       // Keep rosNode() temporarily
   };
   ```

3. **Update main.cpp**
   ```cpp
   int main() {
       auto node = std::make_shared<rclcpp::Node>("veranda");

       // Create factory
       auto factory = std::make_shared<veranda::channels::ros2::Ros2ChannelFactory>(node);

       SimulatorCore core(...);
       core.setChannelFactory(factory);  // New
       core.setROSNode(node);            // Keep temporarily
   }
   ```

**Deliverable**: Infrastructure in place to provide factory to components, old system still works.

**Risk**: Low - dual support, no behavior changes yet

---

### Phase 3: Migrate Components One-by-One (3-4 weeks)

**Goal**: Migrate each component individually, test thoroughly, commit.

#### Migration Order (Risk-Based)

1. **Lidar_Sensor** (complex, validates design) - 3-4 days
2. **Touch_Sensor** (simplest, quick win) - 1-2 days
3. **GPS_Sensor** (simple publisher) - 1-2 days
4. **Omni_Drive** (subscriber example) - 2-3 days
5. **Fixed_Wheel** (another wheel type) - 2-3 days
6. **Ackermann_Steer** (complex wheel) - 2-3 days

#### Per-Component Migration Process

For each component (e.g., Lidar_Sensor):

**Step 1: Create new implementation in same file**

```cpp
// lidar_sensor.h
class Lidar_Sensor : public WorldObjectComponent {
    // OLD (keep temporarily)
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _sendChannel;

    // NEW
    std::unique_ptr<veranda::channels::IPublisher<veranda::messages::ILaserScan>> _publisher;

public:
    void _connectChannels() override {
        disconnectChannels();

        // Try new way first
        if (channelFactory()) {
            _publisher = channelFactory()->createPublisher<veranda::messages::ILaserScan>(
                output_channel.get().toString().toStdString()
            );
        }
        // Fallback to old way
        else if (rosNode()) {
            _sendChannel = rosNode()->create_publisher<sensor_msgs::msg::LaserScan>(...);
        }
    }

    void _worldTicked(double dt) override {
        performScan();

        // Try new way first
        if (_publisher && _publisher->isValid()) {
            auto msg = channelFactory()->createMessage<veranda::messages::ILaserScan>();
            msg->setAngleMin(_minAngle);
            msg->setRanges(_ranges);
            _publisher->publish(*msg);
        }
        // Fallback to old way
        else if (_sendChannel) {
            sensor_msgs::msg::LaserScan rosMsg;
            rosMsg.angle_min = _minAngle;
            rosMsg.ranges = _ranges;
            _sendChannel->publish(rosMsg);
        }
    }
};
```

**Step 2: Write unit tests using mock backend**

```cpp
// test/test_lidar_sensor.cpp
TEST_F(LidarSensorTest, PublishesCorrectScanData) {
    auto mockFactory = std::make_shared<veranda::channels::mock::MockChannelFactory>();

    Lidar_Sensor sensor(...);
    sensor.setChannelFactory(mockFactory);
    sensor.connectChannels();

    sensor.worldTicked(0.1);

    auto mockPub = mockFactory->getMockPublisher<veranda::messages::ILaserScan>("/scan");
    ASSERT_NE(mockPub, nullptr);
    ASSERT_EQ(mockPub->messageCount(), 1);

    auto msg = mockPub->lastMessage();
    EXPECT_NEAR(msg->angleMin(), -M_PI, 0.01);
    EXPECT_EQ(msg->ranges().size(), 360);
}
```

**Step 3: Integration test with ROS2**

Create temporary standalone test:

```cpp
// test/integration/test_lidar_ros2.cpp
int main() {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_lidar");

    auto factory = std::make_shared<veranda::channels::ros2::Ros2ChannelFactory>(node);

    Lidar_Sensor sensor(...);
    sensor.setChannelFactory(factory);
    sensor.connectChannels();

    // Verify publishes correctly
    auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [](auto msg) { /* verify */ });

    sensor.worldTicked(0.1);
    rclcpp::spin_some(node);

    return 0;
}
```

Run with:
```bash
./test_lidar_ros2 &
ros2 topic echo /scan  # Verify output
```

**Step 4: Remove old implementation once validated**

After tests pass:

```cpp
// lidar_sensor.h
class Lidar_Sensor : public WorldObjectComponent {
    // OLD - REMOVED
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _sendChannel;

    // NEW - only way
    std::unique_ptr<veranda::channels::IPublisher<veranda::messages::ILaserScan>> _publisher;

public:
    void _connectChannels() override {
        disconnectChannels();

        if (channelFactory()) {
            _publisher = channelFactory()->createPublisher<veranda::messages::ILaserScan>(
                output_channel.get().toString().toStdString()
            );
        }
    }

    void _worldTicked(double dt) override {
        if (_publisher && _publisher->isValid()) {
            auto msg = channelFactory()->createMessage<veranda::messages::ILaserScan>();
            msg->setAngleMin(_minAngle);
            msg->setRanges(_ranges);
            _publisher->publish(*msg);
        }
    }
};
```

**Step 5: Commit**

```bash
git add lidar_sensor.h lidar_sensor.cpp test/test_lidar_sensor.cpp
git commit -m "Migrate Lidar_Sensor to abstraction layer

- Remove direct ROS2 dependencies
- Use IChannelFactory and ILaserScan interface
- Add unit tests with mock backend
- Verified with ROS2 integration test"
```

**Repeat for each component.**

**Deliverable**: One fully migrated component per iteration, tested and committed.

**Risk**: Medium - changes one component at a time, easy to rollback individual commits

---

### Phase 4: Remove Old System (1 week)

**Goal**: Clean up dual support, remove deprecated APIs.

#### Tasks

1. **Verify all components migrated**
   ```bash
   # Should find no matches
   git grep "rosNode()" -- '*.cpp' '*.h' | grep -v WorldObjectComponent
   git grep "_rosNode" -- '*.cpp' '*.h'
   git grep "rclcpp::Publisher" -- 'src/components/**'
   ```

2. **Remove old APIs from base class**
   ```cpp
   class WorldObjectComponent : public QObject {
       std::shared_ptr<veranda::channels::IChannelFactory> _channelFactory;
       // REMOVED: rclcpp::Node::SharedPtr _rosNode;

   public:
       void setChannelFactory(std::shared_ptr<veranda::channels::IChannelFactory> factory) {
           _channelFactory = factory;
       }
       // REMOVED: setROSNode()

   protected:
       std::shared_ptr<veranda::channels::IChannelFactory> channelFactory() const {
           return _channelFactory;
       }
       // REMOVED: rosNode()
   };
   ```

3. **Remove old APIs from SimulatorCore**

4. **Update main.cpp**
   ```cpp
   int main() {
       auto node = std::make_shared<rclcpp::Node>("veranda");
       auto factory = std::make_shared<veranda::channels::ros2::Ros2ChannelFactory>(node);

       SimulatorCore core(...);
       core.setChannelFactory(factory);
       // REMOVED: core.setROSNode(node);
   }
   ```

5. **Update documentation**

6. **Final integration test** - run full simulator, verify all components work

**Deliverable**: Clean codebase with only new abstraction layer.

**Risk**: Low - all components already migrated

---

## Alternative Strategy: Big Bang Migration (Higher Risk)

If you want to minimize coexistence time at the cost of higher risk:

### Approach

1. **Phases 0-1**: Same as above (build infrastructure)

2. **Phase 2: Migrate everything at once** (2-3 weeks)
   - Create feature branch
   - Migrate all components simultaneously
   - Update SimulatorCore and main.cpp
   - Remove old APIs immediately
   - Extensive testing before merge

3. **Phase 3: Merge** (1 week)
   - Final validation
   - Merge to main

### Pros
- Shorter coexistence period
- Cleaner - no dual support code
- Forces complete migration

### Cons
- ❌ Higher risk - big changes all at once
- ❌ Harder to test incrementally
- ❌ Difficult to isolate component-specific issues
- ❌ Large code review
- ❌ If rollback needed, lose all work

**Recommendation**: Only use if you have very high confidence in the design and comprehensive automated tests.

---

## Hybrid Strategy: Feature Branch with Staged Commits (Recommended)

Combines benefits of both approaches:

### Approach

1. **Create feature branch** `feature/component-ros-separation`

2. **Phases 0-2**: Build infrastructure (as above), commit to feature branch

3. **Phase 3**: Migrate components one-by-one
   - Migrate one component
   - Test thoroughly
   - Commit to feature branch
   - Repeat for next component

4. **Phase 4**: Remove old system
   - Clean up dual support
   - Commit to feature branch

5. **Merge to main** when all components done
   - Entire migration appears atomic in main branch
   - Feature branch has granular history for debugging

### Pros
- ✅ Staged development with safety
- ✅ Main branch stays clean (no dual support visible)
- ✅ Can validate entire migration before merging
- ✅ Granular history for debugging on feature branch
- ✅ Easy rollback - just don't merge

### Cons
- Feature branch diverges from main over time
- Need to keep rebasing if main is active

**This is the recommended approach** - combines incremental safety with clean main branch.

---

## Risk Mitigation

### Automated Testing

Create comprehensive test suite before starting:

```cpp
// test/component_interface_test.cpp
// Generic test that runs against all components

template<typename ComponentType>
class ComponentInterfaceTest : public ::testing::Test {
protected:
    std::shared_ptr<MockChannelFactory> factory;
    std::unique_ptr<ComponentType> component;

    void SetUp() override {
        factory = std::make_shared<MockChannelFactory>();
        component = std::make_unique<ComponentType>(...);
        component->setChannelFactory(factory);
    }
};

using ComponentTypes = ::testing::Types<
    Lidar_Sensor,
    GPS_Sensor,
    Touch_Sensor,
    Omni_Drive,
    Fixed_Wheel,
    Ackermann_Steer
>;

TYPED_TEST_SUITE(ComponentInterfaceTest, ComponentTypes);

TYPED_TEST(ComponentInterfaceTest, ConnectsChannels) {
    this->component->connectChannels();
    // Verify channels created
}

TYPED_TEST(ComponentInterfaceTest, PublishesOnTick) {
    this->component->connectChannels();
    this->component->worldTicked(0.1);
    // Verify messages published
}
```

### Integration Test Script

```bash
#!/bin/bash
# test/integration/run_all.sh

set -e

echo "Building with new abstraction layer..."
colcon build

echo "Running unit tests..."
colcon test

echo "Starting ROS2 integration test..."
./build/veranda/test_integration &
VERANDA_PID=$!

sleep 2

echo "Verifying topics..."
ros2 topic list | grep /scan || exit 1
ros2 topic echo /scan --once || exit 1

kill $VERANDA_PID

echo "All tests passed!"
```

### Checklist per Component

- [ ] Unit tests with mock backend pass
- [ ] Integration test with ROS2 publishes correct messages
- [ ] Can subscribe to commands (if applicable)
- [ ] No compiler warnings
- [ ] No direct ROS2 includes in component header
- [ ] Old implementation removed
- [ ] Committed to version control

---

## Timeline Summary

### Staged Migration (Recommended - 6-8 weeks total)
- Week 1-2: Phase 0 (Infrastructure)
- Week 3: Phase 1 (ROS2 backend)
- Week 4: Phase 2 (SimulatorCore)
- Week 5-7: Phase 3 (Components, ~1 per week)
- Week 8: Phase 4 (Cleanup)

### Big Bang (Higher Risk - 4-5 weeks total)
- Week 1-2: Infrastructure
- Week 3-4: Migrate everything
- Week 5: Testing and fixes

### Feature Branch Hybrid (Recommended - 6-8 weeks total)
- Same timeline as staged, but on feature branch
- Single merge to main when complete

---

## Rollback Strategy

### If Migration Fails Mid-Stream

**Staged approach**:
```bash
# Revert just the failing component commit
git revert <commit-hash>
```

**Feature branch approach**:
```bash
# Don't merge feature branch, continue on main
git checkout main
```

**Big bang approach**:
```bash
# Revert entire merge
git revert -m 1 <merge-commit>
# Back to square one, all work lost
```

---

## Recommendation

**Use the Feature Branch Hybrid Strategy**:

1. Create `feature/component-ros-separation` branch
2. Build infrastructure (Phases 0-1)
3. Update SimulatorCore (Phase 2)
4. Migrate components one-by-one (Phase 3), committing each
5. Remove old system (Phase 4)
6. Merge to main when done

**Benefits**:
- Incremental, low-risk development
- Thorough testing at each step
- Clean history on feature branch
- Main branch unaffected until ready
- Easy to abandon if design issues found
- Can cherry-pick individual component migrations if needed

**Timeline**: 6-8 weeks

**Risk**: Low - can validate thoroughly before merging to main
