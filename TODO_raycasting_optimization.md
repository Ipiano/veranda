# Design Proposal: Raycasting Optimization

## Overview

This document proposes optimizations to the sensor raycasting system in Veranda, which is a significant performance bottleneck when running multiple lidar sensors at high update rates.

> **Related documents**:
> - [TODO.md](TODO.md) - Phase 3.1 (Optimize sensor raycasting)
>
> **Note**: These optimizations should be applied **after** completing Phase 1 and Phase 2 infrastructure modernization. Profile first to identify actual bottlenecks before implementing.

## Current State

### Implementation

The current lidar sensor implementation (`lidar_sensor.cpp:211-256`) performs individual Box2D raycasts for each ray:

```cpp
void Lidar_Sensor::_worldTicked(const double dt) {
    // ... timing check ...

    for(int i=0; i<scan_image.size(); i++, curr_angle += data->angle_increment) {
        b2Vec2 localEndpoint = _getRayPoint(curr_angle, scan_radius);
        b2Vec2 worldEndpoint = sensorBody->GetWorldPoint(localEndpoint);

        // Individual raycast per ray
        QPair<b2Vec2, double> rayCastResult =
            _rayCaster.rayCast(_world, worldOrigin, worldEndpoint, -objectId);

        // Update visual model
        b2EdgeShape* thisLine = dynamic_cast<b2EdgeShape*>(scan_image[i]);
        thisLine->m_vertex2 = sensorBody->GetLocalPoint(rayCastResult.first);

        data->ranges[i] = rayCastResult.second;
        // ... min/max tracking ...
    }

    // Publish and redraw
    if(_sendChannel) _sendChannel->publish(data);
    scan_model->forceDraw();
}
```

### Performance Analysis

**Computational Cost:**
- N sensors × M rays × P publish rate = total raycasts/second
- Example: 4 lidars × 360 rays × 30 Hz = 43,200 raycasts/second

**Current Bottlenecks:**

1. **Per-Ray Overhead**: Each raycast has callback setup cost
2. **Model Updates**: Visual line models rebuilt every scan
3. **Signal Emissions**: `forceDraw()` triggers Qt signal cascade
4. **Memory Allocation**: `dynamic_cast` on every ray iteration
5. **Trigonometric Calculations**: `cos`/`sin` per ray per scan

### Profiling Results (Hypothetical Baseline)

```
Profile: 4 lidars, 360 rays each, 30 Hz

Function                          | % Time | Calls/sec
----------------------------------|--------|----------
b2World::RayCast                  | 45%    | 43,200
Model::forceDraw (signal chain)   | 20%    | 120
_getRayPoint (trig)               | 12%    | 43,200
dynamic_cast<b2EdgeShape*>        | 8%     | 43,200
QGraphicsView redraw              | 10%    | 120
Other                             | 5%     | -
```

## Proposed Optimizations

### 1. Batch Raycasting

Instead of individual raycasts, collect all rays and process together:

```cpp
class BatchRayCaster : public b2RayCastCallback {
    struct RayResult {
        b2Vec2 hitPoint;
        float distance;
        bool hit;
    };

    std::vector<RayResult> _results;
    std::vector<std::pair<b2Vec2, b2Vec2>> _rays;  // start, end pairs
    int64_t _ignoreGroup;
    size_t _currentRay;

public:
    void addRay(const b2Vec2& start, const b2Vec2& end) {
        _rays.emplace_back(start, end);
        _results.emplace_back();
    }

    void execute(const b2World* world, int64_t ignoreGroup) {
        _ignoreGroup = ignoreGroup;
        _results.resize(_rays.size());

        for (_currentRay = 0; _currentRay < _rays.size(); ++_currentRay) {
            auto& [start, end] = _rays[_currentRay];
            _results[_currentRay] = {end, b2Distance(start, end), false};
            world->RayCast(this, start, end);
        }
    }

    float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                          const b2Vec2& normal, float32 fraction) override {
        if (fixture->IsSensor() ||
            fixture->GetFilterData().groupIndex == _ignoreGroup) {
            return 1.0f;  // Continue
        }

        auto& result = _results[_currentRay];
        float dist = b2Distance(_rays[_currentRay].first, point);
        if (dist < result.distance) {
            result.hitPoint = point;
            result.distance = dist;
            result.hit = true;
        }
        return fraction;
    }

    const std::vector<RayResult>& results() const { return _results; }

    void clear() {
        _rays.clear();
        _results.clear();
    }
};
```

### 2. Precomputed Ray Directions

Cache trigonometric calculations:

```cpp
class Lidar_Sensor : public WorldObjectComponent {
    // Precomputed ray directions (relative to sensor)
    std::vector<b2Vec2> _rayDirections;
    bool _rayDirectionsDirty = true;

    void _rebuildRayDirections() {
        if (!_rayDirectionsDirty) return;

        int numRays = scan_points.get().toInt();
        double range = angle_range.get().toDouble() * DEG2RAD;
        double radius = this->radius.get().toDouble();

        _rayDirections.resize(numRays);

        double startAngle = -range / 2.0;
        double angleStep = range / (numRays - 1);

        for (int i = 0; i < numRays; ++i) {
            double angle = startAngle + i * angleStep;
            _rayDirections[i] = b2Vec2(
                std::cos(angle) * radius,
                std::sin(angle) * radius
            );
        }

        _rayDirectionsDirty = false;
    }

    // Mark dirty when parameters change
    void _onParameterChanged() {
        _rayDirectionsDirty = true;
    }
};
```

### 3. Deferred Visual Updates

Separate scan computation from visualization:

```cpp
class Lidar_Sensor : public WorldObjectComponent {
    // Scan data (updated every scan)
    std::vector<float> _scanDistances;
    std::atomic<bool> _scanDataDirty{false};

    // Visual update rate (can be lower than scan rate)
    double _visualUpdateRate = 10.0;  // Hz
    double _timeSinceVisualUpdate = 0;

    void _worldTicked(const double dt) override {
        _timeSinceScan += dt;
        double scanPeriod = 1.0 / pub_rate.get().toDouble();

        if (_timeSinceScan >= scanPeriod) {
            _performScan();  // Fast: just raycasts and data
            _timeSinceScan = 0;
        }

        // Visual updates at lower rate
        _timeSinceVisualUpdate += dt;
        if (_timeSinceVisualUpdate >= 1.0 / _visualUpdateRate) {
            if (_scanDataDirty.exchange(false)) {
                _updateVisualModel();  // Slower: graphics updates
            }
            _timeSinceVisualUpdate = 0;
        }
    }

    void _performScan() {
        // Fast path: just collect distances
        _batchCaster.clear();
        b2Vec2 origin = sensorBody->GetPosition();

        for (const auto& dir : _rayDirections) {
            b2Vec2 endpoint = sensorBody->GetWorldPoint(dir);
            _batchCaster.addRay(origin, endpoint);
        }

        _batchCaster.execute(_world, -objectId);

        // Update data array
        const auto& results = _batchCaster.results();
        for (size_t i = 0; i < results.size(); ++i) {
            _scanDistances[i] = results[i].distance;
        }

        // Publish to ROS
        _publishScan();

        _scanDataDirty = true;
    }

    void _updateVisualModel() {
        // Slow path: update graphics
        for (size_t i = 0; i < scan_image.size(); ++i) {
            auto* line = static_cast<b2EdgeShape*>(scan_image[i]);
            float dist = _scanDistances[i];
            line->m_vertex2 = b2Vec2(
                _rayDirections[i].x * dist / radius.get().toDouble(),
                _rayDirections[i].y * dist / radius.get().toDouble()
            );
        }
        scan_model->forceDraw();
    }
};
```

### 4. Spatial Hashing for Dense Scenes

For scenes with many objects, add spatial acceleration:

```cpp
class SpatialHashGrid {
    struct Cell {
        std::vector<b2Fixture*> fixtures;
    };

    float _cellSize;
    std::unordered_map<int64_t, Cell> _cells;

    int64_t _hash(int x, int y) const {
        return (static_cast<int64_t>(x) << 32) | static_cast<uint32_t>(y);
    }

    std::pair<int, int> _toCell(const b2Vec2& point) const {
        return {
            static_cast<int>(std::floor(point.x / _cellSize)),
            static_cast<int>(std::floor(point.y / _cellSize))
        };
    }

public:
    SpatialHashGrid(float cellSize = 5.0f) : _cellSize(cellSize) {}

    void rebuild(b2World* world) {
        _cells.clear();

        for (b2Body* body = world->GetBodyList(); body; body = body->GetNext()) {
            for (b2Fixture* fix = body->GetFixtureList(); fix; fix = fix->GetNext()) {
                b2AABB aabb;
                fix->GetShape()->ComputeAABB(&aabb, body->GetTransform(), 0);

                auto [minX, minY] = _toCell(aabb.lowerBound);
                auto [maxX, maxY] = _toCell(aabb.upperBound);

                for (int x = minX; x <= maxX; ++x) {
                    for (int y = minY; y <= maxY; ++y) {
                        _cells[_hash(x, y)].fixtures.push_back(fix);
                    }
                }
            }
        }
    }

    // Get fixtures that might intersect a ray
    std::vector<b2Fixture*> queryRay(const b2Vec2& start, const b2Vec2& end) const {
        std::unordered_set<b2Fixture*> result;

        // Bresenham-style ray march through cells
        auto [x0, y0] = _toCell(start);
        auto [x1, y1] = _toCell(end);

        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true) {
            auto it = _cells.find(_hash(x0, y0));
            if (it != _cells.end()) {
                for (auto* fix : it->second.fixtures) {
                    result.insert(fix);
                }
            }

            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }

        return {result.begin(), result.end()};
    }
};
```

### 5. SIMD Optimization for Ray-Shape Tests

Use SIMD for parallel ray-circle intersection tests:

```cpp
#include <immintrin.h>  // SSE/AVX

class SIMDRayTester {
public:
    // Test 4 rays against one circle simultaneously
    void testRaysAgainstCircle(
        const float* rayOriginX, const float* rayOriginY,
        const float* rayDirX, const float* rayDirY,
        float circleX, float circleY, float circleR,
        float* outDistances)
    {
        // Load ray data (4 rays at once)
        __m128 ox = _mm_loadu_ps(rayOriginX);
        __m128 oy = _mm_loadu_ps(rayOriginY);
        __m128 dx = _mm_loadu_ps(rayDirX);
        __m128 dy = _mm_loadu_ps(rayDirY);

        // Circle center broadcast
        __m128 cx = _mm_set1_ps(circleX);
        __m128 cy = _mm_set1_ps(circleY);
        __m128 r2 = _mm_set1_ps(circleR * circleR);

        // Vector from ray origin to circle center
        __m128 ocx = _mm_sub_ps(cx, ox);
        __m128 ocy = _mm_sub_ps(cy, oy);

        // Project onto ray direction
        __m128 tca = _mm_add_ps(_mm_mul_ps(ocx, dx), _mm_mul_ps(ocy, dy));

        // Distance squared from circle center to closest point on ray
        __m128 oc2 = _mm_add_ps(_mm_mul_ps(ocx, ocx), _mm_mul_ps(ocy, ocy));
        __m128 d2 = _mm_sub_ps(oc2, _mm_mul_ps(tca, tca));

        // Check if ray misses circle
        __m128 hitMask = _mm_cmple_ps(d2, r2);

        // Distance from closest point to intersection
        __m128 thc = _mm_sqrt_ps(_mm_sub_ps(r2, d2));

        // Final intersection distance
        __m128 t = _mm_sub_ps(tca, thc);

        // Store results (infinity for misses)
        __m128 inf = _mm_set1_ps(std::numeric_limits<float>::infinity());
        __m128 result = _mm_blendv_ps(inf, t, hitMask);

        _mm_storeu_ps(outDistances, result);
    }
};
```

### 6. Level-of-Detail for Distant Scans

Reduce ray count for distant or fast-moving objects:

```cpp
class AdaptiveLidar : public Lidar_Sensor {
    int _computeAdaptiveRayCount() const {
        int baseRays = scan_points.get().toInt();

        // Reduce rays based on velocity (fast motion = blur anyway)
        b2Vec2 velocity = sensorBody->GetLinearVelocity();
        float speed = velocity.Length();
        float speedFactor = std::max(0.25f, 1.0f - speed / 10.0f);

        // Reduce rays based on visual distance (zoom level)
        float visualScale = getVisualScale();  // From graphics view
        float scaleFactor = std::min(1.0f, visualScale / 0.5f);

        int adaptiveRays = static_cast<int>(baseRays * speedFactor * scaleFactor);
        return std::max(16, adaptiveRays);  // Minimum 16 rays
    }
};
```

## Implementation Phases

### Phase 1: Measurement

1. Add profiling instrumentation
2. Measure current performance with various configurations
3. Identify actual bottlenecks (not assumed)

```cpp
// Add to lidar_sensor.cpp
#ifdef VERANDA_PROFILING
#include <chrono>

struct ScanProfiler {
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point start;
    double& accumulator;

    ScanProfiler(double& acc) : accumulator(acc), start(Clock::now()) {}
    ~ScanProfiler() {
        auto end = Clock::now();
        accumulator += std::chrono::duration<double, std::milli>(end - start).count();
    }
};

// Usage
double raycastTime = 0, visualTime = 0;
{
    ScanProfiler p(raycastTime);
    // raycast code
}
{
    ScanProfiler p(visualTime);
    // visual update code
}
#endif
```

### Phase 2: Quick Wins

1. Implement precomputed ray directions
2. Replace `dynamic_cast` with `static_cast`
3. Cache scan array allocations
4. Add visual update rate limiting

### Phase 3: Batch Processing

1. Implement `BatchRayCaster`
2. Integrate with lidar sensor
3. Benchmark improvement

### Phase 4: Advanced Optimizations

1. Implement spatial hashing (if profiling shows benefit)
2. Consider SIMD for specific bottlenecks
3. Add LOD system

### Phase 5: Validation

1. Verify scan accuracy unchanged
2. Performance benchmarks
3. Memory usage check
4. Integration testing

## Expected Results

### Performance Targets

| Scenario | Current | Optimized | Improvement |
|----------|---------|-----------|-------------|
| 1 lidar, 360 rays, 30 Hz | 100% CPU baseline | 40% | 2.5x |
| 4 lidars, 360 rays, 30 Hz | ~400% (bottleneck) | 120% | 3.3x |
| 1 lidar, 360 rays, 100 Hz | Not achievable | 80% | ∞ |
| 8 lidars, 180 rays, 30 Hz | ~300% | 100% | 3x |

### Memory Impact

| Optimization | Memory Delta |
|--------------|--------------|
| Precomputed directions | +2.8 KB per lidar (360 rays × 8 bytes) |
| Batch caster | +5.6 KB per lidar (temporary) |
| Spatial hash | +100 KB - 1 MB depending on scene |

## Testing Strategy

### Correctness Tests

```cpp
TEST_CASE("Batch raycaster produces same results as individual") {
    b2World world(b2Vec2(0, 0));
    // Setup test scene with known geometry

    // Individual raycasts
    std::vector<float> individualResults;
    for (auto& ray : testRays) {
        SingleRayCaster caster;
        float dist = caster.cast(&world, ray.start, ray.end, 0);
        individualResults.push_back(dist);
    }

    // Batch raycast
    BatchRayCaster batch;
    for (auto& ray : testRays) {
        batch.addRay(ray.start, ray.end);
    }
    batch.execute(&world, 0);

    // Compare
    for (size_t i = 0; i < testRays.size(); ++i) {
        REQUIRE(batch.results()[i].distance ==
                Approx(individualResults[i]).margin(0.001));
    }
}
```

### Performance Tests

```cpp
BENCHMARK("Lidar scan performance") {
    Lidar_Sensor sensor(/*...*/);
    sensor.generateBodies(world, 1, anchor);

    // Warm up
    for (int i = 0; i < 10; ++i) {
        sensor._worldTicked(1.0/30.0);
    }

    // Measure
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        sensor._worldTicked(1.0/30.0);
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration<double, std::milli>(end - start);
    double avgMs = duration.count() / 1000.0;

    REQUIRE(avgMs < 1.0);  // Must complete in under 1ms
}
```

## Alternative Approaches Considered

### GPU Raycasting

**Pros**: Massive parallelism, can handle thousands of rays
**Cons**: GPU↔CPU transfer overhead, requires OpenGL/Vulkan, complex integration
**Decision**: Defer to future if CPU optimization insufficient

### Approximate Methods

**Pros**: O(1) queries using signed distance fields
**Cons**: Pre-computation cost, memory for SDF grid, less accurate
**Decision**: Not suitable for dynamic scenes

### Physics Engine Change

**Pros**: Some engines have optimized batch raycasting
**Cons**: Major refactor, Box2D is well-integrated
**Decision**: Optimize Box2D usage first

## References

- [Box2D Raycasting Documentation](https://box2d.org/documentation/)
- [Spatial Hashing for Collision Detection](http://www.cs.ucf.edu/~jmesit/publications/scsc%202005.pdf)
- [SIMD Programming Guide](https://www.intel.com/content/www/us/en/docs/intrinsics-guide/)
- [Real-Time Collision Detection (Ericson)](https://realtimecollisiondetection.net/)
