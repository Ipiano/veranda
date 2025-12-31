# Migration Plan: Catch2 to GoogleTest

This document outlines the work required to migrate the Veranda test suite from Catch2 to GoogleTest.

## Current State

| Aspect | Details |
|--------|---------|
| Test files | 2 (`test_model.cpp`, `test_property.cpp`) |
| Test cases | 10 |
| Nested sections | 44 |
| Assertions | ~220 `REQUIRE()` calls |
| Lines of test code | ~885 |
| Custom CMake macro | `ament_add_catch_test()` (~90 lines) |
| Catch2 version | 2.2.1 (single-header) |

### Current File Locations

**Catch2 Package:**
- `veranda/Packages/libraries/pkg-veranda_catch2/CMakeLists.txt`
- `veranda/Packages/libraries/pkg-veranda_catch2/package.xml`
- `veranda/Packages/libraries/pkg-veranda_catch2/include/catch2/catch.hpp`
- `veranda/Packages/libraries/pkg-veranda_catch2/include/catch2/catch_main.cpp`
- `veranda/Packages/libraries/pkg-veranda_catch2/include/catch2/ament_cmake_add_catch_test.cmake`

**Test Files:**
- `veranda/Packages/core/pkg-veranda_core_api/tests/test_model.cpp`
- `veranda/Packages/core/pkg-veranda_core_api/tests/test_property.cpp`

**Test Configuration:**
- `veranda/Packages/core/pkg-veranda_core_api/CMakeLists.txt` (lines 119-142)

---

## Migration Tasks

### 1. Replace pkg-veranda_catch2 Package

**Effort:** 1-2 hours

**Steps:**
- [ ] Rename package to `pkg-veranda_gtest` or similar
- [ ] Update `package.xml` with GoogleTest dependencies
- [ ] Remove `catch.hpp` and `catch_main.cpp`
- [ ] Add GoogleTest as a dependency (either vendored or system)
- [ ] Update `CMakeLists.txt` to export GoogleTest includes/libraries

**Notes:**
- GoogleTest requires linking a library (`gtest`, `gtest_main`) vs Catch2's single-header
- Consider using `ament_cmake_gtest` if available in target ROS2 version

### 2. Rewrite CMake Integration Macro

**Effort:** 2-3 hours

**Current macro:** `ament_add_catch_test()` in `ament_cmake_add_catch_test.cmake`

**Steps:**
- [ ] Create new `ament_cmake_add_gtest.cmake` macro
- [ ] Change test runner output format:
  - Catch2: `-r junit -o <file> -s`
  - GoogleTest: `--gtest_output=xml:<file>`
- [ ] Link `gtest_main` library instead of compiling `catch_main.cpp`
- [ ] Keep Qt MOC handling (unchanged)
- [ ] Keep ROS2 dependency handling (unchanged)
- [ ] Update test label from "Catch Test" to "GTest"

**Macro signature (keep compatible):**
```cmake
ament_add_gtest_test(target_name
    CPP_SOURCES <source_files>
    QT_HEADERS <headers_with_Q_OBJECT>
    QT_SOURCES <qt_source_files>
    QT_LIBS <Qt5_components>
    ROS_LIBS <ROS2_package_names>
    LIBS <other_libraries>
)
```

### 3. Convert Test Files

**Effort:** 4-6 hours total

#### Syntax Conversion Reference

| Catch2 | GoogleTest | Notes |
|--------|------------|-------|
| `#include <catch2/catch.hpp>` | `#include <gtest/gtest.h>` | |
| `TEST_CASE("description")` | `TEST(SuiteName, TestName)` | Suite name required |
| `SECTION("description")` | Separate `TEST()` or fixture | See below |
| `REQUIRE(expr)` | `ASSERT_TRUE(expr)` | Fatal on failure |
| `CHECK(expr)` | `EXPECT_TRUE(expr)` | Non-fatal |
| `REQUIRE(a == b)` | `ASSERT_EQ(a, b)` | |
| `REQUIRE(a != b)` | `ASSERT_NE(a, b)` | |
| `REQUIRE(a < b)` | `ASSERT_LT(a, b)` | |
| `REQUIRE(a > b)` | `ASSERT_GT(a, b)` | |
| `Approx(val)` | `ASSERT_NEAR(a, b, tol)` | Must specify tolerance |
| `REQUIRE_THROWS(expr)` | `ASSERT_THROW(expr, type)` | |
| `REQUIRE_NOTHROW(expr)` | `ASSERT_NO_THROW(expr)` | |

#### Handling Nested SECTION() Blocks

Catch2's `SECTION()` allows nested test organization that GoogleTest lacks. Conversion strategies:

**Option A: Flatten to separate tests (Recommended)**
```cpp
// Catch2
TEST_CASE("Model Transform") {
    Model m;
    SECTION("defaults to zero") {
        REQUIRE(m.x() == 0);
    }
    SECTION("setter works") {
        m.setX(5);
        REQUIRE(m.x() == 5);
    }
}

// GoogleTest
TEST(ModelTransform, DefaultsToZero) {
    Model m;
    ASSERT_EQ(m.x(), 0);
}

TEST(ModelTransform, SetterWorks) {
    Model m;
    m.setX(5);
    ASSERT_EQ(m.x(), 5);
}
```

**Option B: Use test fixtures for shared setup**
```cpp
class ModelTransformTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Common setup
    }
    Model m;
};

TEST_F(ModelTransformTest, DefaultsToZero) {
    ASSERT_EQ(m.x(), 0);
}

TEST_F(ModelTransformTest, SetterWorks) {
    m.setX(5);
    ASSERT_EQ(m.x(), 5);
}
```

#### Floating-Point Comparisons

Catch2's `Approx()` uses a default tolerance. GoogleTest requires explicit tolerance:

```cpp
// Catch2
REQUIRE(value == Approx(expected));

// GoogleTest - choose appropriate tolerance
ASSERT_NEAR(value, expected, 1e-10);  // absolute tolerance
// or use Google Mock matchers:
ASSERT_THAT(value, ::testing::DoubleNear(expected, 1e-10));
```

#### File-Specific Notes

**test_model.cpp (449 lines, 5 TEST_CASEs, 22 SECTIONs):**
- [ ] Convert `Get/Set Model Transform` (6 sections)
- [ ] Convert `Add/Remove shapes from model` (4 sections)
- [ ] Convert `Add/Remove fixtures from model` (4 sections)
- [ ] Convert `Add/Remove joints from model` (4 sections)
- [ ] Convert `Model dragging` (4 sections)
- [ ] Replace all `Approx()` with `ASSERT_NEAR()` with tolerance ~1e-10

**test_property.cpp (436 lines, 5 TEST_CASEs, 22 SECTIONs):**
- [ ] Convert `Get/Set Property value` (4 sections)
- [ ] Convert `Linking properties` (4 sections)
- [ ] Convert `Reading/Writing properties` (4 sections)
- [ ] Convert `Property Info` (6 sections)
- [ ] Convert `Property Validators` (4 sections)
- [ ] Qt signal testing (lambda captures) works the same way

### 4. Update Consuming CMakeLists.txt

**Effort:** < 1 hour

**File:** `veranda/Packages/core/pkg-veranda_core_api/CMakeLists.txt`

**Steps:**
- [ ] Change `find_package(veranda_catch2 REQUIRED)` to new package name
- [ ] Update include path for new CMake macro
- [ ] Change `ament_add_catch_test()` calls to new macro name

---

## Effort Summary

| Task | Estimated Hours |
|------|-----------------|
| New GoogleTest package setup | 1-2 |
| Rewrite CMake macro | 2-3 |
| Convert `test_model.cpp` | 2-3 |
| Convert `test_property.cpp` | 2-3 |
| Testing and debugging | 1-2 |
| **Total** | **8-13** |

---

## Considerations

### Arguments For Migration

- GoogleTest is more widely used in ROS2 ecosystem
- Better IDE integration and debugging support
- More active development and larger community
- Native support in many CI/CD systems
- `ament_cmake_gtest` may provide easier ROS2 integration

### Arguments Against Migration

- Small test suite (only 2 files) - limited benefit from migration
- Catch2's `SECTION()` pattern is more concise than GoogleTest fixtures
- Current system works with no apparent issues
- Will roughly double line count of test files due to flattening sections
- ROS2 Ardent is old; if upgrading ROS2 anyway, could bundle migration

### Recommendation

The migration is **straightforward but offers limited benefit** given the small test suite. The main complexity is converting nested `SECTION()` blocks to GoogleTest's flat test structure.

**If proceeding with migration:**
1. Use `TEST_F` fixtures to reduce code duplication from flattening sections
2. Define a tolerance constant for floating-point comparisons
3. Consider migrating when upgrading ROS2 version to bundle changes

---

## Verification Checklist

After migration, verify:

- [ ] All tests pass: `ament test --packages-select veranda_core_api`
- [ ] JUnit XML output generated correctly
- [ ] Qt signal tests still function
- [ ] Build works on both Linux and Windows
- [ ] `BUILD_TESTING=OFF` still compiles without test dependencies
