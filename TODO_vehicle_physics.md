# Vehicle Physics Refactor: Mecanum Wheels and Rigid Islands

## Overview

The current implementation uses Box2D for full dynamics with high-mass wheels, low-mass chassis, and stiff spring/weld joints. This causes issues with mecanum wheel force cancellation and numerical instability. The solution is to use Box2D primarily for collision detection while computing vehicle dynamics ourselves.

> **Related documents**:
> - [TODO.md](TODO.md) - Phase 3.3 (Vehicle physics refactor)
>
> **Note**: This is a **specialized optimization** for mecanum wheel configurations. Only implement if mecanum wheels are a critical feature. This should be done after Phase 1 and Phase 2 infrastructure work.

## Problem Summary

- Box2D assumes side-view physics with gravity; top-down requires fighting its friction model
- Mecanum wheels have anisotropic friction (high perpendicular to roller axis, near-zero parallel) which Box2D can't model natively
- Per-wheel force application on separate bodies causes incorrect torque calculations—constraint solver propagates forces between bodies, introducing numerical error
- Multi-body chassis joined with welds has the same propagation problem

---

## Implementation Steps

### 1. Identify Rigid Islands at Build Time

When the user constructs or modifies a robot, build a connectivity graph and partition into rigid islands:

```cpp
struct RigidIsland {
    std::vector<b2Body*> bodies;
    std::vector<Wheel*> wheels;  // wheels attached to this island
    float totalMass;
    b2Vec2 centerOfMass;
    float momentOfInertia;
};
```

**Algorithm:**
- Bodies are nodes, joints are edges
- Flood-fill through only rigid joints (welds, fixed constraints)
- Stop at pivot joints, driven joints, or other non-rigid connections
- Each connected component becomes one island

**Recompute when:**
- User adds/removes parts
- User changes joint type

---

### 2. Zero Out Friction on Mecanum Wheels

For each mecanum wheel body:

```cpp
for (b2Fixture* f = wheelBody->GetFixtureList(); f; f = f->GetNext()) {
    f->SetFriction(0.0f);
}
```

This prevents Box2D's isotropic friction model from interfering. We apply all wheel forces explicitly.

---

### 3. Per-Island Wheel Force Summation

For each wheel in an island, compute its force contribution using the mecanum force model.

**Per-wheel parameters (in robot/island local frame):**
- Position $(r_x, r_y)$ relative to island COM
- Wheel heading $\psi$ (direction the wheel points)
- Roller angle $\alpha$ (typically ±45°, but configurable)
- Motor effort $u$ (scalar force magnitude from motor command)

**Force direction:**

$$\phi = \psi + \alpha + 90°$$

**Force vector:**

$$\vec{F} = u \begin{pmatrix} \cos\phi \\ \sin\phi \end{pmatrix}$$

**Torque about island COM:**

$$\tau = r_x \sin\phi - r_y \cos\phi$$

Or equivalently: $\tau = \vec{r} \times \vec{F}$ (2D cross product gives scalar)

**Sum across all wheels in the island:**

$$F_x^{net} = \sum_i u_i \cos\phi_i$$
$$F_y^{net} = \sum_i u_i \sin\phi_i$$
$$\tau^{net} = \sum_i u_i (r_{x_i} \sin\phi_i - r_{y_i} \cos\phi_i)$$

**Add external torques** (e.g., from driven joints connecting to other islands):

$$\tau^{net} \mathrel{+}= \tau_{external}$$

**Compute accelerations:**

$$\vec{a} = \frac{\vec{F}^{net}}{M}$$
$$\dot{\omega} = \frac{\tau^{net}}{I}$$

---

### 4. Set Velocities Directly on All Bodies in the Island

This is the critical step that bypasses Box2D's constraint solver for intra-island dynamics.

#### 4.1 Core Concept

A rigid island rotates as a unit about its COM. Every body in the island shares the same angular velocity, but their linear velocities differ based on their position relative to the COM.

For a point at position $\vec{p}$ in a rigid body rotating about COM with angular velocity $\omega$:

$$\vec{v}_{point} = \vec{v}_{COM} + \omega \times (\vec{p} - \vec{p}_{COM})$$

In 2D, the cross product of a scalar angular velocity with a 2D vector gives:

$$\omega \times \vec{r} = \omega \cdot (-r_y, r_x) = (-\omega \cdot r_y, \omega \cdot r_x)$$

#### 4.2 Per-Tick Update Procedure

```cpp
void updateIslandVelocities(RigidIsland& island, float dt) {
    // 1. Recompute COM from current body positions (in case of external collision displacement)
    b2Vec2 com = computeCenterOfMass(island);
    
    // 2. Get current island velocity (from any body, transformed to COM)
    //    Or track this explicitly on the island struct
    b2Vec2 linearVel = island.linearVelocity;
    float angularVel = island.angularVelocity;
    
    // 3. Compute net force and torque from wheels (see step 3)
    b2Vec2 netForce = computeNetForce(island);
    float netTorque = computeNetTorque(island, com);
    
    // 4. Add drag forces (optional but recommended for realism)
    //    Linear drag opposes velocity
    netForce -= linearDragCoeff * linearVel;
    //    Angular drag opposes rotation
    netTorque -= angularDragCoeff * angularVel;
    
    // 5. Compute accelerations
    b2Vec2 linearAccel = (1.0f / island.totalMass) * netForce;
    float angularAccel = netTorque / island.momentOfInertia;
    
    // 6. Integrate velocities
    linearVel += dt * linearAccel;
    angularVel += dt * angularAccel;
    
    // 7. Store updated island velocities
    island.linearVelocity = linearVel;
    island.angularVelocity = angularVel;
    
    // 8. Apply to all bodies in the island
    for (b2Body* body : island.bodies) {
        b2Vec2 r = body->GetPosition() - com;
        
        // Velocity of this body = COM velocity + rotational component
        b2Vec2 bodyVel;
        bodyVel.x = linearVel.x - angularVel * r.y;
        bodyVel.y = linearVel.y + angularVel * r.x;
        
        body->SetLinearVelocity(bodyVel);
        body->SetAngularVelocity(angularVel);
    }
}
```

#### 4.3 Computing Center of Mass

```cpp
b2Vec2 computeCenterOfMass(const RigidIsland& island) {
    b2Vec2 com(0.0f, 0.0f);
    float totalMass = 0.0f;
    
    for (b2Body* body : island.bodies) {
        float m = body->GetMass();
        com += m * body->GetPosition();
        totalMass += m;
    }
    
    return (1.0f / totalMass) * com;
}
```

#### 4.4 Computing Moment of Inertia About COM

Use parallel axis theorem: $I_{total} = \sum_i (I_i + m_i d_i^2)$

```cpp
float computeMomentOfInertia(const RigidIsland& island, b2Vec2 com) {
    float I = 0.0f;
    
    for (b2Body* body : island.bodies) {
        float m = body->GetMass();
        float I_body = body->GetInertia();  // inertia about body's own COM
        b2Vec2 r = body->GetPosition() - com;
        float d_squared = r.x * r.x + r.y * r.y;
        
        I += I_body + m * d_squared;  // parallel axis theorem
    }
    
    return I;
}
```

#### 4.5 Handling Collisions

Box2D will still apply collision impulses to individual bodies. After Box2D's step, the bodies may have inconsistent velocities. Options:

**Option A: Re-read velocity from one body and redistribute**
```cpp
// After b2World::Step()
// Pick a reference body and compute what the island velocity "should" be
b2Body* ref = island.bodies[0];
b2Vec2 r = ref->GetPosition() - com;
b2Vec2 refVel = ref->GetLinearVelocity();

// Back-calculate island COM velocity
// refVel = comVel + omega × r
// This is approximate; for accuracy, average across all bodies
island.angularVelocity = ref->GetAngularVelocity();
island.linearVelocity.x = refVel.x + island.angularVelocity * r.y;
island.linearVelocity.y = refVel.y - island.angularVelocity * r.x;
```

**Option B: Average impulses across island (more accurate)**
```cpp
// Compute momentum-weighted average of all body velocities
b2Vec2 totalMomentum(0, 0);
float totalAngularMomentum = 0;

for (b2Body* body : island.bodies) {
    float m = body->GetMass();
    totalMomentum += m * body->GetLinearVelocity();
    
    b2Vec2 r = body->GetPosition() - com;
    b2Vec2 v = body->GetLinearVelocity();
    totalAngularMomentum += body->GetInertia() * body->GetAngularVelocity();
    totalAngularMomentum += m * (r.x * v.y - r.y * v.x);  // r × v
}

island.linearVelocity = (1.0f / island.totalMass) * totalMomentum;
island.angularVelocity = totalAngularMomentum / island.momentOfInertia;
```

Then redistribute velocities as in 4.2 step 8.

#### 4.6 Coordinate Frame Considerations

The force summation (step 3) is easiest in the **island's local frame**. The velocity updates (step 4) must be in **world frame**.

When computing wheel positions and orientations for the force sum:
- Get wheel's world position and angle from Box2D
- Subtract island COM (world coords) to get $\vec{r}$
- Wheel heading $\psi$ is already in world frame

Net force comes out in world frame, so no final rotation needed.

---

### 5. Let Box2D Handle Collisions and Pivot Joints

**Collisions:**
- Box2D's broadphase and narrowphase still detect collisions
- Collision impulses are applied to individual bodies
- After each `b2World::Step()`, re-synchronize island velocities (see 4.5)

**Pivot joints between islands:**
- Box2D's constraint solver keeps pivot points coincident
- Each island computes its own dynamics independently
- Natural relative rotation emerges

**Driven joints between islands:**
- Compute motor torque based on joint angle and target
- Apply equal and opposite torques to each island:
  ```cpp
  float motorTorque = kp * (targetAngle - currentAngle);
  islandA.externalTorque -= motorTorque;
  islandB.externalTorque += motorTorque;
  ```

---

## Additional Notes

### Wheel Types

| Type | Force Direction | Constraint |
|------|-----------------|------------|
| Mecanum | Perpendicular to roller axis ($\psi + \alpha + 90°$) | None (roller spins freely) |
| Omni | Wheel heading ($\psi$) | None (roller spins freely) |
| Standard | Wheel heading ($\psi$) | High friction perpendicular to heading |

Standard wheels can be approximated by adding a lateral friction force opposing sideways velocity, or by letting Box2D's friction model handle it (set friction back to non-zero for standard wheels).

### Mixing Wheel Types

Technically supported, but standard wheels' lateral friction dominates, eliminating omnidirectional capability. Consider warning users if they mix types.

### Non-Standard Mecanum Configurations

The math works for any number of wheels and any roller angle. Validate that the resulting $B$ matrix has rank 3; if not, warn the user their configuration is under-actuated.

---

## Testing Checklist

- [ ] Single 4-wheel mecanum robot: strafe, rotate, translate all work
- [ ] Diagonal roller angles (verify force direction calculation)
- [ ] Non-45° roller angles
- [ ] 3-wheel mecanum (under-actuated case, should warn)
- [ ] 6-wheel mecanum (over-actuated, should work)
- [ ] Two-island configuration (tractor-trailer)
- [ ] Collision with wall doesn't break island velocity sync
- [ ] Encoder readings match actual wheel motion
