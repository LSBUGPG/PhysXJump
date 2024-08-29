# PhysXJump

This sample project creates a simple physics test to demonstrate lost energy when using impulse forces or setting initial velocities. The program sets up a physics world with a floor plane and a ball. The ball is either given an initial velocity or provided with an impulse to make it jump into the air. The values for ball radius, ball mass, and height of the jump are set for simulating tennis, but other values could be tested.

## Output

```
Test with initial velocity...
Initial energy is 0.53 J
Ball mechanical energy 0.5 J at peak height of 0.9 m

Test with impulse...
Impulse energy is 0.53 J
Ball mechanical energy 0.5 J at peak height of 0.9 m

Apply correction to initial velocity...
Initial energy is 0.53 J
Ball mechanical energy 0.53 J at peak height of 0.94 m

Apply correction to impulse...
Impulse energy is 0.53 J
Ball mechanical energy 0.53 J at peak height of 0.94 m
```

This example shows that, simulating a tennis game, the errors would be large enough to prevent the ball from clearing the net.

## Correction

What does the correction do?

Because PhysX uses Euler-Cromer simplectic integration, the velocity values lag the position values in time by half of the delta time interval. The correction calculates what the impulse force (or starting velocity) would have needed to be before the update so that the desired value is achieved at the point of the update.

One nice feature of this correction is that it makes the effect of the impulse independent of the delta time. Without this correction, changing the delta time changes the amount of energy lost during the first update frame and so the peak height of the ball.

## Problems

This correction depends on the internal implementation of the physics engine. It requires knowledge of all accelerations applied to the rigidbody before the update. If changes to the forces on the rigidbody are made in the same frame as the impulse, the adjustment would need to be different.

## Potential fix

This project contains two additional branches: `test-without-leapfrog-integration` and `test-leapfrog-integration`. These branches contain a modified version of the main branch test. They each contain the same code but different DLL's. These DLL's are from my fork of the current PhysX main branch: https://github.com/paulsinnett/PhysX and from a my own modified branch: https://github.com/paulsinnett/PhysX/tree/leapfrog-integration

These modified tests, test both the built in gravity and manually applied gravity. And there is an additional test to check that a two body orbital simulation remains stable and measures the error compared to the measured properties of the Earth - Sun orbit.

The `test-without-leapfrog-integration` produces this result:

```
Test initial velocity jump with built in gravity...
Initial energy is 0.53 J
Ball mechanical energy 0.5 J at peak height of 0.9 m

Test impulse jump with built in gravity...
Impulse energy is 0.53 J
Ball mechanical energy 0.5 J at peak height of 0.9 m

Test initial velocity jump with manual gravity...
Initial energy is 0.53 J
Ball mechanical energy 0.5 J at peak height of 0.9 m

Test impulse jump with manual gravity...
Impulse energy is 0.53 J
Ball mechanical energy 0.5 J at peak height of 0.9 m

Test orbit...
Aphelion error is 0.036 AU after 100 orbits
```

The `test-leapfrog-integration` produces this result:

```
Test initial velocity jump with built in gravity...
Initial energy is 0.53 J
Ball mechanical energy 0.53 J at peak height of 0.94 m

Test impulse jump with built in gravity...
Impulse energy is 0.53 J
Ball mechanical energy 0.53 J at peak height of 0.94 m

Test initial velocity jump with manual gravity...
Initial energy is 0.53 J
Ball mechanical energy 0.53 J at peak height of 0.94 m

Test impulse jump with manual gravity...
Impulse energy is 0.53 J
Ball mechanical energy 0.53 J at peak height of 0.94 m

Test orbit...
Aphelion error is 0.0045 AU after 100 orbits
```

This demonstrates that the leapfrog integration fix correctly applies impulses without losing energy, that it is stable in orbital simulation, and that the overal error is smaller than the original method.
