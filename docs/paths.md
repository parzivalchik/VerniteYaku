# Paths

## Coordinates

**The frame is the field.** Origin at the centre of the field, `+x` and `+y` in
the floor plane, heading CCW-positive radians from `+x`. The field is 144 inches
square, so every coordinate is in [-72, +72].

So `new Point(-36, -36)` is a fixed spot on the field, and it means the same
thing whichever tile the robot starts on. This is the same convention the FTC
SDK's AprilTag support uses, which is what lets a tag observation drop straight
into the pose filter.

`FieldCoordinates` holds the constants and a few helpers — field size, tile size,
`contains()` for checking a plan fits inside the walls, and `rotated180()` for
mirroring a plan to the other alliance.

### You must tell it where the robot starts

Absolute coordinates cannot be inferred from encoders, so the localizer needs a
`startPose`:

```java
FusedLocalizer localizer = FusedLocalizer.builder(drivetrain, Clock.system())
        .startPose(new Pose2d(-60, -36, 0))    // where the robot is placed
        .startPose(new Pose2d(-60, -36, 0))
        .build();
```

Get it wrong and every path in the auto is offset by the same amount, which looks
like the robot *driving the right shape in the wrong place*. It is the first
thing to check when an auto is uniformly off.

### The one axis question

Origin and handedness are fixed. **Which physical wall `+x` points at is a
choice**, and it has to match your AprilTag layout and IMU zero. The library
deliberately does not bake in a season-specific wall mapping.

Confirm it once on a real field: place the robot at a known spot facing a known
wall, read `getPose()`, and check the signs. If `+x` points at the opposite wall
from what you assumed, use `FieldCoordinates.rotated180()` on the whole plan
rather than negating coordinates one at a time.

---

## Units

Every builder takes a `DistanceUnit` and converts at the boundary. Internally
everything is inches, but you never have to work in them:

```java
new PathBuilder().setUnit(DistanceUnit.CM).line(0, 0, 60, 0)   // 60 cm
Point.of(60.96, 0, DistanceUnit.CM)                            // 24 inches
follower.getPose().getX(DistanceUnit.CM)                       // read back in cm
```

`INCH`, `CM`, `MM`, `METER`. Because conversion happens once at the edge, a
centimetre value cannot leak into a computation expecting inches.

`setUnit()` affects the builder's own `line()` and `curve()` helpers. A `Point`
you construct yourself carries its own unit from `Point.of(...)`.

**Angles are always radians**, everywhere, with no unit switch.

---

## Segments

```java
new BezierLine(new Point(-60, -36), new Point(-36, -36))
new BezierCurve(new Point(-36, -36), new Point(-24, -24), new Point(-24, -6))
```

`BezierLine` is a straight segment — a degree-1 Bezier. `BezierCurve` takes three
or more control points and supports any degree; the curve passes through its
first and last points and is only pulled toward the intermediate ones.

Both are `BezierPath` underneath, evaluated with de Casteljau's algorithm.

### Arc length, not parameter

A Bezier's parameter `t` is **not** proportional to distance — the curve moves
faster through the middle of its parameter range than near its ends. Anything
that needs even spacing goes through the arc-length methods, which are backed by
a lookup table built once at construction:

```java
path.tAtArcLength(12.0);   // the t that is 12 inches along
path.arcLengthAt(0.5);     // how far along t = 0.5 actually is
```

Stepping `t` directly gives you uneven spacing on the ground. This is the usual
reason a hand-rolled Bezier follower speeds up and slows down for no reason.

---

## Chains

A `PathChain` is several segments driven as one continuous motion. It presents
itself to the follower as a single curve parameterised by arc length, hiding the
boundaries — which is what lets one motion profile span the whole chain instead
of stopping and restarting at each junction.

```java
PathChain chain = new PathBuilder()
        .addPath(new BezierLine(new Point(-60, -36), new Point(-36, -36)))
        .setLinearHeadingInterpolation(0, Math.toRadians(90))
        .addPath(new BezierCurve(new Point(-36, -36), new Point(-24, -24), new Point(-24, -6)))
        .setConstantHeadingInterpolation(Math.toRadians(90))
        .setMaxVelocity(15)
        .build();
```

Make the first point the robot's starting position, so it does not have to drive
onto the path before it can follow it.

Each `set...` call applies to the path most recently added.

Segments are not required to be continuous. A gap or a corner between two
segments is legal — but a sharp corner is profiled as if it were smooth, so the
follower will cut it somewhat. Build a proper curve if that matters.

---

## Heading

Heading is independent of geometry, so the same curve can be driven nose-first,
held at a fixed angle for a shooter, or swept through a turn.

| Call | Behaviour |
|---|---|
| `setTangentHeadingInterpolation()` | Face along the direction of travel. **Default** |
| `setReverseTangentHeadingInterpolation()` | Face backwards along travel |
| `setConstantHeadingInterpolation(rad)` | Hold one angle for the segment |
| `setLinearHeadingInterpolation(a, b)` | Sweep from `a` to `b`, the short way |
| `setReversedHeadingInterpolation(a, b)` | Sweep the long way, through the reflex angle |

Linear interpolation takes the short way around the wrap, so 350° → 10° is a 20°
turn forward, not 340° back. Use `setReversedHeadingInterpolation` when you
specifically want the long way — usually because a mechanism or cable would
otherwise sweep through something.

Interpolation is linear in the curve parameter, not arc length, so on a strongly
non-uniform Bezier the turn is slightly front- or back-loaded. This matches what
Pedro Pathing does.

For anything else, implement `HeadingInterpolator` and pass it to
`setHeadingInterpolation()`.

---

## Speed caps

`setMaxVelocity()` caps one segment without slowing the whole run:

```java
.addPath(approach)
.setMaxVelocity(10)     // careful, lining up on the scoring position
.addPath(sprintBack)    // full speed again
```

A segment cap never *raises* the global limit — it is always `min(cap, global)`,
and a corner's own physical limit applies on top of it. See
[`maxLateralAcceleration`](tuning.md#maxlateralacceleration--cornering-grip).

The profile brakes into a capped segment **ahead of the boundary**, so the robot
arrives already slowed rather than braking impossibly hard at the junction. That
falls out of the forward-backward sweep described in
[Architecture](architecture.md#motion-profiling); it is the part a naive
per-segment profile gets wrong.

---

## Inspecting a chain

```java
chain.length();                       // total arc length, inches
chain.size();                         // segment count
chain.stateAtArcLength(12.0);         // point, tangent, heading, curvature there
chain.startState();  chain.endState();
chain.project(robotPosition, segIdx, tGuess);   // nearest point on the chain
```

`PathState` carries everything resolved at one point: `point`, `tangent`,
`heading`, `curvature`, `arcLength`, and which `segmentIndex` it fell on.

`project()` deliberately only searches the current segment and the next one. A
full-chain search lets a robot passing near an earlier segment jump backwards —
on a chain that loops past its own start, that means the follower decides it has
un-driven half the auto.

---

## Building paths visually

`tools/alliance-planner/index.html` — a standalone browser tool, no build step.

Drag path chains, scrub a timeline that runs the same profile the follower does,
and export ready-to-paste `pathBuilder()` code in the same field frame, so the
numbers on the canvas are the numbers you paste.

Plan for **one robot** — a visual path editor for your own auto — or **two**,
which additionally checks both alliance partners' paths against each other.
Either way it tests against **obstacles** you define, using the same exact
rotated-rectangle intersection, with a 2" safety margin by default.

It keeps **named plan variants** — two or three candidate routes side by side,
with New / Duplicate / Remove — rather than one plan you overwrite every time you
try something.

Press `M` for a ruler and protractor: drag anywhere to read a distance and a
field heading, which is quicker than dropping a path point just to see a
coordinate.

Motion limits and robot dimensions in the planner are simulation settings. They
do not reach your robot's code — copy them across yourself.

It also supports mid-path **waits**. Those are not part of a `PathChain` — the
export flags them as comments — so sequence them yourself, for instance with
`Commands.waitSeconds()` between two `FollowPathCommand`s. See
[commands.md](commands.md).

See the [README](../README.md#alliance-planner).
