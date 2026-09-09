# Paths

## Coordinates

**The frame is start-relative.** The origin is wherever the robot is sitting when
the follower is constructed. `+x` points out its front at that instant, `+y` out
its left, and heading is CCW-positive radians from that initial forward
direction. Every auto begins at exactly `new Pose2d(0, 0, 0)`.

So `new Point(24, 0)` means *two feet ahead of where I started* — not a fixed
spot on the field. Move the robot a tile over and the whole path moves with it.
The library never knows where the field is.

### Getting field coordinates back

Hold the start pose yourself and compose:

```java
Pose2d startInField = new Pose2d(-60, -36, 0);       // you decide this
Pose2d fieldPose = startInField.transformBy(follower.getPose());
```

`transformBy` and `relativeTo` are exact inverses, so you can go either way. That
one line is the only place the two frames meet — there is no converter layer to
keep in sync.

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
new BezierLine(new Point(0, 0), new Point(24, 0))
new BezierCurve(new Point(24, 0), new Point(36, 12), new Point(36, 30))
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
        .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
        .setLinearHeadingInterpolation(0, Math.toRadians(90))
        .addPath(new BezierCurve(new Point(24, 0), new Point(36, 12), new Point(36, 30)))
        .setConstantHeadingInterpolation(Math.toRadians(90))
        .setMaxVelocity(15)
        .build();
```

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

A segment cap never *raises* the global limit — it is always `min(cap, global)`.

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
and export ready-to-paste `pathBuilder()` code in this same start-relative frame.

Plan for **one robot** — just a visual path editor for your own auto — or **two**,
which additionally checks both alliance partners' paths against each other for
collisions using exact rotated-rectangle intersection. Switch under *Plan for*.

See the [README](../README.md#alliance-planner).
