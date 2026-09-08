# Commands

**Optional.** `Follower` does not know this package exists. If you already use
FTCLib commands or Road Runner actions, ignore all of it — the follower is three
methods and adapts to either in about ten lines.

This package exists so that a team with no command framework does not have to
hand-roll a state machine with an integer and a switch.

---

## What it deliberately is not

No scheduler singleton. No subsystem model. No requirement declarations. No
exclusive-resource arbitration.

Those are the parts of a command library that force themselves on the rest of
your code, and a path-following library has no business imposing them. What is
here is composition and nothing else.

---

## Command

```java
public interface Command {
    default void initialize() {}
    void execute();
    boolean isFinished();
    default void end(boolean interrupted) {}
}
```

1. `initialize()` once, before the first execute
2. `execute()` every loop until `isFinished()` returns true
3. `end(interrupted)` once — `interrupted` says whether it finished on its own
   terms or was cancelled

A command must not block. One call, one loop's worth of progress, return.

---

## Running one

```java
CommandRunner runner = new CommandRunner(
        Commands.sequence(
                new FollowPathCommand(follower, toBasket),
                Commands.run(arm::raise),
                Commands.waitSeconds(0.4, clock),
                Commands.run(claw::open),
                new FollowPathCommand(follower, toPark)));

while (opModeIsActive() && !runner.isFinished()) {
    runner.run();
    telemetry.update();
}
```

`CommandRunner` owns exactly one command. Compose everything into one tree and
hand it over. Calling `run()` after it finishes is a no-op, not a restart.

`cancel()` stops the command where it is and gives it a chance to clean up.

---

## Combinators

| Factory | Behaviour |
|---|---|
| `Commands.none()` | Finishes immediately |
| `Commands.run(action)` | Runs once, finishes |
| `Commands.runUntil(action, until)` | Runs every loop until the condition holds |
| `Commands.waitSeconds(s, clock)` | Finishes after a delay |
| `Commands.waitUntil(condition)` | Finishes when the condition becomes true |
| `Commands.sequence(...)` | One after another |
| `Commands.parallel(...)` | All at once, finishes when **every** one has |
| `Commands.race(...)` | All at once, finishes when **any** one does |
| `Commands.deadline(first, ...)` | Finishes with `first`; the rest are interrupted |
| `Commands.either(cond, a, b)` | Picks a branch when it starts |

Fluent equivalents: `andThen`, `alongWith`, `raceWith`, `withTimeout(s, clock)`.

```java
new FollowPathCommand(follower, path)
        .alongWith(Commands.run(intake::spinUp))
        .withTimeout(5.0, clock);
```

A sequence advances past instant commands within a single loop, so a run of five
`Commands.run(...)` calls does not cost a tenth of a second of doing nothing.

---

## FollowPathCommand

The whole adapter, and the only place this package touches the follower:

```java
public void initialize()          { follower.followPath(path); }
public void execute()             { follower.update(); }
public boolean isFinished()       { return !follower.isBusy(); }
public void end(boolean cancelled) { if (cancelled) follower.breakFollowing(); }
```

Stopping on interruption matters — losing a race or being cancelled has to cut
the motors, or the follower keeps commanding them while whatever comes next also
tries to.

**Cleanup is per-command.** `FollowPathCommand` stops the drivetrain because it
knows it started it. A command that spun up a shooter has to switch it off in its
own `end()`; the runner has no way to know what needs undoing.

---

## Using a different framework

Write the same four lines against whatever you already have. FTCLib:

```java
public class FollowPath extends CommandBase {
    private final Follower follower;
    private final PathChain path;

    public FollowPath(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
    }

    @Override public void initialize() { follower.followPath(path); }
    @Override public void execute()    { follower.update(); }
    @Override public boolean isFinished() { return !follower.isBusy(); }
    @Override public void end(boolean interrupted) {
        if (interrupted) follower.breakFollowing();
    }
}
```

Road Runner actions:

```java
Action followPath(Follower follower, PathChain path) {
    boolean[] started = {false};
    return packet -> {
        if (!started[0]) { follower.followPath(path); started[0] = true; }
        follower.update();
        return follower.isBusy();
    };
}
```

That the coupling is this thin is the point.
