# Expression Reference

Every field with an **=** or a condition takes an expression. Every value is a number: `true` is 1 and `false` is 0, and any non-zero value counts as true. Angles are in **degrees**, lengths in **metres** and times in **seconds**.

## Operators

Listed from lowest to highest precedence:

| Operator | Meaning |
|---|---|
| `c ? a : b` | if *c* then *a* else *b* |
| `||` | or |
| `&&` | and |
| `==`  `!=` | equal, not equal |
| `<`  `<=`  `>`  `>=` | comparison |
| `+`  `-` | add, subtract |
| `*`  `/`  `%` | multiply, divide, remainder |
| `!`  `-` | not, negate (unary) |

## Functions

| Function | Result |
|---|---|
| `abs(x)` `sign(x)` | absolute value; -1, 0 or 1 |
| `min(a, b, …)` `max(a, b, …)` | smallest or largest of up to 16 values |
| `clamp(x, lo, hi)` | *x* limited to the range |
| `floor(x)` `ceil(x)` `round(x)` | rounding |
| `sqrt(x)` | square root |
| `sin(deg)` `cos(deg)` `tan(deg)` | trigonometry, in **degrees** |
| `time()` | seconds since Play |
| `rising(x)` | true for one scan when *x* turns true |
| `falling(x)` | true for one scan when *x* turns false |
| `changed(x)` | true for one scan when *x* changes |

## Names you can use

- Any signal: `Start`, `SpeedPct`
- Actuator values: `J1.pos`, `J1.vel`, `J1.target`, `J1.inPosition`, `J1.moving`, `J1.atMin`, `J1.atMax`, `J1.active`
- Sequence values: `Cycle.running`, `Cycle.done`, `Cycle.cycles`
- `true` and `false`

Typing in an expression field suggests names. A red field means the expression has an error; hover over it to see the problem.

## Actions

*On start* and *On finish* take assignments separated by `;`. Only Input and Internal signals can be assigned:

```
Gripper = true; State = 2
```

## Handy patterns

Choose between two values:

```
Fast ? 720 : 180
```

Ping-pong between ±Amplitude every two seconds:

```
(floor(time() / 2) % 2 == 0) ? Amplitude : -Amplitude
```

Smooth oscillation:

```
Amplitude * sin(time() * 180)
```

Keep a value in range:

```
clamp(SpeedPct, 10, 100)
```

Within tolerance:

```
abs(J1.pos - 90) < 0.5
```
