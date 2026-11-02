# Assembling Parts

Put parts together by their geometry instead of typing coordinates: seat a block flush on a plate, line up a shaft with a bore, drop a bolt into a hole. Each connection is a **mate**. Mates are kept in the project and hold while you keep editing.

## Make a mate

1. **Tools › Assemble** (or press **A**). The Assemble panel opens in the viewport.
2. Click a feature on the part that should **move**.
3. Click a feature on the part it goes against. That part **stays put**.

The first part moves into place right away, and the panel shows the new mate. Press **Enter** (or **Done**) to keep it and make the next one. **Esc** steps back: first it clears a half-made pick, then it closes the mate, then it leaves Assemble mode.

The hovered feature is outlined and labelled, for example *Pin · round edge Ø 40.0 mm*.

| You hover | You get |
|---|---|
| A flat face | a **plane**: the face and the side it points to |
| Near a round edge (hole rim, shaft end) | a **circle**: its centre and its axis |
| A round face (bore, shaft) | an **axis**: the centre line |
| Near a straight edge | an **axis** along the edge |
| Near the end of an edge | a **point** (corner) |

## Mate types

The panel lists the mates that fit the two features you picked, with the usual one first.

| Mate | Picks | What it does |
|---|---|---|
| **Flush faces** | face + face | The faces touch: against each other, or **Level** (pointing the same way, e.g. two tops at one height). **Offset** leaves a gap. |
| **Seat on face** | round edge + face | The rim sits on the face, with the axis square to it (a shaft end or a bolt head on a plate). |
| **Insert** | round edge + round edge | Same axis and the rim centres together: a pin or bolt into a hole. **Offset** moves it along the axis. |
| **Align axes** | axis or rim + axis or rim | Both centre lines on one line. The part can still slide along it and turn about it. |
| **Coincident** | point + point | The two points meet. |
| **Point on face / Point on axis** | point + face or axis | The point lies on the face's plane, or on the line. |
| **Parallel / Faces at 90°** | two directions | Only turns the part. Nothing slides. |

Use **Flip** (or **Level** for two faces) when the part ends up facing the wrong way.

## Nothing jumps around

- **The smallest move.** A mate turns the part only as far as needed, about the feature you clicked, then slides it only in the directions the mate fixes. A part that is nearly in place barely moves.
- **Only the moving part moves.** If you clicked a part inside an imported assembly, its sub-assembly moves with it. If both parts are in the same assembly, only the branch you clicked moves.
- **Earlier mates keep holding.** Every new mate is solved together with the existing ones. Flush and then Align axes leaves the pin both seated and centred.
- **Faces go against each other.** Faces and rims point out of their part, so by default they meet face to face and parts don't sink into each other.
- **Free motion stays free.** Dragging a mated part with the gizmo, or typing its position, moves it only where its mates allow. A block flush on a plate slides along the plate but can't lift off it. Parts mated to the one you move come along.
- **Edits are reversible.** Changing the type, Flip or Offset starts again from where the parts were before the mate. **Remove** puts the part back, and **Undo** (Ctrl+Z) works as usual.

## The mates list

The **Mates** list in the panel shows every mate. Click one to edit it; its two features are drawn in the viewport (orange on the moving part, white or black on the fixed one). Untick a mate to switch it off without deleting it. **Solve** moves parts until every mate holds again. **Delete** removes the selected mate and leaves the parts where they are.

A ⚠ next to a mate means it can't hold together with the others, for example a part made flush with two faces at different heights. Flip it, change its type, or delete one of the mates.

## Good to know

- Mates are for building the machine. They are not enforced while the simulation runs, when **joints** move parts. Assemble the parts first, then add joints ([Joints & Actuators](help:actuators)).
- A part can't be mated to something inside its own hierarchy (a parent to its child). Give it another parent first: select both parts in the Hierarchy and right-click.
- **Duplicate** (Ctrl+D) copies the mates between parts inside the copy. Deleting a part deletes its mates.
- Components from the library bring their mates along ([Import & Export](help:exchange)).
