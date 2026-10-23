# Quick stop

What a quick stop is, how a drive carries it out, and why this package holds the joint's position
itself while a drive is quick-stopped.

## What a quick stop is

A drive that follows the CiA 402 profile moves through a small set of states, such as
"Switch On Disabled" (no power to the motor) and "Operation Enabled" (powered, following commands).
A **quick stop** is a command that brakes the motor to a standstill in a controlled way.
It is meant for situations where the motion must stop now, but cutting the power would be worse,
for example a loaded joint that would drop.

While it brakes, and afterwards, the drive reports the state **Quick Stop Active**.

## How it is meant to work (the standard)

Quick stop is part of the **CiA 402** drive profile, which is also the international standard
**IEC 61800-7-201**. **ETG.6010** describes how it is used over EtherCAT.
Two drive settings decide what a quick stop does:

- **`0x605A`, the quick stop option code.** Whether the drive brakes on a ramp or cuts the power, and
  whether it stays powered afterwards. Code **6** means: brake on the quick stop ramp, then stay in
  Quick Stop Active, powered and holding. Code **2** brakes the same way and then cuts the power.
- **`0x6085`, the quick stop deceleration.** How hard the drive brakes. Its unit follows the drive's
  velocity unit (`0x60A9`), so the same number means different things on differently configured drives.

The idea is simple: the controller sends the quick stop command, and the **drive does the rest**.

## What a SOMANET drive does

SOMANET drives support option codes 0, 2, 6 and 9. Code 9 (active short circuit) is Synapticon's own
addition. This package requires **firmware 5.6.11 or newer**. Bring-up refuses an impedance joint below it and
warns about any other joint.
What the drive does in Quick Stop Active depends on the mode:

- **Position modes** such as CSP: the drive brakes on its own ramp, at the rate `0x6085` sets, and then
  keeps its position, ignoring new position targets.
- **Velocity modes**: it brakes on its own ramp and then keeps the velocity at zero.
- **Impedance mode**: there is **no ramp**, by design, as in the drive's torque-type modes. The drive
  keeps pulling the joint toward the position target, like a spring, and waits for the velocity to reach
  zero. `0x6085` plays no part.

## The controller always sends a target

It would seem natural to let the drive "do its thing" and send it nothing after the quick stop.
Over EtherCAT that is not possible: every cycle's frame carries a target position, so there is always one.
In position modes the drive ignores it while quick-stopped, so it does not matter what it is.
In torque-type modes it matters a lot: in impedance mode it is the point the joint is pulled toward,
so a motion controller that keeps streaming its trajectory keeps the joint moving.

## What the HAL sends while a drive is quick-stopped

From the quick stop on, the HAL forwards no new targets to that drive until it is enabled again
(see [ControlLayers.md](ControlLayers.md)). What the frame carries depends on the mode:

- **CSP:** the last target the drive received before the stop. The drive brakes on its own ramp and
  then keeps its position, ignoring the target, so nothing more is needed.
- **Impedance:** a hold of the HAL's own, the target frozen where the quick stop began. Since the
  drive does not brake in this mode, the spring pulling toward that point is what stops the joint.
  This is the approach Synapticon recommends for option code 6 in torque-type modes.

The hardware parameter `quick_stop_hold` (default `true`) switches the impedance hold off, for testing
and diagnostics. Bring-up then logs a warning. Leave it on with option code 6, where the hold is what
stops the joint.

> **Older firmware.** Before 5.6.7, a SOMANET drive in CSP went back to following the position target
> once its quick stop ramp was over, while still reporting Quick Stop Active (a firmware bug: for the
> ramp the drive switches internally to a position profile, and switching back went wrong). A motion
> controller that kept streaming made the joint carry on, and a target left at the request point made
> it snap back, measured at up to 18 rad/s. The HAL used to hold the joint itself in CSP for this.
> That hold is gone: on such a drive the joint may move back to the last target it was sent before
> the stop, which is why bring-up checks the firmware.

## Two reactions: stay powered (6) or switch off (2)

Each drive's slave_config sets its reaction in the `sdo:` block, and bring-up reads it back:

```yaml
- { index: 0x605a, sub_index: 0, type: int16, value: 6 }   # or 2
```

- **6, the default:** the joint stops and the drive stays powered and holding. This is what keeps an
  unbraked, loaded joint from dropping.
- **2:** the joint stops and then the drive switches off. Synapticon suggests this for impedance mode,
  where the drive would otherwise keep running its control law. Only use
  it where a switched-off joint cannot fall, for example one with a brake or without load.
- **Any other value** counts as no quick stop: bring-up logs an error, and a `quick_stop` request
  performs a `disable` instead.

In CSV the drive ramps and then keeps the velocity at zero, ignoring the target velocity as it ignores
the position target in CSP. The hardware tests cover the stop and the hold in both modes.

## Sources

- [CAN in Automation: CiA 402 series](https://www.can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control)
- [EtherCAT Technology Group: ETG.6010 Implementation Directive for CiA402](https://www.ethercat.org/it/downloads/downloads_733ABB98E11545EA901D80D9A4CA7F80.htm)
- [Synapticon: Quick Stop (Node, software 5.6)](https://doc.synapticon.com/node/sw5.6/motion_control/advanced_control_options/quick_stop.htm)
- [Synapticon: Quick stop (Circulo, software 5.1)](https://doc.synapticon.com/circulo/sw5.1/motion_control/advanced_control_options/quick_stop.html)
- [Kollmorgen: object 605Ah, quick stop option code](https://webhelp.kollmorgen.com/kas3.03/Content/CANopen/Object%20605Ah%20Quick%20stop%20option%20code.htm)
- [Nanotec: 605Ah Quick Stop Option Code](https://en.nanotec.com/products/manual/cl3e-can-usb-en/object_dictionary%2Fod_motion_0x605A.html)
- [Synapticon: firmware update over FoE (PDF)](https://synapticon-documentation.s3.eu-west-1.amazonaws.com/Drive+Operations+through+FoE+for+SOMANET+Circulo+and+SOMANET+Node.pdf)
