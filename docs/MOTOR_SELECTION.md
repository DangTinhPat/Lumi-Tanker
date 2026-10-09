# Motor Selection

## Confirmed Decision

On 2026-09-12 the user initially selected Steadywin GIM6010-8 for six joints,
then superseded the TWO wheel motors with **CJC BL4820M12**. The current
selection is four GIM6010-8 leg motors and two BL4820M12 wheel motors:

| Joint | Function | Motor |
| --- | --- | --- |
| `cad_left_hip` | Left hip | GIM6010-8 |
| `cad_left_knee` | Left knee | GIM6010-8 |
| `cad_left_wheel` | Left wheel | BL4820M12 |
| `cad_right_hip` | Right hip | GIM6010-8 |
| `cad_right_knee` | Right knee | GIM6010-8 |
| `cad_right_wheel` | Right wheel | BL4820M12 |

## Wheel Motor: CJC BL4820M12

User-supplied manufacturer source: https://www.cjcmotor.com/motor/bl4820m12

Retrieved 2026-09-12; unmodified HTML snapshot:
`source_assets/datasheets/cjc_bl4820m12_20260912.html`.
SHA256: `4334f5f2ecd5c97f759c863e799dc7d407430dca5b45b0a9c7ec35a734234484`.
The datasheet snapshot above is the only copy kept in the repository. No
structured motor file is loaded by the controller, and nothing here enforces
actuator limits in the simulator.

| Published property | Value |
| --- | --- |
| Rated voltage | 12 V |
| Rated current | 1 A |
| Rated power | 14 W |
| Rated speed | 400 rpm = 41.887902 rad/s |
| No-load current | 0.15 A |
| No-load speed | 1000, row omits unit |
| Torque | 130 mN.m = 0.130 Nm, rating type unspecified |
| Outer diameter / length | 48 / 20, rows omit units |
| Integrated controller | Yes |
| Communication | UART in page title; detailed protocol not supplied |

The page says parameters are reference values and customization is available.
Do not treat 0.130 Nm as a verified continuous or peak allowance, or no-load
speed as a loaded speed limit. With the current 0.040 m wheel and assumed
direct drive, 400 rpm corresponds to 1.675516 m/s, not a robot speed command.
The page does not establish gearing; GIM's 8:1 ratio does NOT apply here.

The published ratings need clarification: 12 V * 1 A = 12 W, whereas rated
power is listed as 14 W; 0.130 Nm * 400 rpm = 5.445427 W mechanical power.
These cannot be assumed to describe one consistent operating point. Do not
derive efficiency, torque constant or an actuator curve from this table.
Mass, rotor inertia, stator mounting, protocol/baud rate, braking/regeneration,
bidirectional low-speed torque control and actual purchased variant are pending.

## Leg Motor: Steadywin GIM6010-8

User-supplied product reference:
https://www.alibaba.com/product-detail/Steadywin-GIM6010-8-Planetary-Reducer-Motor_1601524091608.html

The direct listing returned an anti-bot verification page during retrieval.
Official documentation was subsequently found independently, as recorded below.
The exact Alibaba item/driver revision match still requires seller confirmation.
Do not infer specifications from the model suffix alone.

## Official Manual

Manufacturer download index: https://www.steadywin.cn/h-col-124.html

GIM6010-8 manual rev2.2 (2025-05-20):
https://14180476.s21i.faiusr.com/61/ABUIABA9GAAgzpi6wQYoh9D9CQ.pdf

An unmodified copy is in
`source_assets/datasheets/steadywin_gim6010_8_rev2_2.pdf`.
SHA256: `a65b510ee38ea063251c971ac99ff42a001a7ee9f74f5c2f0c3e45afeb543521`.
Retrieved on 2026-09-12. Printed manual pages 6-7 list:

| Published property | Value |
| --- | --- |
| Rated torque | 5 Nm |
| Stall torque | 11 Nm |
| Rated speed | 120 rpm +/-10% |
| Maximum speed | 420 rpm +/-10% |
| Rated current | 10.5 A |
| Stall current | 25 A |
| Assembly mass | 388 g +/-3 g |
| Gear reduction | 8:1 |

The driver section lists rated supply 15..48 V DC and CAN up to 1 Mbit/s;
the documented default CAN rate is 500 kbit/s. CAN Simple, CANopen and MIT
protocols are documented, but shipped firmware/interface options must be matched.
Motor current ratings and the driver's separate line/phase-current limits are
not interchangeable. Do not use absolute maximum voltage as an operating target.

**11 Nm is labelled stall torque, not a validated timed peak or continuous
command allowance.** The continuous thermal envelope and transient duration
still need confirmation. Confirm shaft reference and firmware command units
before converting speed/torque or applying the 8:1 reduction again.

### Embedded Characteristic Plot

Manual page 6 includes a bitmap plot that plain text extraction omits. A
rendered page is saved as `source_assets/datasheets/steadywin_gim6010_8_rev2_2_page6.png`.
Visual inspection during jump development shows the red speed curve near
200 rpm at 5.1 Nm and approximately 285 rpm at near-zero torque. These are
approximate graphical readings, not numeric calibration data. The page does
not specify the plot's supply voltage, temperature or duty cycle. In particular,
120 rpm rated speed is not itself a hard maximum loaded speed, and the graph
does not justify a 5 Nm rectangular envelope extending to 420 rpm.

## Required Variant Data

Obtain the datasheet and protocol for the exact purchased variant:

- Supply voltage, rated/peak current, continuous output torque and transient
  peak output torque with allowed duration, cooling and temperature conditions.
- Output speed versus load and voltage, gear ratio and efficiency; distinguish
  rotor-side values from gearbox-output values and do not apply gearing twice.
- Mass including gearbox, driver and encoder; drawing, output flange, mounting
  geometry and CAD for each installed motor assembly.
- Command interface and firmware: torque/current, velocity and position modes,
  units/signs, update rate, watchdog, disable semantics and feedback definitions.
- Continuous wheel rotation support, encoder wrap/multi-turn handling and
  allowable regenerative energy on the intended power supply.
- Position of each stator/gearbox assembly in the robot, output/rotor inertia,
  and bare wheel mass separate from its old motor.

The reported knee encoder range 30..145 degrees remains a separate joint-limit
calibration issue. The CAD-zero encoder reading and sign are still unknown.
That range must not be imposed on the continuous wheel joints.

No robot mass allocation is currently attached to the native simulation.
Derive it from the adopted CAD and BOM before using model-based torque or
balance results.

Motor stator/gearbox mass must follow its supporting link rather than all being
assigned to the spinning wheel. Revised COM/inertia require mounting geometry,
not only a total mass. CAD fit and wheel/leg interference must be checked again.

Existing 3 Nm leg command caps, damping, speed limits and ideal wheel velocity
servos remain simulation assumptions. No runtime parameter, model file, inertia,
encoder limit or live desktop process was changed for this selection record.
Previous balance/deep-squat passes apply to the recorded legacy model only;
they do not validate the selected four-GIM/two-CJC actuator combination.

After variant verification, update mass allocation and actuator modeling,
regenerate COM/pose data, then repeat balance and drive tests in
simulation. No hardware actuation or firmware deployment is authorized by this note.
