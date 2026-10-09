# Motor Selection

## Confirmed Decision

On 2026-09-12 the user initially selected Steadywin GIM6010-8 for six joints,
then superseded the TWO wheel motors with **CJC BL4820M12**. The current
selection is four GIM6010-8 leg motors and two BL4820M12 wheel motors:

| Joint | Function | Motor |
| --- | --- | --- |
| `hip_left` | Left hip | GIM6010-8 |
| `knee_left` | Left knee | GIM6010-8 |
| `wheel_left` | Left wheel | BL4820M12 |
| `hip_right` | Right hip | GIM6010-8 |
| `knee_right` | Right knee | GIM6010-8 |
| `wheel_right` | Right wheel | BL4820M12 |

## Wheel Motor: CJC BL4820M12

User-supplied manufacturer source: https://www.cjcmotor.com/motor/bl4820m12

Retrieved 2026-09-12; unmodified HTML snapshot:
`source_assets/datasheets/cjc_bl4820m12_20260912.html`.
SHA256: `4334f5f2ecd5c97f759c863e799dc7d407430dca5b45b0a9c7ec35a734234484`.
The datasheet snapshot above is the only copy kept in the repository. The
MuJoCo actuator assumptions are configured separately in
`src/tap_sim/model/gim_dynamic.xml` and documented in
`docs/GIM_SIM_PHYSICS_SPEC.md`.

| Published property | Value |
| --- | --- |
| Rated voltage | 12 V |
| Rated current | 1 A |
| Rated power | 14 W |
| Rated speed | 400 rpm = 41.887902 rad/s |
| No-load current | 0.15 A |
| No-load speed | 1000, row omits unit |
| Continuous torque | 130 mN.m = 0.130 Nm |
| Peak torque | 0.150 Nm (user clarification) |
| Outer diameter / length | 48 / 20, rows omit units |
| Integrated controller | Yes |
| Communication | UART in page title; detailed protocol not supplied |

The page says parameters are reference values and customization is available.
The user clarified that 0.130 Nm is the continuous torque and 0.150 Nm is the
peak torque; the simulator therefore keeps its 0.150 Nm peak command limit.
Do not treat no-load speed as a loaded speed limit. With the current 0.040 m
wheel and assumed
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
Visual inspection shows the red speed curve near
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

The simulation maps the knee CAD pose to 22..135 degrees of internal angle
(`q_knee = 0..113 degrees`). Physical encoder zero, sign and stop calibration
remain unverified; do not transfer that simulator mapping to hardware without
calibration. Wheel joints remain continuous.

The active GIM simulation uses the provisional mass allocation in
`src/tap_sim/model/gim_mass_properties.yaml`. Treat it as an estimate until
solid mass properties or hardware measurements replace it.

The current provisional model groups each wheel motor stator with the shin and
only the wheel with its rotating link. Its COM and inertia remain estimates;
replace them when verified motor mounting geometry or solid mass properties
become available. CAD fit and wheel/leg interference still need verification.

The current GIM torque, friction and wheel-response assumptions are recorded in
`docs/GIM_SIM_PHYSICS_SPEC.md`; they are simulation parameters, not verified
continuous motor limits. Previous balance/deep-squat results for the legacy
model do not validate the GIM simulation.

After variant verification, update mass allocation and actuator modeling,
regenerate COM/pose data, then repeat balance and drive tests in
simulation. No hardware actuation or firmware deployment is authorized by this note.
