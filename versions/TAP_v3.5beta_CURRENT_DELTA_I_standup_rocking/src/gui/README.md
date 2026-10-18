# Native TAP dashboard

The dashboard is a Python/Tk client for the native MuJoCo runner. No robot
model is currently configured in this workspace; the dashboard refuses to
start the simulation. Keep it closed until a validated model derived from the
current CAD is available.

The runner uses `EmbeddedController`, while firmware behavior is exercised by
the planar host harness in `src_fw/tests`. A viewer display does not show that
the firmware controller balances.
