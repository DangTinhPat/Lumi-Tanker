// Derived from ROBOT (GIM6010-8).STEP and the adopted mass assignments.
// Body inertial aggregates the assigned masses with the 210 g battery moved
// from the rear STEP slot to the symmetric front slot; this proposed physical
// placement must be matched on hardware before deploying this geometry.
// Mass-distribution assumptions are recorded in PHYSICS.md.
#pragma once
#include <array>
namespace robot_control::geometry {
struct Part { float mass, x, z, iyy; };
struct Origin { float x, y, z; };
// Order: body, left thigh/shin/wheel, right thigh/shin/wheel.
inline constexpr std::array<Part, 7> parts{{
    {2.40F,  0.00488392F, 0.01378601F, 0.004314721F},
    {0.05F, -0.04908F, -0.00954F, 0.0000417F},
    {0.24F,  0.08998F, -0.01749F, 0.0002200F},
    {0.02F,  0.00000F,  0.00000F, 0.0000160F},
    {0.05F, -0.04908F, -0.00954F, 0.0000417F},
    {0.24F,  0.08998F, -0.01749F, 0.0002200F},
    {0.02F,  0.00000F,  0.00000F, 0.0000160F}
}};
// Joint origins relative to the body or preceding link, CAD frame in metres.
inline constexpr std::array<Origin, 6> origins{{
    {0.00000F,  0.10250F,  0.00828F},
    {-0.09816F, 0.01050F, -0.01908F},
    {0.09816F,  0.01970F, -0.01908F},
    {0.00000F, -0.10250F,  0.00828F},
    {-0.09816F,-0.01050F, -0.01908F},
    {0.09816F, -0.01970F, -0.01908F}
}};
inline constexpr float total_mass_kg = 3.02F;
inline constexpr float wheel_radius_m = 0.040F;
inline constexpr float axle_spacing_m = 0.2654F;
inline constexpr float neutral_hip_rad = 0.0F;
inline constexpr float neutral_knee_rad = 0.0F;  // CAD internal angle is 22 deg.
inline constexpr float knee_zero_rebase_rad = 0.0F;
inline constexpr float neutral_spawn_z_m = 0.06988F;
inline constexpr float native_hip_min_rad = -3.14159265358979323846F;
inline constexpr float native_hip_max_rad =  3.14159265358979323846F;
inline constexpr float native_knee_min_rad = 0.0F;
inline constexpr float native_knee_max_rad = 1.97222222222222222222F;
inline constexpr float native_knee_travel_rad = native_knee_max_rad;
inline constexpr float leg_hip_inertia_kgm2 = 7.5e-04F;
inline constexpr float leg_knee_inertia_kgm2 = 2.5e-03F;
}  // namespace robot_control::geometry
