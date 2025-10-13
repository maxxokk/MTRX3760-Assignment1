#pragma once
namespace cfg {
  // Distances (meters)
  inline constexpr double DESIRED_WALL = 0.45;
  inline constexpr double FRONT_STOP   = 0.50;
  inline constexpr double LOST_WALL    = 0.80;

  // Speeds
  inline constexpr double LIN_SPEED     = 0.15;
  inline constexpr double MAX_ANG_SPEED = 0.60;

  // Control gain
  inline constexpr double KP = 1.20;

  // Sector centers (radians, relative to robot forward = 0)
  // We'll match the right-hand wall: right = -90°, front-right = -30°, front = 0°
  inline constexpr double FRONT_ANGLE       = 0.0;                //   0°
  inline constexpr double FRONT_RIGHT_ANGLE = -0.5235987756;      //  -30°
  inline constexpr double RIGHT_ANGLE       = -1.57079632679;     //  -90°

  inline constexpr double WINDOW = 0.26; // +/- ~15°
}
