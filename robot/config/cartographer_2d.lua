-- Configuración de Google Cartographer para robot diferencial con lidar 2D (sin IMU)
-- Neato XV-11 / topic: /scan / frame base: base_footprint

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frames de TF
  map_frame = "map",
  tracking_frame = "base_link",   -- sin IMU, se usa el frame base
  published_frame = "base_link",
  odom_frame = "odom",

  provide_odom_frame = false, --(ros2_control)
  publish_frame_projected_to_2d = true,
  use_odometry = false,                 -- usa /odom topic
  use_nav_sat = false,
  use_landmarks = false,

  -- Topics de sensores
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- Tiempos
  lookup_transform_timeout_sec = 0.5, --0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 1e-2, --5e-3,
  trajectory_publish_period_sec = 50e-3, --30e-3,

  -- Factor de muestreo (1.0 = usa todos los rangos del lidar)
  rangefinder_sampling_ratio = 1.0, --0.5,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- ── Configuración 2D ──────────────────────────────────────────────────────────
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false      -- sin IMU

-- Rango del lidar Neato XV-11: 0.02 m – 5.0 m típico
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 5.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- rayo "libre" cuando no hay retorno

-- Resolución del submapa (metros por píxel)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Scan matcher local (Ceres)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)

-- Pose extrapolator — usa odometría cuando no hay IMU
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- Optimización global (loop closure)
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options