#ifndef TRAFFIC_TRACKER_H
#define TRAFFIC_TRACKER_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

struct MapData;
struct PlanningSnapshot;

struct TrafficTrackerConfig {
  double simulator_time_step_s = 0.02;
  double stale_after_s = 0.30;
  double drop_after_s = 1.00;
  double minimum_safety_track_age_s = 0.06;
  double maximum_abs_road_s_rate_mps = 80.0;
  double maximum_abs_d_rate_mps = 5.0;
  double maximum_abs_longitudinal_acceleration_mps2 = 8.0;
  double id_reuse_longitudinal_tolerance_m = 8.0;
  double id_reuse_lateral_tolerance_m = 2.0;
  double obstacle_length_m = 4.8;
  double obstacle_width_m = 2.0;
  double initial_longitudinal_uncertainty_m = 0.50;
  double stale_longitudinal_uncertainty_growth_mps = 1.00;
  std::size_t maximum_tracks = 128;
};

enum class TrafficTrackerResetReason {
  kNone,
  kInitialization,
  kControlHistoryReset,
  kTimeReversed
};

struct TrackedVehicleState {
  int id = 0;
  double road_s_wrapped_m = 0.0;
  double road_s_unwrapped_m = 0.0;
  double relative_road_s_m = 0.0;
  double d_m = 0.0;
  double road_s_rate_mps = 0.0;
  double d_rate_mps = 0.0;
  double longitudinal_acceleration_mps2 = 0.0;
  double length_m = 0.0;
  double width_m = 0.0;
  double track_age_s = 0.0;
  double time_since_update_s = 0.0;
  double longitudinal_uncertainty_m = 0.0;
  std::uint64_t last_update_cycle = 0;
  std::uint64_t observation_count = 0;
  bool observed_this_cycle = false;
  bool motion_within_configured_bounds = true;
  bool valid = false;
  bool safety_admissible = false;

  // These fields preserve the last actual observation while the public state
  // above is propagated to the current telemetry time during short dropouts.
  double last_observed_road_s_wrapped_m = 0.0;
  double last_observed_road_s_unwrapped_m = 0.0;
  double last_observed_road_s_rate_mps = 0.0;
};

struct TrafficTrackerState {
  bool initialized = false;
  std::uint64_t cycle = 0;
  double telemetry_time_s = 0.0;
  double ego_road_s_wrapped_m = 0.0;
  double ego_road_s_unwrapped_m = 0.0;
  std::map<int, TrackedVehicleState> tracks;
};

struct TrafficTrackingSnapshot {
  std::uint64_t cycle = 0;
  TrafficTrackerResetReason reset_reason =
      TrafficTrackerResetReason::kNone;
  double ego_road_s_unwrapped_m = 0.0;
  std::vector<TrackedVehicleState> tracks;
  std::size_t observed_track_count = 0;
  std::size_t valid_track_count = 0;
  std::size_t safety_admissible_track_count = 0;
  std::size_t stale_track_count = 0;
  std::size_t reacquired_track_count = 0;
  std::size_t reused_id_count = 0;
};

struct TrafficTrackingUpdate {
  TrafficTrackerState next_state;
  TrafficTrackingSnapshot snapshot;
};

const char *TrafficTrackerResetReasonName(TrafficTrackerResetReason reason);

// Evaluate is deliberately stateless: callers decide whether next_state is
// committed. This lets ActiveBehaviorPlanner discard an unselected proposal
// without changing the history used by the next accepted cycle.
class TrafficTracker {
public:
  explicit TrafficTracker(const TrafficTrackerConfig &config);

  TrafficTrackingUpdate Evaluate(const PlanningSnapshot &snapshot,
                                 const MapData &map,
                                 const TrafficTrackerState &state) const;

private:
  TrafficTrackerConfig config_;
};

#endif // TRAFFIC_TRACKER_H
