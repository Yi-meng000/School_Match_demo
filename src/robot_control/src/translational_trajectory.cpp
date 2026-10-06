#include "robot_control/translational_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace robot_control
{
namespace trajectory
{
namespace
{

constexpr double kEps = 1e-9;
constexpr double kPi = 3.14159265358979323846;

bool finite(double value)
{
  return std::isfinite(value);
}

bool finitePoint(const Point2& point)
{
  return finite(point.x) && finite(point.y);
}

double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(value, hi));
}

Vector2 subtract(const Point2& a, const Point2& b)
{
  return {a.x - b.x, a.y - b.y};
}

Vector2 scale(const Vector2& vector, double scalar)
{
  return {vector.x * scalar, vector.y * scalar};
}

Vector2 add(const Vector2& a, const Vector2& b)
{
  return {a.x + b.x, a.y + b.y};
}

double dot(const Vector2& a, const Vector2& b)
{
  return a.x * b.x + a.y * b.y;
}

double cross(const Vector2& a, const Vector2& b)
{
  return a.x * b.y - a.y * b.x;
}

double norm(const Vector2& vector)
{
  return std::hypot(vector.x, vector.y);
}

Vector2 normalized(const Vector2& vector)
{
  const double length = norm(vector);
  if (length <= kEps) {
    return {1.0, 0.0};
  }
  return {vector.x / length, vector.y / length};
}

Point2 lerp(const Point2& a, const Point2& b, double ratio)
{
  return {
    a.x + (b.x - a.x) * ratio,
    a.y + (b.y - a.y) * ratio};
}

double distanceSquaredToSegment(
  const Point2& point, const Point2& a, const Point2& b,
  double* projection = nullptr)
{
  const Vector2 ab = subtract(b, a);
  const Vector2 ap = subtract(point, a);
  const double length_squared = dot(ab, ab);
  double ratio = 0.0;
  if (length_squared > kEps) {
    ratio = clamp(dot(ap, ab) / length_squared, 0.0, 1.0);
  }
  if (projection != nullptr) {
    *projection = ratio;
  }
  const Point2 closest = lerp(a, b, ratio);
  const double dx = point.x - closest.x;
  const double dy = point.y - closest.y;
  return dx * dx + dy * dy;
}

void buildGeometryTable(
  const std::vector<Point2>& points, std::vector<double>* arc_lengths,
  std::vector<Vector2>* tangents, std::vector<double>* curvatures,
  double* total_length)
{
  arc_lengths->assign(points.size(), 0.0);
  tangents->assign(points.size(), {1.0, 0.0});
  curvatures->assign(points.size(), 0.0);

  for (std::size_t i = 1; i < points.size(); ++i) {
    (*arc_lengths)[i] =
      (*arc_lengths)[i - 1] + norm(subtract(points[i], points[i - 1]));
  }
  *total_length = arc_lengths->back();

  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t before = i == 0 ? 0 : i - 1;
    const std::size_t after = std::min(points.size() - 1, i + 1);
    (*tangents)[i] = normalized(subtract(points[after], points[before]));
  }

  if (points.size() < 3) {
    return;
  }
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Vector2 a = subtract(points[i], points[i - 1]);
    const Vector2 b = subtract(points[i + 1], points[i]);
    const Vector2 chord = subtract(points[i + 1], points[i - 1]);
    const double denominator = norm(a) * norm(b) * norm(chord);
    if (denominator > kEps) {
      (*curvatures)[i] = 2.0 * cross(a, b) / denominator;
    }
  }
  curvatures->front() = (*curvatures)[1];
  curvatures->back() = (*curvatures)[curvatures->size() - 2];
}

double minimumSinusoidalDistance(
  double start_speed, double end_speed, double acceleration)
{
  if (acceleration <= kEps) {
    return std::numeric_limits<double>::infinity();
  }
  return kPi * std::fabs(
    end_speed * end_speed - start_speed * start_speed) / (4.0 * acceleration);
}

double rampDuration(double distance, double start_speed, double end_speed)
{
  const double speed_sum = start_speed + end_speed;
  if (distance <= kEps || speed_sum <= kEps) {
    return 0.0;
  }
  return 2.0 * distance / speed_sum;
}

struct RampValue
{
  double distance{0.0};
  double speed{0.0};
  double acceleration{0.0};
};

RampValue sampleRamp(
  double duration, double start_speed, double end_speed, double time)
{
  if (duration <= kEps) {
    return {0.0, end_speed, 0.0};
  }
  const double clamped_time = clamp(time, 0.0, duration);
  const double delta = end_speed - start_speed;
  const double angle = kPi * clamped_time / duration;
  RampValue result;
  result.distance = 0.5 * (start_speed + end_speed) * clamped_time -
    delta * duration * std::sin(angle) / (2.0 * kPi);
  result.speed = start_speed + 0.5 * delta * (1.0 - std::cos(angle));
  result.acceleration = 0.5 * delta * kPi * std::sin(angle) / duration;
  return result;
}

double inverseRampTime(
  double duration, double start_speed, double end_speed, double distance)
{
  if (duration <= kEps) {
    return 0.0;
  }
  const double total_distance = 0.5 * (start_speed + end_speed) * duration;
  const double target = clamp(distance, 0.0, total_distance);
  double lo = 0.0;
  double hi = duration;
  double time = total_distance > kEps ? duration * target / total_distance : 0.0;

  for (int iteration = 0; iteration < 60; ++iteration) {
    const RampValue value = sampleRamp(duration, start_speed, end_speed, time);
    const double residual = value.distance - target;
    if (std::fabs(residual) <= 1e-12 * std::max(1.0, total_distance)) {
      break;
    }
    if (residual > 0.0) {
      hi = time;
    } else {
      lo = time;
    }
    double next = value.speed > kEps ? time - residual / value.speed : 0.5 * (lo + hi);
    if (!(next > lo && next < hi)) {
      next = 0.5 * (lo + hi);
    }
    time = next;
  }
  return clamp(time, 0.0, duration);
}

}  // namespace

bool SpeedProfileOptions::isValid(std::string* error) const
{
  if (!finite(cruise_speed) || !finite(nominal_accel) || !finite(nominal_decel) ||
      !finite(accel_fraction) || !finite(decel_fraction) ||
      cruise_speed <= 0.0 || nominal_accel <= 0.0 || nominal_decel <= 0.0 ||
      accel_fraction < 0.0 || decel_fraction < 0.0)
  {
    if (error != nullptr) {
      *error = "SpeedProfileOptions requires finite positive speeds/accelerations and non-negative fractions";
    }
    return false;
  }
  return true;
}

bool GeneratorOptions::isValid(std::string* error) const
{
  if (!finite(max_activation_offset) || !finite(local_projection_backtrack) ||
      !finite(local_projection_lookahead) || max_activation_offset < 0.0 ||
      local_projection_backtrack < 0.0 || local_projection_lookahead < 0.0)
  {
    if (error != nullptr) {
      *error = "GeneratorOptions requires finite non-negative distances";
    }
    return false;
  }
  return true;
}

bool PathGeometry::build(const std::vector<Point2>& points, std::string* error)
{
  valid_ = false;
  total_length_ = 0.0;
  arc_lengths_.clear();
  points_.clear();
  tangents_.clear();
  curvatures_.clear();

  points_.reserve(points.size());
  for (const Point2& point : points) {
    if (!finitePoint(point)) {
      points_.clear();
      if (error != nullptr) {
        *error = "Path contains NaN or infinity";
      }
      return false;
    }
    if (points_.empty() || norm(subtract(point, points_.back())) > kEps) {
      points_.push_back(point);
    }
  }

  if (points_.size() < 2) {
    points_.clear();
    if (error != nullptr) {
      *error = "Path has fewer than two distinct consecutive points";
    }
    return false;
  }

  buildGeometryTable(
    points_, &arc_lengths_, &tangents_, &curvatures_, &total_length_);
  if (total_length_ <= kEps) {
    points_.clear();
    arc_lengths_.clear();
    tangents_.clear();
    curvatures_.clear();
    if (error != nullptr) {
      *error = "Path length is zero";
    }
    return false;
  }

  valid_ = true;
  return true;
}

PathSample PathGeometry::sample(double arc_length) const
{
  PathSample result;
  if (!valid_ || points_.size() < 2) {
    return result;
  }

  const double s = clamp(arc_length, 0.0, total_length_);
  const auto upper = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), s);
  std::size_t index = 0;
  if (upper == arc_lengths_.end()) {
    index = arc_lengths_.size() - 2;
  } else if (upper != arc_lengths_.begin()) {
    index = static_cast<std::size_t>(upper - arc_lengths_.begin() - 1);
  }

  const double span = arc_lengths_[index + 1] - arc_lengths_[index];
  const double ratio = span > kEps ? (s - arc_lengths_[index]) / span : 0.0;
  result.position = lerp(points_[index], points_[index + 1], ratio);
  result.tangent = normalized(add(
    scale(tangents_[index], 1.0 - ratio),
    scale(tangents_[index + 1], ratio)));
  result.arc_length = s;
  result.curvature = curvatures_[index] +
    (curvatures_[index + 1] - curvatures_[index]) * ratio;
  return result;
}

Projection PathGeometry::projectRange(
  const Point2& position, double s_lo, double s_hi) const
{
  Projection result;
  result.distance = std::numeric_limits<double>::infinity();
  if (!valid_ || points_.size() < 2) {
    return result;
  }

  s_lo = clamp(s_lo, 0.0, total_length_);
  s_hi = clamp(s_hi, 0.0, total_length_);
  if (s_lo > s_hi) {
    std::swap(s_lo, s_hi);
  }

  const auto first_upper = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), s_lo);
  const auto last_upper = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), s_hi);
  std::size_t first = first_upper == arc_lengths_.begin() ? 0 :
    static_cast<std::size_t>(first_upper - arc_lengths_.begin() - 1);
  std::size_t last = std::min(
    points_.size() - 2,
    static_cast<std::size_t>(last_upper - arc_lengths_.begin()));
  if (last < first) {
    last = first;
  }

  double best_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = first; i <= last; ++i) {
    double ratio = 0.0;
    distanceSquaredToSegment(position, points_[i], points_[i + 1], &ratio);
    const double segment_length = arc_lengths_[i + 1] - arc_lengths_[i];
    const double candidate_s = clamp(
      arc_lengths_[i] + ratio * segment_length, s_lo, s_hi);
    const Point2 candidate = sample(candidate_s).position;
    const double dx = position.x - candidate.x;
    const double dy = position.y - candidate.y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < best_squared) {
      best_squared = distance_squared;
      result.arc_length = candidate_s;
    }
  }
  result.distance = std::sqrt(best_squared);
  return result;
}

Projection PathGeometry::project(
  const Point2& position, double arc_length_hint,
  double backtrack, double lookahead) const
{
  if (!valid_) {
    return {};
  }
  if (arc_length_hint < 0.0) {
    return projectRange(position, 0.0, total_length_);
  }
  return projectRange(
    position,
    arc_length_hint - std::max(0.0, backtrack),
    arc_length_hint + std::max(0.0, lookahead));
}

TrajectoryGenerator::TrajectoryGenerator(
  const SpeedProfileOptions& speed_options,
  const GeneratorOptions& generator_options)
: speed_options_(speed_options), generator_options_(generator_options)
{
}

bool TrajectoryGenerator::setSpeedProfileOptions(
  const SpeedProfileOptions& options, std::string* error)
{
  if (!options.isValid(error)) {
    return false;
  }
  speed_options_ = options;
  clearPath();
  return true;
}

bool TrajectoryGenerator::buildProfile(
  double start_arc_length, double start_speed)
{
  profile_ = {};
  profile_.start_arc_length = start_arc_length;
  profile_.length = std::max(0.0, path_.length() - start_arc_length);
  profile_.start_speed = std::max(0.0, start_speed);

  diagnostics_.required_peak_deceleration = 0.0;
  diagnostics_.nominal_decel_exceeded = false;

  if (profile_.length <= kEps) {
    profile_.start_speed = 0.0;
    profile_.peak_speed = 0.0;
    profile_.valid = true;
    return true;
  }

  const double minimum_stop_distance = minimumSinusoidalDistance(
    profile_.start_speed, 0.0, speed_options_.nominal_decel);

  // Above-cruise activation and insufficient stopping distance both use one
  // deterministic sine ramp over all remaining path.  This preserves a zero
  // terminal speed without introducing an emergency state or changing v_des(s)
  // on later control ticks.
  if (profile_.start_speed > speed_options_.cruise_speed + kEps ||
      minimum_stop_distance > profile_.length + kEps)
  {
    profile_.direct_deceleration = true;
    profile_.peak_speed = profile_.start_speed;
    profile_.decel_distance = profile_.length;
    profile_.decel_duration = rampDuration(
      profile_.decel_distance, profile_.start_speed, 0.0);
    diagnostics_.required_peak_deceleration =
      profile_.start_speed > kEps ?
      kPi * profile_.start_speed * profile_.start_speed /
      (4.0 * profile_.decel_distance) : 0.0;
    diagnostics_.nominal_decel_exceeded =
      diagnostics_.required_peak_deceleration >
      speed_options_.nominal_decel + 1e-9;
    profile_.valid = true;
    return true;
  }

  const double accel_to_cruise = minimumSinusoidalDistance(
    profile_.start_speed, speed_options_.cruise_speed,
    speed_options_.nominal_accel);
  const double decel_from_cruise = minimumSinusoidalDistance(
    speed_options_.cruise_speed, 0.0, speed_options_.nominal_decel);

  if (accel_to_cruise + decel_from_cruise <= profile_.length + kEps) {
    profile_.peak_speed = speed_options_.cruise_speed;
    profile_.accel_distance = std::max(
      speed_options_.accel_fraction * profile_.length, accel_to_cruise);
    profile_.decel_distance = std::max(
      speed_options_.decel_fraction * profile_.length, decel_from_cruise);

    if (profile_.accel_distance + profile_.decel_distance > profile_.length) {
      const double excess =
        profile_.accel_distance + profile_.decel_distance - profile_.length;
      const double accel_slack = profile_.accel_distance - accel_to_cruise;
      const double decel_slack = profile_.decel_distance - decel_from_cruise;
      const double slack = accel_slack + decel_slack;
      if (slack > kEps) {
        profile_.accel_distance -= excess * accel_slack / slack;
        profile_.decel_distance -= excess * decel_slack / slack;
      }
    }
    profile_.cruise_distance = std::max(
      0.0, profile_.length - profile_.accel_distance - profile_.decel_distance);
  } else {
    // No cruise segment.  The peak is the unique speed whose nominal sine
    // acceleration and deceleration distances exactly fill the path.
    const double accel_coefficient = kPi / (4.0 * speed_options_.nominal_accel);
    const double decel_coefficient = kPi / (4.0 * speed_options_.nominal_decel);
    const double peak_squared = std::max(
      profile_.start_speed * profile_.start_speed,
      (profile_.length +
      accel_coefficient * profile_.start_speed * profile_.start_speed) /
      (accel_coefficient + decel_coefficient));
    profile_.peak_speed = std::sqrt(peak_squared);
    profile_.accel_distance = accel_coefficient *
      (peak_squared - profile_.start_speed * profile_.start_speed);
    profile_.decel_distance = std::max(
      0.0, profile_.length - profile_.accel_distance);
  }

  profile_.accel_duration = rampDuration(
    profile_.accel_distance, profile_.start_speed, profile_.peak_speed);
  profile_.cruise_duration = profile_.peak_speed > kEps ?
    profile_.cruise_distance / profile_.peak_speed : 0.0;
  profile_.decel_duration = rampDuration(
    profile_.decel_distance, profile_.peak_speed, 0.0);
  diagnostics_.required_peak_deceleration =
    profile_.decel_distance > kEps ?
    kPi * profile_.peak_speed * profile_.peak_speed /
    (4.0 * profile_.decel_distance) : 0.0;
  diagnostics_.nominal_decel_exceeded =
    diagnostics_.required_peak_deceleration >
    speed_options_.nominal_decel + 1e-9;
  profile_.valid = true;
  return true;
}

bool TrajectoryGenerator::activatePath(
  const PathGeometry& geometry, const MotionState2D& current_state,
  std::string* error)
{
  active_ = false;
  profile_ = {};
  diagnostics_ = {};
  if (!speed_options_.isValid(error) || !generator_options_.isValid(error)) {
    diagnostics_.status = TrajectoryStatus::PathRejected;
    return false;
  }
  if (!geometry.valid() || !finitePoint(current_state.position) ||
      !finite(current_state.velocity.x) || !finite(current_state.velocity.y))
  {
    if (error != nullptr) {
      *error = "Cannot activate an invalid path or non-finite motion state";
    }
    diagnostics_.status = TrajectoryStatus::PathRejected;
    return false;
  }

  const Projection projection = geometry.project(current_state.position);
  if (projection.distance > generator_options_.max_activation_offset) {
    if (error != nullptr) {
      *error = "New path is farther than max_activation_offset from the vehicle";
    }
    diagnostics_.status = TrajectoryStatus::PathRejected;
    return false;
  }

  path_ = geometry;
  progress_ = projection.arc_length;
  const PathSample start = path_.sample(progress_);
  const double start_speed = std::max(0.0, dot(current_state.velocity, start.tangent));
  if (!buildProfile(progress_, start_speed)) {
    diagnostics_.status = TrajectoryStatus::PathRejected;
    if (error != nullptr) {
      *error = "Cannot construct the fixed sinusoidal speed profile";
    }
    return false;
  }

  active_ = true;
  diagnostics_.status = TrajectoryStatus::Ready;
  diagnostics_.progress = progress_;
  diagnostics_.remaining_length = path_.length() - progress_;
  diagnostics_.speed_at_progress = desiredSpeed(progress_);
  return true;
}

void TrajectoryGenerator::clearPath()
{
  active_ = false;
  progress_ = 0.0;
  profile_ = {};
  diagnostics_ = {};
}

double TrajectoryGenerator::profileDuration() const
{
  if (!profile_.valid) {
    return 0.0;
  }
  return profile_.direct_deceleration ? profile_.decel_duration :
    profile_.accel_duration + profile_.cruise_duration + profile_.decel_duration;
}

double TrajectoryGenerator::profileTimeAtArcLength(double arc_length) const
{
  if (!profile_.valid || profile_.length <= kEps) {
    return 0.0;
  }
  const double relative_s = clamp(
    arc_length - profile_.start_arc_length, 0.0, profile_.length);

  if (profile_.direct_deceleration) {
    return inverseRampTime(
      profile_.decel_duration, profile_.start_speed, 0.0, relative_s);
  }
  if (relative_s <= profile_.accel_distance) {
    return inverseRampTime(
      profile_.accel_duration, profile_.start_speed,
      profile_.peak_speed, relative_s);
  }
  if (relative_s <= profile_.accel_distance + profile_.cruise_distance) {
    return profile_.accel_duration +
      (relative_s - profile_.accel_distance) / profile_.peak_speed;
  }
  return profile_.accel_duration + profile_.cruise_duration +
    inverseRampTime(
      profile_.decel_duration, profile_.peak_speed, 0.0,
      relative_s - profile_.accel_distance - profile_.cruise_distance);
}

ReferencePoint TrajectoryGenerator::sampleProfileTime(double profile_time) const
{
  ReferencePoint result;
  if (!active_ || !profile_.valid) {
    return result;
  }

  const double time = clamp(profile_time, 0.0, profileDuration());
  double relative_s = 0.0;
  double speed = 0.0;
  double tangential_acceleration = 0.0;

  if (profile_.length <= kEps || time >= profileDuration() - kEps) {
    relative_s = profile_.length;
  } else if (profile_.direct_deceleration) {
    const RampValue value = sampleRamp(
      profile_.decel_duration, profile_.start_speed, 0.0, time);
    relative_s = value.distance;
    speed = value.speed;
    tangential_acceleration = value.acceleration;
  } else if (time <= profile_.accel_duration) {
    const RampValue value = sampleRamp(
      profile_.accel_duration, profile_.start_speed,
      profile_.peak_speed, time);
    relative_s = value.distance;
    speed = value.speed;
    tangential_acceleration = value.acceleration;
  } else if (time <= profile_.accel_duration + profile_.cruise_duration) {
    relative_s = profile_.accel_distance +
      profile_.peak_speed * (time - profile_.accel_duration);
    speed = profile_.peak_speed;
  } else {
    const RampValue value = sampleRamp(
      profile_.decel_duration, profile_.peak_speed, 0.0,
      time - profile_.accel_duration - profile_.cruise_duration);
    relative_s = profile_.accel_distance + profile_.cruise_distance + value.distance;
    speed = value.speed;
    tangential_acceleration = value.acceleration;
  }

  const double arc_length = clamp(
    profile_.start_arc_length + relative_s,
    profile_.start_arc_length, path_.length());
  const PathSample path_sample = path_.sample(arc_length);
  const Vector2 normal{-path_sample.tangent.y, path_sample.tangent.x};
  result.arc_length = arc_length;
  result.position = path_sample.position;
  result.speed = std::max(0.0, speed);
  result.tangential_acceleration = tangential_acceleration;
  result.curvature = path_sample.curvature;
  result.velocity = scale(path_sample.tangent, result.speed);
  result.acceleration = add(
    scale(path_sample.tangent, tangential_acceleration),
    scale(normal, path_sample.curvature * result.speed * result.speed));
  return result;
}

double TrajectoryGenerator::desiredSpeed(double arc_length) const
{
  if (!active_ || !profile_.valid) {
    return 0.0;
  }
  return sampleProfileTime(profileTimeAtArcLength(arc_length)).speed;
}

TrajectoryStatus TrajectoryGenerator::makeHorizon(
  const MotionState2D& current_state, double dt, std::size_t steps,
  std::vector<ReferencePoint>* out,
  const HeadingProvider& heading_provider)
{
  if (out == nullptr) {
    return TrajectoryStatus::NoActivePath;
  }
  out->clear();
  if (!active_ || !path_.valid() || !profile_.valid ||
      !finite(dt) || dt <= 0.0 || steps == 0 ||
      !finitePoint(current_state.position) ||
      !finite(current_state.velocity.x) || !finite(current_state.velocity.y))
  {
    return TrajectoryStatus::NoActivePath;
  }

  const Projection projection = path_.project(
    current_state.position, progress_,
    generator_options_.local_projection_backtrack,
    generator_options_.local_projection_lookahead);
  progress_ = std::max(progress_, projection.arc_length);
  diagnostics_.status = TrajectoryStatus::Ready;
  diagnostics_.progress = progress_;
  diagnostics_.remaining_length = std::max(0.0, path_.length() - progress_);
  diagnostics_.speed_at_progress = desiredSpeed(progress_);

  const double current_profile_time = profileTimeAtArcLength(progress_);
  out->reserve(steps);
  for (std::size_t i = 0; i < steps; ++i) {
    const double time_from_now = dt * static_cast<double>(i + 1);
    ReferencePoint point = sampleProfileTime(current_profile_time + time_from_now);
    point.time_from_now = time_from_now;
    if (heading_provider) {
      point.heading = heading_provider(time_from_now, point.arc_length);
    }
    out->push_back(point);
  }
  return TrajectoryStatus::Ready;
}

}  // namespace trajectory
}  // namespace robot_control
