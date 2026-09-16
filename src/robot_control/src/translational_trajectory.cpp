#include "robot_control/translational_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

bool finitePoint(const Point2 & point)
{
  return finite(point.x) && finite(point.y);
}

double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(value, hi));
}

Vector2 subtract(const Point2 & a, const Point2 & b)
{
  return {a.x - b.x, a.y - b.y};
}

Vector2 scale(const Vector2 & vector, double scalar)
{
  return {vector.x * scalar, vector.y * scalar};
}

Vector2 add(const Vector2 & a, const Vector2 & b)
{
  return {a.x + b.x, a.y + b.y};
}

double dot(const Vector2 & a, const Vector2 & b)
{
  return a.x * b.x + a.y * b.y;
}

double cross(const Vector2 & a, const Vector2 & b)
{
  return a.x * b.y - a.y * b.x;
}

double norm(const Vector2 & vector)
{
  return std::hypot(vector.x, vector.y);
}

Vector2 normalized(const Vector2 & vector)
{
  const double length = norm(vector);
  if (length < kEps) {
    return {1.0, 0.0};
  }
  return {vector.x / length, vector.y / length};
}

Point2 lerp(const Point2 & a, const Point2 & b, double t)
{
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

double distanceSquaredToSegment(
  const Point2 & point,
  const Point2 & a,
  const Point2 & b,
  double * projection = nullptr)
{
  const Vector2 ab = subtract(b, a);
  const Vector2 ap = subtract(point, a);
  const double length_squared = dot(ab, ab);
  double t = 0.0;
  if (length_squared > kEps) {
    t = clamp(dot(ap, ab) / length_squared, 0.0, 1.0);
  }
  if (projection != nullptr) {
    *projection = t;
  }
  const Point2 closest = lerp(a, b, t);
  const double dx = point.x - closest.x;
  const double dy = point.y - closest.y;
  return dx * dx + dy * dy;
}

double pointToPolylineDistanceSquared(
  const Point2 & point,
  const std::vector<Point2> & polyline)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    best = std::min(best, distanceSquaredToSegment(point, polyline[i], polyline[i + 1]));
  }
  return best;
}

void rdp(
  const std::vector<Point2> & input,
  std::size_t first,
  std::size_t last,
  double epsilon,
  std::vector<Point2> * output)
{
  double worst_distance = -1.0;
  std::size_t worst_index = first;
  for (std::size_t i = first + 1; i < last; ++i) {
    const double d2 = distanceSquaredToSegment(input[i], input[first], input[last]);
    if (d2 > worst_distance) {
      worst_distance = d2;
      worst_index = i;
    }
  }

  if (worst_distance > epsilon * epsilon && worst_index > first && worst_index < last) {
    rdp(input, first, worst_index, epsilon, output);
    output->pop_back();
    rdp(input, worst_index, last, epsilon, output);
    return;
  }

  output->push_back(input[first]);
  output->push_back(input[last]);
}

bool resamplePolyline(
  const std::vector<Point2> & input,
  double spacing,
  std::vector<Point2> * output)
{
  output->clear();
  if (input.size() < 2 || spacing <= kEps) {
    return false;
  }

  std::vector<double> cumulative(input.size(), 0.0);
  for (std::size_t i = 1; i < input.size(); ++i) {
    cumulative[i] = cumulative[i - 1] + norm(subtract(input[i], input[i - 1]));
  }
  const double total = cumulative.back();
  if (total <= kEps) {
    return false;
  }

  const std::size_t steps =
    std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(total / spacing)));
  output->reserve(steps + 1);
  std::size_t segment = 0;
  for (std::size_t i = 0; i <= steps; ++i) {
    const double target =
      (i == steps) ? total : total * static_cast<double>(i) / static_cast<double>(steps);
    while (segment + 1 < cumulative.size() - 1 && cumulative[segment + 1] < target) {
      ++segment;
    }
    const double begin = cumulative[segment];
    const double end = cumulative[segment + 1];
    const double ratio = (end - begin > kEps) ? (target - begin) / (end - begin) : 0.0;
    output->push_back(lerp(input[segment], input[segment + 1], clamp(ratio, 0.0, 1.0)));
  }
  output->front() = input.front();
  output->back() = input.back();
  return true;
}

void smoothPolyline(std::vector<Point2> * points, int half_window)
{
  if (points->size() < 2 || half_window <= 0) {
    return;
  }

  const std::vector<Point2> original = *points;
  const int size = static_cast<int>(original.size());
  auto sample = [&original, size](int index) {
      if (index < 0) {
        const double k = static_cast<double>(-index);
        return Point2{(k + 1.0) * original[0].x - k * original[1].x,
        (k + 1.0) * original[0].y - k * original[1].y};
      }
      if (index >= size) {
        const double k = static_cast<double>(index - (size - 1));
        return Point2{(k + 1.0) * original[size - 1].x - k * original[size - 2].x,
        (k + 1.0) * original[size - 1].y - k * original[size - 2].y};
      }
      return original[index];
    };

  for (int i = 0; i < size; ++i) {
    Point2 sum{};
    for (int offset = -half_window; offset <= half_window; ++offset) {
      const Point2 value = sample(i + offset);
      sum.x += value.x;
      sum.y += value.y;
    }
    const double divisor = static_cast<double>(2 * half_window + 1);
    (*points)[i] = {sum.x / divisor, sum.y / divisor};
  }
}

double symmetricPolylineDeviation(
  const std::vector<Point2> & original,
  const std::vector<Point2> & smoothed)
{
  double worst_squared = 0.0;
  for (const Point2 & point : original) {
    worst_squared = std::max(worst_squared, pointToPolylineDistanceSquared(point, smoothed));
  }
  for (const Point2 & point : smoothed) {
    worst_squared = std::max(worst_squared, pointToPolylineDistanceSquared(point, original));
  }
  return std::sqrt(worst_squared);
}

void buildGeometryTable(
  const std::vector<Point2> & points,
  std::vector<double> * arc_lengths,
  std::vector<Vector2> * tangents,
  std::vector<double> * curvatures,
  double * total_length)
{
  arc_lengths->assign(points.size(), 0.0);
  tangents->assign(points.size(), {1.0, 0.0});
  curvatures->assign(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    (*arc_lengths)[i] = (*arc_lengths)[i - 1] + norm(subtract(points[i], points[i - 1]));
  }
  *total_length = arc_lengths->back();

  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t before = (i == 0) ? 0 : i - 1;
    const std::size_t after = std::min(points.size() - 1, i + 1);
    (*tangents)[i] = normalized(subtract(points[after], points[before]));
  }

  if (points.size() < 3) {
    return;
  }
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Vector2 a = subtract(points[i], points[i - 1]);
    const Vector2 b = subtract(points[i + 1], points[i]);
    const Vector2 c = subtract(points[i + 1], points[i - 1]);
    const double denominator = norm(a) * norm(b) * norm(c);
    if (denominator > kEps) {
      (*curvatures)[i] = 2.0 * cross(a, b) / denominator;
    }
  }
  (*curvatures).front() = (*curvatures)[1];
  (*curvatures).back() = (*curvatures)[(*curvatures).size() - 2];
}

double minimumSinusoidalDistance(double start_speed, double end_speed, double acceleration)
{
  if (acceleration <= kEps) {
    return std::numeric_limits<double>::infinity();
  }
  return kPi * std::fabs(end_speed * end_speed - start_speed * start_speed) / (4.0 * acceleration);
}

double inverseSinusoidalPhase(
  double duration, double start_speed, double end_speed,
  double distance)
{
  if (duration <= kEps) {
    return 0.0;
  }
  const double delta = end_speed - start_speed;
  const double mean = 0.5 * (start_speed + end_speed);
  if (mean <= kEps) {
    return 0.0;
  }

  double lo = 0.0;
  double hi = duration;
  double time = clamp(distance / mean, 0.0, duration);
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double phase = kPi * time / duration;
    const double residual = mean * time - (delta * duration / (2.0 * kPi)) * std::sin(phase) -
      distance;
    if (residual > 0.0) {
      hi = time;
    } else {
      lo = time;
    }
    const double derivative = mean - 0.5 * delta * std::cos(phase);
    double next = derivative > kEps ? time - residual / derivative : 0.5 * (lo + hi);
    if (!(next > lo && next < hi)) {
      next = 0.5 * (lo + hi);
    }
    if (std::fabs(next - time) < 1e-13 * std::max(1.0, duration)) {
      return next;
    }
    time = next;
  }
  return time;
}

struct NominalSpeedShape
{
  bool valid{false};
  bool direct_ramp{false};
  double length{0.0};
  double start_speed{0.0};
  double peak_speed{0.0};
  double end_speed{0.0};
  double accel_distance{0.0};
  double cruise_distance{0.0};
  double decel_distance{0.0};
  double accel_limit{0.0};
  double decel_limit{0.0};

  bool build(double path_length, double start, const MotionLimits & limits)
  {
    length = path_length;
    start_speed = std::max(0.0, start);
    end_speed = std::max(0.0, limits.terminal_speed);
    accel_limit = limits.max_accel;
    decel_limit = limits.normal_decel;
    if (length <= kEps) {
      return false;
    }

    // If the vehicle is already above cruise, a single smooth ramp is the
    // least surprising nominal request.  Infeasible cases are later handled
    // by the hard acceleration envelope and emergency state.
    if (start_speed > limits.cruise_speed + kEps) {
      if (minimumSinusoidalDistance(start_speed, end_speed, decel_limit) > length + 1e-8) {
        return false;
      }
      direct_ramp = true;
      peak_speed = start_speed;
      decel_distance = length;
      valid = true;
      return true;
    }

    const double c_accel = kPi / (4.0 * accel_limit);
    const double c_decel = kPi / (4.0 * decel_limit);
    const double maximum_peak = std::sqrt(
      std::max(
        0.0, (length + c_accel * start_speed * start_speed + c_decel * end_speed * end_speed) /
        (c_accel + c_decel)));
    peak_speed = std::min(limits.cruise_speed, maximum_peak);
    peak_speed = std::max(peak_speed, std::max(start_speed, end_speed));

    const double accel_min = minimumSinusoidalDistance(start_speed, peak_speed, accel_limit);
    const double decel_min = minimumSinusoidalDistance(peak_speed, end_speed, decel_limit);
    if (accel_min + decel_min > length + 1e-8) {
      return false;
    }

    accel_distance = std::max(limits.accel_fraction * length, accel_min);
    decel_distance = std::max(limits.decel_fraction * length, decel_min);
    if (accel_distance + decel_distance > length) {
      const double excess = accel_distance + decel_distance - length;
      const double accel_slack = accel_distance - accel_min;
      const double decel_slack = decel_distance - decel_min;
      const double slack = accel_slack + decel_slack;
      if (slack > kEps) {
        accel_distance -= excess * accel_slack / slack;
        decel_distance -= excess * decel_slack / slack;
      }
    }
    cruise_distance = std::max(0.0, length - accel_distance - decel_distance);
    valid = true;
    return true;
  }

  double rampSpeed(double distance, double from, double to, double progress) const
  {
    if (distance <= kEps) {
      return to;
    }
    const double sum = from + to;
    if (sum <= kEps) {
      return to;
    }
    const double duration = 2.0 * distance / sum;
    const double time = inverseSinusoidalPhase(duration, from, to, clamp(progress, 0.0, distance));
    return from + 0.5 * (to - from) * (1.0 - std::cos(kPi * time / duration));
  }

  double speedAt(double progress) const
  {
    if (!valid) {
      return std::numeric_limits<double>::infinity();
    }
    const double s = clamp(progress, 0.0, length);
    if (direct_ramp) {
      return rampSpeed(length, start_speed, end_speed, s);
    }
    if (s <= accel_distance) {
      return rampSpeed(accel_distance, start_speed, peak_speed, s);
    }
    if (s <= accel_distance + cruise_distance) {
      return peak_speed;
    }
    return rampSpeed(
      decel_distance, peak_speed, end_speed,
      s - accel_distance - cruise_distance);
  }
};

std::vector<double> backwardEnvelope(
  const std::vector<double> & caps,
  const std::vector<double> & arc_lengths,
  double terminal_speed,
  double decel)
{
  std::vector<double> result(caps.size(), 0.0);
  if (caps.empty()) {
    return result;
  }
  result.back() = std::min(caps.back(), terminal_speed);
  for (std::size_t reverse = caps.size() - 1; reverse > 0; --reverse) {
    const std::size_t i = reverse - 1;
    const double ds = std::max(0.0, arc_lengths[i + 1] - arc_lengths[i]);
    const double reachable =
      std::sqrt(std::max(0.0, result[i + 1] * result[i + 1] + 2.0 * decel * ds));
    result[i] = std::min(caps[i], reachable);
  }
  return result;
}

}  // namespace

bool MotionLimits::isValid(std::string * error) const
{
  const bool finite_values = finite(cruise_speed) && finite(max_accel) && finite(normal_decel) &&
    finite(emergency_decel) && finite(max_lateral_accel) &&
    finite(terminal_speed) && finite(accel_fraction) && finite(decel_fraction);
  if (!finite_values || cruise_speed <= 0.0 || max_accel <= 0.0 || normal_decel <= 0.0 ||
    emergency_decel <= 0.0 || max_lateral_accel <= 0.0 || terminal_speed < 0.0 ||
    accel_fraction < 0.0 || decel_fraction < 0.0)
  {
    if (error != nullptr) {
      *error = "MotionLimits contains missing, non-finite, or non-positive physical limits";
    }
    return false;
  }
  if (emergency_decel + kEps < normal_decel) {
    if (error != nullptr) {
      *error = "emergency_decel must be greater than or equal to normal_decel";
    }
    return false;
  }
  if (terminal_speed > cruise_speed + kEps) {
    if (error != nullptr) {
      *error = "terminal_speed must not exceed cruise_speed";
    }
    return false;
  }
  return true;
}

bool PathGeometry::build(
  const std::vector<Point2> & raw_points,
  const PathBuildOptions & options,
  std::string * error)
{
  valid_ = false;
  total_length_ = 0.0;
  max_smooth_deviation_ = 0.0;
  arc_lengths_.clear();
  points_.clear();
  tangents_.clear();
  curvatures_.clear();

  if (options.sample_spacing <= kEps || options.rdp_epsilon < 0.0 ||
    options.smooth_half_window < 0.0 || options.max_smooth_deviation < 0.0 ||
    options.max_smoothing_attempts < 1)
  {
    if (error != nullptr) {
      *error = "PathBuildOptions contains invalid values";
    }
    return false;
  }

  std::vector<Point2> input;
  input.reserve(raw_points.size());
  for (const Point2 & point : raw_points) {
    if (!finitePoint(point)) {
      if (error != nullptr) {
        *error = "Path contains NaN or infinity";
      }
      return false;
    }
    if (input.empty() || norm(subtract(point, input.back())) > kEps) {
      input.push_back(point);
    }
  }
  if (input.size() < 2) {
    if (error != nullptr) {
      *error = "Path has fewer than two distinct points";
    }
    return false;
  }

  std::vector<Point2> simplified;
  rdp(input, 0, input.size() - 1, options.rdp_epsilon, &simplified);
  if (simplified.size() < 2) {
    if (error != nullptr) {
      *error = "RDP simplification removed the whole path";
    }
    return false;
  }

  double half_window = options.smooth_half_window;
  for (int attempt = 0; attempt < options.max_smoothing_attempts; ++attempt) {
    std::vector<Point2> dense;
    if (!resamplePolyline(simplified, options.sample_spacing, &dense)) {
      break;
    }
    const Point2 original_first = dense.front();
    const Point2 original_last = dense.back();
    const int window = static_cast<int>(std::lround(half_window / options.sample_spacing));
    smoothPolyline(&dense, window);
    // Linear extrapolation avoids shrinking a straight path, but curved path
    // endpoints must still be anchored exactly at the planner's endpoints.
    dense.front() = original_first;
    dense.back() = original_last;

    std::vector<Point2> uniform;
    if (!resamplePolyline(dense, options.sample_spacing, &uniform)) {
      break;
    }
    const double deviation = symmetricPolylineDeviation(input, uniform);
    if (deviation <= options.max_smooth_deviation + 1e-9) {
      std::vector<double> arc_lengths;
      std::vector<Vector2> tangents;
      std::vector<double> curvatures;
      double length = 0.0;
      buildGeometryTable(uniform, &arc_lengths, &tangents, &curvatures, &length);
      if (length <= kEps) {
        break;
      }
      points_ = std::move(uniform);
      arc_lengths_ = std::move(arc_lengths);
      tangents_ = std::move(tangents);
      curvatures_ = std::move(curvatures);
      total_length_ = length;
      max_smooth_deviation_ = deviation;
      valid_ = true;
      return true;
    }
    half_window *= 0.5;
  }

  if (error != nullptr) {
    *error = "Smoothed path exceeds max_smooth_deviation";
  }
  return false;
}

PathSample PathGeometry::sample(double arc_length) const
{
  PathSample result;
  if (!valid_ || points_.empty()) {
    return result;
  }
  const double s = clamp(arc_length, 0.0, total_length_);
  const auto upper = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), s);
  std::size_t index = 0;
  if (upper == arc_lengths_.begin()) {
    index = 0;
  } else if (upper == arc_lengths_.end()) {
    index = arc_lengths_.size() - 2;
  } else {
    index = static_cast<std::size_t>(upper - arc_lengths_.begin() - 1);
  }
  const double span = arc_lengths_[index + 1] - arc_lengths_[index];
  const double ratio = span > kEps ? (s - arc_lengths_[index]) / span : 0.0;
  result.position = lerp(points_[index], points_[index + 1], ratio);
  result.tangent = normalized(
    add(
      scale(tangents_[index], 1.0 - ratio),
      scale(tangents_[index + 1], ratio)));
  result.arc_length = s;
  result.curvature = curvatures_[index] + (curvatures_[index + 1] - curvatures_[index]) * ratio;
  return result;
}

Projection PathGeometry::projectRange(const Point2 & position, double s_lo, double s_hi) const
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

  for (std::size_t i = first; i <= last; ++i) {
    const double segment_length = arc_lengths_[i + 1] - arc_lengths_[i];
    double ratio = 0.0;
    const double d2 = distanceSquaredToSegment(position, points_[i], points_[i + 1], &ratio);
    double candidate_s = arc_lengths_[i] + ratio * segment_length;
    candidate_s = clamp(candidate_s, s_lo, s_hi);
    const PathSample candidate = sample(candidate_s);
    const double dx = position.x - candidate.position.x;
    const double dy = position.y - candidate.position.y;
    const double restricted_d2 = dx * dx + dy * dy;
    (void)d2;
    if (restricted_d2 < result.distance * result.distance) {
      result.arc_length = candidate_s;
      result.distance = std::sqrt(restricted_d2);
    }
  }
  return result;
}

Projection PathGeometry::project(
  const Point2 & position,
  double arc_length_hint,
  double backtrack,
  double lookahead) const
{
  if (!valid_) {
    return {};
  }
  if (arc_length_hint < 0.0) {
    return projectRange(position, 0.0, total_length_);
  }
  Projection local = projectRange(
    position, arc_length_hint - std::max(0.0, backtrack),
    arc_length_hint + std::max(0.0, lookahead));
  if (local.distance > 0.5) {
    return projectRange(position, 0.0, total_length_);
  }
  return local;
}

TrajectoryGenerator::TrajectoryGenerator(
  const MotionLimits & limits,
  const GeneratorOptions & options)
: limits_(limits), options_(options)
{
}

bool TrajectoryGenerator::setLimits(const MotionLimits & limits, std::string * error)
{
  if (!limits.isValid(error)) {
    return false;
  }
  limits_ = limits;
  return true;
}

bool TrajectoryGenerator::activatePath(
  const PathGeometry & geometry,
  const MotionState2D & current_state,
  std::string * error)
{
  if (!limits_.isValid(error)) {
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
  if (projection.distance > options_.max_activation_offset) {
    if (error != nullptr) {
      *error = "New path is farther than max_activation_offset from the vehicle";
    }
    diagnostics_.status = TrajectoryStatus::PathRejected;
    return false;
  }

  path_ = geometry;
  active_ = true;
  progress_ = projection.arc_length;
  profile_.clear();
  diagnostics_ = {};
  diagnostics_.status = TrajectoryStatus::Ready;
  diagnostics_.progress = progress_;
  diagnostics_.remaining_length = path_.length() - progress_;
  return true;
}

void TrajectoryGenerator::clearPath()
{
  active_ = false;
  progress_ = 0.0;
  profile_.clear();
  exact_nominal_ = {};
  diagnostics_ = {};
}

bool TrajectoryGenerator::rebuildProfile(const MotionState2D & current_state)
{
  profile_.clear();
  exact_nominal_ = {};
  if (!active_ || !path_.valid() || !finitePoint(current_state.position) ||
    !finite(current_state.velocity.x) || !finite(current_state.velocity.y))
  {
    diagnostics_.status = TrajectoryStatus::NoActivePath;
    return false;
  }
  if (options_.profile_spacing <= kEps || options_.minimum_speed_for_time <= 0.0) {
    diagnostics_.status = TrajectoryStatus::NoActivePath;
    return false;
  }

  const Projection projection = path_.project(
    current_state.position, progress_,
    options_.local_projection_backtrack,
    options_.local_projection_lookahead);
  progress_ = std::max(progress_, projection.arc_length);
  const double remaining = std::max(0.0, path_.length() - progress_);
  const PathSample start = path_.sample(progress_);
  const double initial_speed = std::max(0.0, dot(current_state.velocity, start.tangent));

  diagnostics_ = {};
  diagnostics_.progress = progress_;
  diagnostics_.remaining_length = remaining;
  diagnostics_.normal_stop_distance = minimumSinusoidalDistance(
    initial_speed, limits_.terminal_speed, limits_.normal_decel);
  diagnostics_.emergency_stop_distance = minimumSinusoidalDistance(
    initial_speed, limits_.terminal_speed, limits_.emergency_decel);
  diagnostics_.stop_deficit = std::max(0.0, diagnostics_.normal_stop_distance - remaining);

  if (remaining <= kEps) {
    const bool emergency_possible = initial_speed <= limits_.terminal_speed + kEps;
    diagnostics_.status =
      emergency_possible ? TrajectoryStatus::Ready : TrajectoryStatus::EmergencyInfeasible;
    diagnostics_.terminal_speed_if_unstoppable = emergency_possible ? 0.0 : initial_speed;
    profile_.push_back({0.0, path_.length(), initial_speed, 0.0});
    return true;
  }

  const std::size_t segments = std::max<std::size_t>(
    1,
    static_cast<std::size_t>(std::ceil(remaining / options_.profile_spacing)));
  std::vector<double> arc_lengths;
  arc_lengths.reserve(segments + path_.sampleArcLengths().size() + 2);
  for (std::size_t i = 0; i <= segments; ++i) {
    arc_lengths.push_back(
      progress_ + remaining * static_cast<double>(i) /
      static_cast<double>(segments));
  }
  // Curvature is linearly interpolated between geometry samples.  Include every
  // knot in the time-scaling grid so its local speed cap cannot be skipped.
  for (const double knot : path_.sampleArcLengths()) {
    if (knot > progress_ + kEps && knot < path_.length() - kEps) {
      arc_lengths.push_back(knot);
    }
  }
  std::sort(arc_lengths.begin(), arc_lengths.end());
  arc_lengths.erase(
    std::unique(
      arc_lengths.begin(), arc_lengths.end(),
      [](double a, double b) {return std::fabs(a - b) < 1e-10;}), arc_lengths.end());
  const std::size_t node_count = arc_lengths.size();
  std::vector<double> caps(node_count, limits_.cruise_speed);
  const NominalSpeedShape nominal = [&]() {
      NominalSpeedShape shape;
      shape.build(remaining, initial_speed, limits_);
      return shape;
    }();

  for (std::size_t i = 0; i < node_count; ++i) {
    const PathSample path_sample = path_.sample(arc_lengths[i]);
    const double curvature = std::fabs(path_sample.curvature);
    if (curvature > 1e-8) {
      caps[i] = std::min(caps[i], std::sqrt(limits_.max_lateral_accel / curvature));
    }
    if (nominal.valid) {
      caps[i] = std::min(caps[i], nominal.speedAt(arc_lengths[i] - progress_));
    }
    caps[i] = std::max(0.0, caps[i]);
  }
  // Cap both ends of each profile segment by that segment's maximum curvature.
  // This is deliberately conservative over one small segment, but guarantees
  // interpolated time samples cannot exceed the lateral-acceleration limit.
  for (std::size_t i = 0; i + 1 < node_count; ++i) {
    const double curvature_a = std::fabs(path_.sample(arc_lengths[i]).curvature);
    const double curvature_b = std::fabs(path_.sample(arc_lengths[i + 1]).curvature);
    const double max_curvature = std::max(curvature_a, curvature_b);
    if (max_curvature > 1e-8) {
      const double segment_cap = std::sqrt(limits_.max_lateral_accel / max_curvature);
      caps[i] = std::min(caps[i], segment_cap);
      caps[i + 1] = std::min(caps[i + 1], segment_cap);
    }
  }
  caps.back() = std::min(caps.back(), limits_.terminal_speed);

  const std::vector<double> normal_back = backwardEnvelope(
    caps, arc_lengths, limits_.terminal_speed, limits_.normal_decel);
  const bool normal_infeasible = diagnostics_.normal_stop_distance > remaining + 1e-6 ||
    initial_speed > normal_back.front() + 1e-6;
  const double decel_used = normal_infeasible ? limits_.emergency_decel : limits_.normal_decel;
  const std::vector<double> back = normal_infeasible ?
    backwardEnvelope(caps, arc_lengths, limits_.terminal_speed, limits_.emergency_decel) :
    normal_back;

  const bool emergency_infeasible = normal_infeasible &&
    (diagnostics_.emergency_stop_distance > remaining + 1e-6 ||
    initial_speed > back.front() + 1e-6);
  diagnostics_.status = emergency_infeasible ? TrajectoryStatus::EmergencyInfeasible :
    (normal_infeasible ? TrajectoryStatus::EmergencyBraking : TrajectoryStatus::Ready);
  if (normal_infeasible) {
    const double speed_gap_sq = std::max(
      0.0, initial_speed * initial_speed -
      normal_back.front() * normal_back.front());
    diagnostics_.stop_deficit = std::max(
      diagnostics_.stop_deficit,
      speed_gap_sq / (2.0 * limits_.normal_decel));
  }

  std::vector<double> speeds(node_count, 0.0);
  speeds.front() = initial_speed;
  for (std::size_t i = 1; i < node_count; ++i) {
    const double ds = arc_lengths[i] - arc_lengths[i - 1];
    const double acceleration_reachable = std::sqrt(
      std::max(
        0.0, speeds[i - 1] * speeds[i - 1] + 2.0 * limits_.max_accel * ds));
    const double braking_floor = std::sqrt(
      std::max(
        0.0, speeds[i - 1] * speeds[i - 1] - 2.0 * decel_used * ds));
    speeds[i] = std::min(back[i], acceleration_reachable);
    if (speeds[i] + 1e-6 < braking_floor) {
      // Even emergency braking cannot reach the requested curve/end speed.
      // Keep the physically reachable speed and make the unsafe condition explicit.
      speeds[i] = braking_floor;
      diagnostics_.status = TrajectoryStatus::EmergencyInfeasible;
    }
  }
  diagnostics_.terminal_speed_if_unstoppable =
    diagnostics_.status == TrajectoryStatus::EmergencyInfeasible ? speeds.back() : 0.0;

  // If no curve or acceleration envelope changed the nominal profile, retain
  // its analytic time-domain sine form instead of approximating it as piecewise
  // constant acceleration.  Constrained portions use the table below.
  if (diagnostics_.status == TrajectoryStatus::Ready && nominal.valid) {
    bool equals_nominal = true;
    for (std::size_t i = 0; i < node_count; ++i) {
      if (std::fabs(speeds[i] - nominal.speedAt(arc_lengths[i] - progress_)) > 1e-6) {
        equals_nominal = false;
        break;
      }
    }
    if (equals_nominal) {
      exact_nominal_.active = true;
      exact_nominal_.direct_ramp = nominal.direct_ramp;
      exact_nominal_.start_arc_length = progress_;
      exact_nominal_.length = nominal.length;
      exact_nominal_.start_speed = nominal.start_speed;
      exact_nominal_.peak_speed = nominal.peak_speed;
      exact_nominal_.end_speed = nominal.end_speed;
      exact_nominal_.accel_distance = nominal.accel_distance;
      exact_nominal_.cruise_distance = nominal.cruise_distance;
      exact_nominal_.decel_distance = nominal.decel_distance;
    }
  }

  profile_.resize(node_count);
  profile_[0] = {0.0, arc_lengths[0], speeds[0], 0.0};
  for (std::size_t i = 1; i < node_count; ++i) {
    const double ds = arc_lengths[i] - arc_lengths[i - 1];
    const double speed_sum = speeds[i - 1] + speeds[i];
    // A valid forward path cannot have a finite-length segment with both
    // endpoint speeds zero.  The minimum avoids a division-by-zero while
    // retaining a deterministic diagnostic trajectory for that bad input.
    const double dt = 2.0 * ds / std::max(options_.minimum_speed_for_time, speed_sum);
    const double tangential_acceleration = (speeds[i] - speeds[i - 1]) / dt;
    profile_[i - 1].tangential_acceleration = tangential_acceleration;
    profile_[i] = {profile_[i - 1].time + dt, arc_lengths[i], speeds[i], tangential_acceleration};
  }
  profile_.back().tangential_acceleration = 0.0;
  return true;
}

ReferencePoint TrajectoryGenerator::sampleProfile(
  double time_from_now,
  const HeadingProvider & heading_provider) const
{
  ReferencePoint result;
  if (profile_.empty()) {
    return result;
  }

  double s = profile_.front().arc_length;
  double speed = profile_.front().speed;
  double tangential_acceleration = profile_.front().tangential_acceleration;
  if (exact_nominal_.active) {
    const auto phase = [](double duration, double from, double to, double time,
        double * distance, double * value, double * acceleration) {
        if (duration <= kEps) {
          *distance = 0.0;
          *value = to;
          *acceleration = 0.0;
          return;
        }
        const double delta = to - from;
        const double mean = 0.5 * (from + to);
        const double clamped_time = clamp(time, 0.0, duration);
        const double angle = kPi * clamped_time / duration;
        *distance = mean * clamped_time - (delta * duration / (2.0 * kPi)) * std::sin(angle);
        *value = from + 0.5 * delta * (1.0 - std::cos(angle));
        *acceleration = 0.5 * delta * kPi / duration * std::sin(angle);
      };

    const double accel_duration = exact_nominal_.accel_distance > kEps &&
      exact_nominal_.start_speed + exact_nominal_.peak_speed > kEps ?
      2.0 * exact_nominal_.accel_distance /
      (exact_nominal_.start_speed + exact_nominal_.peak_speed) : 0.0;
    const double cruise_duration = exact_nominal_.peak_speed > kEps ?
      exact_nominal_.cruise_distance / exact_nominal_.peak_speed : 0.0;
    const double decel_duration = exact_nominal_.decel_distance > kEps &&
      exact_nominal_.peak_speed + exact_nominal_.end_speed > kEps ?
      2.0 * exact_nominal_.decel_distance /
      (exact_nominal_.peak_speed + exact_nominal_.end_speed) : 0.0;
    const double total_duration = exact_nominal_.direct_ramp ?
      2.0 * exact_nominal_.length /
      std::max(kEps, exact_nominal_.start_speed + exact_nominal_.end_speed) :
      accel_duration + cruise_duration + decel_duration;

    double relative_s = 0.0;
    if (time_from_now >= total_duration) {
      relative_s = exact_nominal_.length;
      speed = exact_nominal_.end_speed;
      tangential_acceleration = 0.0;
    } else if (exact_nominal_.direct_ramp) {
      phase(
        total_duration, exact_nominal_.start_speed, exact_nominal_.end_speed,
        time_from_now, &relative_s, &speed, &tangential_acceleration);
    } else if (time_from_now <= accel_duration) {
      phase(
        accel_duration, exact_nominal_.start_speed, exact_nominal_.peak_speed,
        time_from_now, &relative_s, &speed, &tangential_acceleration);
    } else if (time_from_now <= accel_duration + cruise_duration) {
      relative_s = exact_nominal_.accel_distance + exact_nominal_.peak_speed *
        (time_from_now - accel_duration);
      speed = exact_nominal_.peak_speed;
      tangential_acceleration = 0.0;
    } else {
      double decel_s = 0.0;
      phase(
        decel_duration, exact_nominal_.peak_speed, exact_nominal_.end_speed,
        time_from_now - accel_duration - cruise_duration, &decel_s, &speed,
        &tangential_acceleration);
      relative_s = exact_nominal_.accel_distance + exact_nominal_.cruise_distance + decel_s;
    }
    s = exact_nominal_.start_arc_length + relative_s;
  } else if (time_from_now >= profile_.back().time) {
    s = profile_.back().arc_length;
    speed = profile_.back().speed;
    tangential_acceleration = 0.0;
  } else if (time_from_now > 0.0) {
    const auto upper = std::upper_bound(
      profile_.begin(), profile_.end(), time_from_now,
      [](double time, const ProfileNode & node) {return time < node.time;});
    const std::size_t next = static_cast<std::size_t>(upper - profile_.begin());
    const std::size_t previous = next - 1;
    const ProfileNode & a = profile_[previous];
    const ProfileNode & b = profile_[next];
    const double duration = b.time - a.time;
    const double local_time = time_from_now - a.time;
    tangential_acceleration = duration > kEps ? (b.speed - a.speed) / duration : 0.0;
    speed = a.speed + tangential_acceleration * local_time;
    s = a.arc_length + a.speed * local_time +
      0.5 * tangential_acceleration * local_time * local_time;
    s = clamp(s, a.arc_length, b.arc_length);
  }

  const PathSample path_sample = path_.sample(s);
  const Vector2 normal{-path_sample.tangent.y, path_sample.tangent.x};
  result.time_from_now = std::max(0.0, time_from_now);
  result.arc_length = s;
  result.position = path_sample.position;
  result.speed = speed;
  result.tangential_acceleration = tangential_acceleration;
  result.curvature = path_sample.curvature;
  result.velocity = scale(path_sample.tangent, speed);
  result.acceleration = add(
    scale(path_sample.tangent, tangential_acceleration),
    scale(normal, path_sample.curvature * speed * speed));
  if (heading_provider) {
    result.heading = heading_provider(result.time_from_now, result.arc_length);
  }
  return result;
}

TrajectoryStatus TrajectoryGenerator::makeHorizon(
  const MotionState2D & current_state,
  double dt,
  std::size_t steps,
  std::vector<ReferencePoint> * out,
  const HeadingProvider & heading_provider)
{
  if (out != nullptr) {
    out->clear();
  }
  if (!active_ || out == nullptr || !finite(dt) || dt <= 0.0 || steps == 0) {
    return active_ ? diagnostics_.status : TrajectoryStatus::NoActivePath;
  }
  if (!rebuildProfile(current_state)) {
    return diagnostics_.status;
  }

  out->reserve(steps);
  for (std::size_t i = 0; i < steps; ++i) {
    out->push_back(sampleProfile(static_cast<double>(i + 1) * dt, heading_provider));
  }
  return diagnostics_.status;
}

}  // namespace trajectory
}  // namespace robot_control
