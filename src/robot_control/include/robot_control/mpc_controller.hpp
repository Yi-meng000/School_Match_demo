#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace robot_control
{

struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};  // world-frame measured x velocity
  double vy{0.0};  // world-frame measured y velocity
  double vw{0.0};
};

struct TrajectoryPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};  // world-frame reference velocity
  double vy{0.0};
  double vw{0.0};
  double ax{0.0};  // world-frame reference acceleration
  double ay{0.0};
  double aw{0.0};
};

struct ControlCmd
{
  double vx{0.0};  // world-frame command velocity
  double vy{0.0};
  double vw{0.0};
};

// Unconstrained world-frame trajectory-tracking MPC.
//
// State:   [x, y, yaw, measured_vx, measured_vy, measured_w]
// Input:   [command_vx, command_vy, command_w]
// Model:   first-order measured-velocity response to a velocity command
// Decision: correction around the trajectory velocity/acceleration feedforward
//
// Every predicted state x(1) ... x(N) is compared with the matching complete
// future reference r(1) ... r(N). Q and R are costs only; there are deliberately
// no position, speed, acceleration or yaw-rate hard limits in this class.
class MpcController
{
public:
  MpcController(
    int horizon, double dt,
    const Eigen::Vector3d& response_time_constants =
      Eigen::Vector3d::Constant(0.10))
  : horizon_(horizon),
    dt_(dt),
    response_time_constants_(response_time_constants)
  {
    if (horizon_ < 1 || !std::isfinite(dt_) || dt_ <= 0.0) {
      throw std::invalid_argument("MPC horizon and dt must be positive");
    }
    if (!validResponseTimeConstants(response_time_constants_)) {
      throw std::invalid_argument(
              "MPC response time constants must be finite and positive");
    }

    q_diag_ << 120.0, 120.0, 90.0, 20.0, 20.0, 2.0;
    correction_diag_ << 2.0, 2.0, 1.0;
    r_diag_ << 1.5, 1.5, 0.8;
    buildModelAndPredictionMatrices();
    buildDifferenceMatrix();
    std::string error;
    if (!rebuildCost(&error)) {
      throw std::invalid_argument(error);
    }
  }

  bool setWeights(
    const Eigen::Matrix<double, 6, 1>& q_diag,
    const Eigen::Vector3d& correction_diag,
    const Eigen::Vector3d& r_diag,
    std::string* error = nullptr)
  {
    if (!q_diag.allFinite() || !correction_diag.allFinite() ||
        !r_diag.allFinite() || (q_diag.array() < 0.0).any() ||
        (correction_diag.array() < 0.0).any() ||
        (r_diag.array() <= 0.0).any())
    {
      if (error != nullptr) {
        *error =
          "MPC weights require finite Q >= 0, correction S >= 0 and R > 0";
      }
      return false;
    }

    const auto old_q = q_diag_;
    const auto old_correction = correction_diag_;
    const auto old_r = r_diag_;
    q_diag_ = q_diag;
    correction_diag_ = correction_diag;
    r_diag_ = r_diag;
    if (!rebuildCost(error)) {
      q_diag_ = old_q;
      correction_diag_ = old_correction;
      r_diag_ = old_r;
      std::string ignored;
      rebuildCost(&ignored);
      return false;
    }
    return true;
  }

  const Eigen::Matrix<double, 6, 1>& qWeights() const { return q_diag_; }
  const Eigen::Vector3d& correctionWeights() const
  {
    return correction_diag_;
  }
  const Eigen::Vector3d& rWeights() const { return r_diag_; }
  const Eigen::Vector3d& responseTimeConstants() const
  {
    return response_time_constants_;
  }
  double lastOptimalityResidual() const { return last_optimality_residual_; }

  static double normalizeAngle(double angle)
  {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;
    while (angle > kPi) {
      angle -= kTwoPi;
    }
    while (angle < -kPi) {
      angle += kTwoPi;
    }
    return angle;
  }

  bool solveMPC(
    const State& current_state,
    const std::vector<TrajectoryPoint>& reference,
    const ControlCmd& previous_command,
    ControlCmd& command)
  {
    if (!factorization_valid_ ||
        reference.size() < static_cast<std::size_t>(horizon_) ||
        !finite(current_state) || !finite(previous_command))
    {
      return false;
    }
    for (int i = 0; i < horizon_; ++i) {
      if (!finite(reference[static_cast<std::size_t>(i)])) {
        return false;
      }
    }

    Eigen::Matrix<double, 6, 1> initial_state;
    initial_state <<
      current_state.x,
      current_state.y,
      current_state.yaw,
      current_state.vx,
      current_state.vy,
      current_state.vw;

    Eigen::VectorXd reference_stack(6 * horizon_);
    Eigen::VectorXd feedforward_stack(3 * horizon_);
    double previous_unwrapped_yaw = current_state.yaw;
    for (int i = 0; i < horizon_; ++i) {
      const auto& point = reference[static_cast<std::size_t>(i)];
      const double unwrapped_yaw = previous_unwrapped_yaw +
        normalizeAngle(point.yaw - previous_unwrapped_yaw);
      previous_unwrapped_yaw = unwrapped_yaw;

      reference_stack.segment<6>(6 * i) <<
        point.x, point.y, unwrapped_yaw,
        point.vx, point.vy, point.vw;

      const Eigen::Vector3d reference_velocity(
        point.vx, point.vy, point.vw);
      const Eigen::Vector3d reference_acceleration(
        point.ax, point.ay, point.aw);
      // For dv/dt = (u_cmd - v)/tau, the continuous feedforward that produces
      // reference acceleration is u_ff = v_ref + tau * a_ref.
      feedforward_stack.segment<3>(3 * i) = reference_velocity +
        response_time_constants_.cwiseProduct(reference_acceleration);
    }

    // Predicted state when only the nominal trajectory feedforward is applied.
    const Eigen::VectorXd nominal_error =
      phi_ * initial_state + gamma_ * feedforward_stack - reference_stack;

    // Penalise changes in the complete command. The first change is measured
    // from the command actually published on the preceding control cycle.
    Eigen::VectorXd feedforward_difference =
      difference_matrix_ * feedforward_stack;
    feedforward_difference.head<3>() -= toVector(previous_command);

    const Eigen::VectorXd gradient = 2.0 * (
      gamma_.transpose() * q_big_ * nominal_error +
      difference_matrix_.transpose() * r_big_ * feedforward_difference);

    const Eigen::VectorXd correction_sequence = ldlt_.solve(-gradient);
    if (ldlt_.info() != Eigen::Success || !correction_sequence.allFinite()) {
      return false;
    }

    last_optimality_residual_ =
      (hessian_ * correction_sequence + gradient).lpNorm<Eigen::Infinity>();
    const Eigen::Vector3d first_command =
      feedforward_stack.head<3>() + correction_sequence.head<3>();
    command.vx = first_command.x();
    command.vy = first_command.y();
    command.vw = first_command.z();
    return finite(command);
  }

private:
  static bool validResponseTimeConstants(const Eigen::Vector3d& values)
  {
    return values.allFinite() && (values.array() > 0.0).all();
  }

  static bool finite(const State& state)
  {
    return std::isfinite(state.x) && std::isfinite(state.y) &&
           std::isfinite(state.yaw) && std::isfinite(state.vx) &&
           std::isfinite(state.vy) && std::isfinite(state.vw);
  }

  static bool finite(const TrajectoryPoint& point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.yaw) && std::isfinite(point.vx) &&
           std::isfinite(point.vy) && std::isfinite(point.vw) &&
           std::isfinite(point.ax) && std::isfinite(point.ay) &&
           std::isfinite(point.aw);
  }

  static bool finite(const ControlCmd& command)
  {
    return std::isfinite(command.vx) && std::isfinite(command.vy) &&
           std::isfinite(command.vw);
  }

  static Eigen::Vector3d toVector(const ControlCmd& command)
  {
    return {command.vx, command.vy, command.vw};
  }

  void buildModelAndPredictionMatrices()
  {
    const Eigen::Vector3d decay =
      (-Eigen::Vector3d::Constant(dt_)
      .cwiseQuotient(response_time_constants_)).array().exp().matrix();
    const Eigen::Vector3d velocity_from_state =
      response_time_constants_.cwiseProduct(
      Eigen::Vector3d::Ones() - decay);
    const Eigen::Vector3d position_from_command =
      Eigen::Vector3d::Constant(dt_) - velocity_from_state;

    state_matrix_.setZero();
    state_matrix_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    state_matrix_.block<3, 3>(0, 3) =
      velocity_from_state.asDiagonal();
    state_matrix_.block<3, 3>(3, 3) = decay.asDiagonal();

    input_matrix_.setZero();
    input_matrix_.block<3, 3>(0, 0) =
      position_from_command.asDiagonal();
    input_matrix_.block<3, 3>(3, 0) =
      (Eigen::Vector3d::Ones() - decay).asDiagonal();

    phi_.resize(6 * horizon_, 6);
    gamma_ = Eigen::MatrixXd::Zero(6 * horizon_, 3 * horizon_);
    Eigen::Matrix<double, 6, 6> accumulated =
      Eigen::Matrix<double, 6, 6>::Identity();
    for (int i = 0; i < horizon_; ++i) {
      accumulated = state_matrix_ * accumulated;
      phi_.block<6, 6>(6 * i, 0) = accumulated;
      for (int j = 0; j <= i; ++j) {
        Eigen::Matrix<double, 6, 6> transition =
          Eigen::Matrix<double, 6, 6>::Identity();
        for (int k = i; k > j; --k) {
          transition = transition * state_matrix_;
        }
        gamma_.block<6, 3>(6 * i, 3 * j) =
          transition * input_matrix_;
      }
    }
  }

  void buildDifferenceMatrix()
  {
    difference_matrix_ = Eigen::MatrixXd::Zero(
      3 * horizon_, 3 * horizon_);
    for (int i = 0; i < horizon_; ++i) {
      difference_matrix_.block<3, 3>(3 * i, 3 * i) =
        Eigen::Matrix3d::Identity();
      if (i > 0) {
        difference_matrix_.block<3, 3>(3 * i, 3 * (i - 1)) =
          -Eigen::Matrix3d::Identity();
      }
    }
  }

  bool rebuildCost(std::string* error)
  {
    q_big_ = Eigen::MatrixXd::Zero(6 * horizon_, 6 * horizon_);
    correction_big_ = Eigen::MatrixXd::Zero(
      3 * horizon_, 3 * horizon_);
    r_big_ = Eigen::MatrixXd::Zero(3 * horizon_, 3 * horizon_);
    for (int i = 0; i < horizon_; ++i) {
      const Eigen::Matrix<double, 6, 1> q =
        i + 1 == horizon_ ? 3.0 * q_diag_ : q_diag_;
      q_big_.block<6, 6>(6 * i, 6 * i) = q.asDiagonal();
      correction_big_.block<3, 3>(3 * i, 3 * i) =
        correction_diag_.asDiagonal();
      r_big_.block<3, 3>(3 * i, 3 * i) = r_diag_.asDiagonal();
    }

    hessian_ = 2.0 * (
      gamma_.transpose() * q_big_ * gamma_ +
      correction_big_ +
      difference_matrix_.transpose() * r_big_ * difference_matrix_);
    hessian_ = 0.5 * (hessian_ + hessian_.transpose());
    ldlt_.compute(hessian_);
    factorization_valid_ = ldlt_.info() == Eigen::Success &&
      (ldlt_.vectorD().array() > 0.0).all();
    if (!factorization_valid_ && error != nullptr) {
      *error = "MPC Hessian is not positive definite";
    }
    return factorization_valid_;
  }

  int horizon_;
  double dt_;
  Eigen::Vector3d response_time_constants_;
  Eigen::Matrix<double, 6, 1> q_diag_;
  Eigen::Vector3d correction_diag_;
  Eigen::Vector3d r_diag_;
  Eigen::Matrix<double, 6, 6> state_matrix_;
  Eigen::Matrix<double, 6, 3> input_matrix_;
  Eigen::MatrixXd phi_;
  Eigen::MatrixXd gamma_;
  Eigen::MatrixXd difference_matrix_;
  Eigen::MatrixXd q_big_;
  Eigen::MatrixXd correction_big_;
  Eigen::MatrixXd r_big_;
  Eigen::MatrixXd hessian_;
  Eigen::LDLT<Eigen::MatrixXd> ldlt_;
  bool factorization_valid_{false};
  double last_optimality_residual_{0.0};
};

}  // namespace robot_control
