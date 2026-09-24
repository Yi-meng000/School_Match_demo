#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>

namespace robot_control
{
    struct State {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        double vx{0.0};    // 世界系 x 方向实测速度
        double vy{0.0};    // 世界系 y 方向实测速度
        double vw{0.0};    // 真实的角速度
    };

    struct TrajectoryPoint {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        double vx{0.0};   // 世界系 x 方向参考速度
        double vy{0.0};   // 世界系 y 方向参考速度
        double vw{0.0};
    };

    struct ControlCmd {
        double vx{0.0};   // 世界系 x 方向速度指令
        double vy{0.0};   // 世界系 y 方向速度指令
        double vw{0.0};
    };

    class MpcController {
    public:
        MpcController(int N, double dt) : N_(N), dt_(dt) {
            // 前 3 项是位姿误差 [ex, ey, eyaw]，后 3 项是速度误差 [vx~, vy~, vw~]。
            // 平动速度误差必须有足够权重，否则优化器会为了追位置窗口而长期超出
            // 速度参考。横向速度同样是全向底盘的平动速度，应当跟随对应参考分量。
            q_diag_ << 120.0, 120.0, 90.0, 50.0, 50.0, 2.0;
            r_diag_ << 1.5, 1.5, 0.8;
            max_accel_ << 3.0, 3.0, 4.0;
        }

        void setWeights(const Eigen::Matrix<double, 6, 1>& q_diag,
                        const Eigen::Vector3d& r_diag,
                        const Eigen::Vector3d& max_accel)
        {
            q_diag_ = q_diag;
            r_diag_ = r_diag;
            max_accel_ = max_accel;
        }

        // The translational trajectory can enter an explicitly configured
        // emergency-braking mode.  Its velocity envelope is only executable
        // when the MPC's command-increment limit is updated consistently.
        void setAccelerationLimits(const Eigen::Vector3d& max_accel)
        {
            max_accel_ = max_accel;
        }

        // Limit the magnitude of each predicted planar command relative to
        // the corresponding reference speed.  `tracking_margin` is a small
        // allowance for position correction.  If the measured speed already
        // exceeds that target, `braking_decel` creates a reachable decreasing
        // bound instead of requesting an instantaneous, infeasible stop.
        void setReferenceRelativeSpeedLimit(double tracking_margin, double braking_decel)
        {
            reference_speed_margin_ = std::max(0.0, tracking_margin);
            reference_speed_braking_decel_ = std::max(1e-3, braking_decel);
            reference_speed_limit_enabled_ = true;
        }

        // 初态误差限幅。路径切换（新障碍物让规划器重发了一条以当前位置为起点的
        // 绕行路径）时，横向误差可能瞬间达到几十厘米，直接送进 MPC 会让它猛打方向。
        // 这里饱和掉，代价是收敛慢一点，换指令平滑。
        void setErrorLimits(double xy_max, double yaw_max)
        {
            e_max_(0) = e_max_(1) = std::max(1e-3, xy_max);
            e_max_(2) = std::max(1e-3, yaw_max);
        }

        static double normalizeAngle(double angle) {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

        // [核心修改] 传入的不再是单点，而是长度 >= N 的未来轨迹窗口
        bool solveMPC(const State& current_state, 
                      const std::vector<TrajectoryPoint>& ref_traj, 
                      ControlCmd& cmd_out)
        {
            if (ref_traj.size() < static_cast<size_t>(N_)) {
                std::cerr << "[MPC] 参考轨迹长度不足 N 步！" << std::endl;
                return false;
            }

            // MPC 的平动位置、速度和速度增量都使用同一世界系。
            // 坐标转换属于调用层，不在控制器内部进行。
            // yaw/vw 仍然是同一 MPC 状态和控制量的第三个分量。
            // 1. 计算世界系位姿误差（针对轨迹的第 0 个点）
            const auto& pt0 = ref_traj[0];
            double dx = pt0.x - current_state.x;
            double dy = pt0.y - current_state.y;
            double dyaw = normalizeAngle(pt0.yaw - current_state.yaw);

            Eigen::Vector3d e_world;
            e_world(0) = dx;
            e_world(1) = dy;
            e_world(2) = dyaw;

            // 限幅：路径切换瞬间的误差尖峰不应该直接变成指令尖峰
            e_world(0) = std::max(-e_max_(0), std::min(e_max_(0), e_world(0)));
            e_world(1) = std::max(-e_max_(1), std::min(e_max_(1), e_world(1)));
            e_world(2) = std::max(-e_max_(2), std::min(e_max_(2), e_world(2)));

            // [新增闭环] 计算当前真实的初态速度误差 (防止静止起步时命令突跳)
            Eigen::Vector3d current_u_tilde;
            current_u_tilde(0) = current_state.vx - pt0.vx;
            current_u_tilde(1) = current_state.vy - pt0.vy;
            current_u_tilde(2) = current_state.vw - pt0.vw;

            // 2. 动态构造未来 N 步的时变状态矩阵 A_tilde_seq 和 B_tilde_seq
            std::vector<Eigen::Matrix<double, 6, 6>> A_tilde_seq(N_);
            std::vector<Eigen::Matrix<double, 6, 3>> B_tilde_seq(N_);

            for (int k = 0; k < N_; ++k) {
                // 在世界系中，x/y 位置误差只由世界系速度误差积分；
                // yaw 误差由角速度误差积分。不再把参考车体坐标
                // 旋转项混入平动误差模型。
                Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity();

                Eigen::Matrix3d Bd = -dt_ * Eigen::Matrix3d::Identity();

                A_tilde_seq[k].setZero();
                A_tilde_seq[k].block<3, 3>(0, 0) = Ad;
                A_tilde_seq[k].block<3, 3>(0, 3) = Bd;
                A_tilde_seq[k].block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

                B_tilde_seq[k].setZero();
                B_tilde_seq[k].block<3, 3>(0, 0) = Bd;
                B_tilde_seq[k].block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
            }

            // 3. 时变预测展开 (完全重写为链式相乘)
            const int dim_xi = 6 * N_;
            const int dim_u = 3 * N_;
            Eigen::MatrixXd Psi(dim_xi, 6);
            Eigen::MatrixXd Theta = Eigen::MatrixXd::Zero(dim_xi, dim_u);

            // 展开 Psi 矩阵: Psi[i] = A_i * A_{i-1} * ... * A_0
            Eigen::Matrix<double, 6, 6> A_accum = Eigen::Matrix<double, 6, 6>::Identity();
            for (int i = 0; i < N_; ++i) {
                A_accum = A_tilde_seq[i] * A_accum;
                Psi.block<6, 6>(i * 6, 0) = A_accum;
            }

            // 3b. 参考加速度作为已知扰动
            // 增广状态里 u_tilde_k = u_k - u_ref_k，而参考速度沿路径是变化的，所以
            //     u_tilde_{k+1} = u_tilde_k + delta_u_k - (u_ref_{k+1} - u_ref_k)
            // 最后一项 (u_ref 的变化量) 是已知量，必须作为仿射项进预测。
            // 漏掉它等于假设参考速度在视界内恒定，加减速段和弯道会有系统性预测偏差。
            // 它只进梯度 g，不影响 Hessian H。
            std::vector<Eigen::Matrix<double, 6, 1>> d_seq(N_);
            for (int k = 0; k < N_; ++k) {
                d_seq[k].setZero();
                if (k + 1 < N_) {   // 最后一个点没有下一步参考，扰动记 0
                    d_seq[k](3) = -(ref_traj[k + 1].vx - ref_traj[k].vx);
                    d_seq[k](4) = -(ref_traj[k + 1].vy - ref_traj[k].vy);
                    d_seq[k](5) = -(ref_traj[k + 1].vw - ref_traj[k].vw);
                }
            }

            // 展开 Theta 矩阵，同时把已知扰动累积成 D (X = Psi*xi + Theta*dU + D)
            Eigen::MatrixXd D = Eigen::MatrixXd::Zero(dim_xi, 1);
            for (int i = 0; i < N_; ++i) {
                for (int j = 0; j <= i; ++j) {
                    Eigen::Matrix<double, 6, 6> A_step = Eigen::Matrix<double, 6, 6>::Identity();
                    // k 倒序右乘, 得到 A_i * A_{i-1} * ... * A_{j+1}
                    for (int k = i; k > j; --k) {
                        A_step = A_step * A_tilde_seq[k];
                    }
                    Theta.block<6, 3>(i * 6, j * 3) = A_step * B_tilde_seq[j];
                    D.block<6, 1>(i * 6, 0) += A_step * d_seq[j];
                }
            }

            // 4. 构造权重矩阵
            Eigen::MatrixXd Q_big = Eigen::MatrixXd::Zero(dim_xi, dim_xi);
            Eigen::MatrixXd R_big = Eigen::MatrixXd::Zero(dim_u, dim_u);

            for (int i = 0; i < N_ - 1; ++i) {
                Q_big.block<6, 6>(i * 6, i * 6) = q_diag_.asDiagonal();
                R_big.block<3, 3>(i * 3, i * 3) = r_diag_.asDiagonal();
            }
            Eigen::Matrix<double, 6, 1> f_diag = 3.0 * q_diag_;
            Q_big.block<6, 6>((N_ - 1) * 6, (N_ - 1) * 6) = f_diag.asDiagonal();
            R_big.block<3, 3>((N_ - 1) * 3, (N_ - 1) * 3) = r_diag_.asDiagonal();            

            // 5. 构造 QP 标准型矩阵
            Eigen::MatrixXd H_dense = 2.0 * (Theta.transpose() * Q_big * Theta + R_big);

            Eigen::Matrix<double, 6, 1> xi;
            xi.segment<3>(0) = e_world;
            xi.segment<3>(3) = current_u_tilde; // 使用真实的初始速度误差

            // D 是已知扰动带来的仿射项，只进梯度
            Eigen::VectorXd g = 2.0 * Theta.transpose() * Q_big * (Psi * xi + D);

            // 6. 物理加速度边界约束。决策量是每步速度增量 delta_u。
            Eigen::VectorXd lb(dim_u);
            Eigen::VectorXd ub(dim_u);
            Eigen::Vector3d max_delta = max_accel_ * dt_;
            for (int i = 0; i < N_; ++i) {
                lb.segment<3>(i * 3) = -max_delta;
                ub.segment<3>(i * 3) =  max_delta;
            }

            // The acceleration bound alone does not constrain the accumulated
            // command speed.  Add an 8-sided inscribed polygon for every
            // predicted planar command:
            //   ||u_xy[k]|| <= ||u_ref[k]|| + tracking_margin.
            // A regular inner polygon keeps the QP linear while never allowing
            // a vector outside the requested circular speed bound.
            constexpr int kPlanarSpeedDirections = 8;
            const int speed_constraint_count = reference_speed_limit_enabled_ ?
                N_ * kPlanarSpeedDirections : 0;
            const int constraint_count = dim_u + speed_constraint_count;
            constexpr double kConstraintInfinity = 1e20;
            constexpr double kPi = 3.14159265358979323846;
            Eigen::MatrixXd constraint_dense = Eigen::MatrixXd::Zero(constraint_count, dim_u);
            constraint_dense.topLeftCorner(dim_u, dim_u).setIdentity();
            Eigen::VectorXd constraint_lower(constraint_count);
            Eigen::VectorXd constraint_upper(constraint_count);
            constraint_lower.head(dim_u) = lb;
            constraint_upper.head(dim_u) = ub;

            if (reference_speed_limit_enabled_) {
                const double current_planar_speed = std::hypot(current_state.vx, current_state.vy);
                const double polygon_inradius_ratio = std::cos(kPi / static_cast<double>(kPlanarSpeedDirections));
                int row = dim_u;
                for (int k = 0; k < N_; ++k) {
                    const double reference_speed = std::hypot(ref_traj[k].vx, ref_traj[k].vy);
                    const double target_speed = reference_speed + reference_speed_margin_;
                    const double reachable_speed = std::max(
                        0.0,
                        current_planar_speed - reference_speed_braking_decel_ * dt_ * static_cast<double>(k + 1));
                    const double speed_bound = std::max(target_speed, reachable_speed);
                    const double polygon_inradius = speed_bound * polygon_inradius_ratio;

                    for (int direction = 0; direction < kPlanarSpeedDirections; ++direction, ++row) {
                        // Polygon normals lie halfway between its vertices, so
                        // the vertices on the cardinal/diagonal directions are
                        // exactly at speed_bound and the whole polygon lies in
                        // the speed_bound circle.
                        const double angle =
                            (static_cast<double>(direction) + 0.5) * 2.0 * kPi /
                            static_cast<double>(kPlanarSpeedDirections);
                        const double nx = std::cos(angle);
                        const double ny = std::sin(angle);
                        for (int j = 0; j <= k; ++j) {
                            constraint_dense(row, 3 * j) = nx;
                            constraint_dense(row, 3 * j + 1) = ny;
                        }
                        constraint_lower(row) = -kConstraintInfinity;
                        constraint_upper(row) = polygon_inradius -
                            nx * current_state.vx - ny * current_state.vy;
                    }
                }
            }

            Eigen::SparseMatrix<double> Ac_sparse = constraint_dense.sparseView();
            Eigen::SparseMatrix<double> H_sparse = H_dense.sparseView();

            // 7. OSQP 求解
            OsqpEigen::Solver solver;
            solver.settings()->setVerbosity(false); 
            solver.settings()->setWarmStart(true);
            solver.settings()->setAbsoluteTolerance(1e-4);
            solver.settings()->setRelativeTolerance(1e-4);
            solver.settings()->setMaxIteration(200);

            solver.data()->setNumberOfVariables(dim_u);
            solver.data()->setNumberOfConstraints(constraint_count);

            if (!solver.data()->setHessianMatrix(H_sparse)) return false;
            if (!solver.data()->setGradient(g)) return false;
            if (!solver.data()->setLinearConstraintsMatrix(Ac_sparse)) return false;
            if (!solver.data()->setLowerBound(constraint_lower)) return false;
            if (!solver.data()->setUpperBound(constraint_upper)) return false;

            if (!solver.initSolver()) return false;
            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;

            // 8. 提取最优解并从实际速度平滑叠加。平动分量仍是世界系，
            // 调用者负责在底盘边界将其转换为车体系指令。
            Eigen::VectorXd delta_U_opt = solver.getSolution();
            Eigen::Vector3d delta_u0 = delta_U_opt.segment<3>(0);

            cmd_out.vx = current_state.vx + delta_u0(0);
            cmd_out.vy = current_state.vy + delta_u0(1);
            cmd_out.vw = current_state.vw + delta_u0(2);

            return true;
        }

    private:
        int N_;
        double dt_;
        Eigen::Matrix<double, 6, 1> q_diag_; 
        Eigen::Vector3d r_diag_; 
        Eigen::Vector3d max_accel_;
        bool reference_speed_limit_enabled_{false};
        double reference_speed_margin_{0.0};
        double reference_speed_braking_decel_{1.0};
        Eigen::Vector3d e_max_{1000.0, 1000.0, M_PI};  // 初态误差限幅 (m, m, rad)
    };
}
