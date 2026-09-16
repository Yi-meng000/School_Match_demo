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
        double vx{0.0};    // 必须传入底盘真实的纵向速度
        double vy{0.0};    // 真实的横向速度
        double vw{0.0};    // 真实的角速度
    };

    struct TrajectoryPoint {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        double vx{0.0};  
        double vy{0.0};
        double vw{0.0};
    };

    struct ControlCmd {
        double vx{0.0};  
        double vy{0.0};
        double vw{0.0};
    };

    class MpcController {
    public:
        MpcController(int N, double dt) : N_(N), dt_(dt) {
            // 前 3 项是位姿误差 [ex, ey, eyaw]，后 3 项是速度误差 [vx~, vy~, vw~]。
            // 速度项不能全给 0：否则速度剖面的跟踪完全由位置环代偿，
            // 实测会有 16%~32% 的速度超调。纵向和偏航通道给个小权重把剖面拉回来。
            // 横向通道刻意留小 —— 全向底盘不需要压制横向速度，权重给大了会跟 ey 的横向纠偏打架。
            q_diag_ << 120.0, 120.0, 90.0, 2.0, 0.5, 2.0;
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

            // 1. 计算机体坐标系误差 (针对轨迹的第 0 个点)
            const auto& pt0 = ref_traj[0];
            double dx = pt0.x - current_state.x;
            double dy = pt0.y - current_state.y;
            double dyaw = normalizeAngle(pt0.yaw - current_state.yaw);

            double cos_yaw = std::cos(current_state.yaw);
            double sin_yaw = std::sin(current_state.yaw);

            Eigen::Vector3d e_body;
            e_body(0) =  cos_yaw * dx + sin_yaw * dy;
            e_body(1) = -sin_yaw * dx + cos_yaw * dy;
            e_body(2) = dyaw;

            // 限幅：路径切换瞬间的误差尖峰不应该直接变成指令尖峰
            e_body(0) = std::max(-e_max_(0), std::min(e_max_(0), e_body(0)));
            e_body(1) = std::max(-e_max_(1), std::min(e_max_(1), e_body(1)));
            e_body(2) = std::max(-e_max_(2), std::min(e_max_(2), e_body(2)));

            // [新增闭环] 计算当前真实的初态速度误差 (防止静止起步时命令突跳)
            Eigen::Vector3d current_u_tilde;
            current_u_tilde(0) = current_state.vx - pt0.vx;
            current_u_tilde(1) = current_state.vy - pt0.vy;
            current_u_tilde(2) = current_state.vw - pt0.vw;

            // 2. 动态构造未来 N 步的时变状态矩阵 A_tilde_seq 和 B_tilde_seq
            std::vector<Eigen::Matrix<double, 6, 6>> A_tilde_seq(N_);
            std::vector<Eigen::Matrix<double, 6, 3>> B_tilde_seq(N_);

            for (int k = 0; k < N_; ++k) {
                const auto& pt = ref_traj[k];
                
                Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity();
                Ad(0, 1) =  dt_ * pt.vw;
                Ad(0, 2) = -dt_ * pt.vy; 
                Ad(1, 0) = -dt_ * pt.vw;
                Ad(1, 2) =  dt_ * pt.vx; 

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
            xi.segment<3>(0) = e_body;
            xi.segment<3>(3) = current_u_tilde; // 使用真实的初始速度误差

            // D 是已知扰动带来的仿射项，只进梯度
            Eigen::VectorXd g = 2.0 * Theta.transpose() * Q_big * (Psi * xi + D);

            // 6. 物理加速度边界约束
            Eigen::VectorXd lb(dim_u);
            Eigen::VectorXd ub(dim_u);
            Eigen::Vector3d max_delta = max_accel_ * dt_;
            for (int i = 0; i < N_; ++i) {
                lb.segment<3>(i * 3) = -max_delta;
                ub.segment<3>(i * 3) =  max_delta;
            }

            Eigen::SparseMatrix<double> Ac_sparse(dim_u, dim_u);
            Ac_sparse.setIdentity();
            Eigen::SparseMatrix<double> H_sparse = H_dense.sparseView();

            // 7. OSQP 求解
            OsqpEigen::Solver solver;
            solver.settings()->setVerbosity(false); 
            solver.settings()->setWarmStart(true);
            solver.settings()->setAbsoluteTolerance(1e-4);
            solver.settings()->setRelativeTolerance(1e-4);
            solver.settings()->setMaxIteration(200);

            solver.data()->setNumberOfVariables(dim_u);
            solver.data()->setNumberOfConstraints(dim_u);

            if (!solver.data()->setHessianMatrix(H_sparse)) return false;
            if (!solver.data()->setGradient(g)) return false;
            if (!solver.data()->setLinearConstraintsMatrix(Ac_sparse)) return false;
            if (!solver.data()->setLowerBound(lb)) return false;
            if (!solver.data()->setUpperBound(ub)) return false;

            if (!solver.initSolver()) return false;
            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;

            // 8. 提取最优解并从实际速度平滑叠加
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
        Eigen::Vector3d e_max_{0.30, 0.30, M_PI / 6.0};  // 初态误差限幅 (m, m, rad)
    };
}
