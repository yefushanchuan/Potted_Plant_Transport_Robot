#include "timed_motion.h"
#include "ieskf.h"
// 引入头文件，定义了 State 和 IESKF 类的接口、别名（M3D,V3D,V22D 等）

double State::gravity = 9.81;  
// 静态成员变量，重力加速度常量，单位 m/s^2

// ---------------- 辅助函数 ----------------

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
// 计算李代数 SO(3) 的左雅可比矩阵的转置，用于旋转增量扰动传播

M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}
// 计算左雅可比逆矩阵的转置，用于旋转误差反映到状态差分

// ---------------- State 的运算符重载 ----------------

void State::operator+=(const V22D &delta) {
    // delta 向量共有 22 维，依次更新各个子状态

    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    // 前3维是旋转扰动 (SO(3))，更新世界到IMU的旋转矩阵

    t_wi += delta.segment<3>(3);
    // 接下来的3维是平移扰动，更新位置

    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    // 接下来的3维是 LiDAR相对IMU的旋转扰动

    t_il += delta.segment<3>(9);
    // 接下来的3维是 LiDAR相对IMU的平移扰动

    v += delta.segment<3>(12);
    // 接下来的3维是速度扰动

    bg += delta.segment<3>(15);
    // 接下来的3维是陀螺仪偏置扰动

    ba += delta.segment<3>(18);
    td += delta(21);
    // 接下来的3维是加速度计偏置扰动
}

V22D State::operator-(const State &other) const {
    // 定义两个 State 之间的差异运算，返回 22维增量
    V22D delta = V22D::Zero();

    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    // 旋转误差 = log(R_other^T * R_current)，映射到so(3)

    delta.segment<3>(3) = t_wi - other.t_wi;
    // 位置差

    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    // LiDAR-IMU 旋转差

    delta.segment<3>(9) = t_il - other.t_il;
    // LiDAR-IMU 平移差

    delta.segment<3>(12) = v - other.v;
    // 速度差

    delta.segment<3>(15) = bg - other.bg;
    // 陀螺仪偏置差

    delta.segment<3>(18) = ba - other.ba;
    delta(21) = td - other.td;
    // 加速度计偏置差

    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    // 重载输出运算符，打印 State 的所有成员
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    // 世界到IMU旋转（欧拉角ZYX顺序）

    os << "t_il: " << state.t_il.transpose() << std::endl;
    // IMU到LiDAR平移（外参平移）

    os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;
    // IMU到LiDAR旋转（外参旋转）

    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    // 世界到IMU平移（IMU在世界系中的位置）

    os << "v: " << state.v.transpose() << std::endl;
    // 速度

    os << "bg: " << state.bg.transpose() << std::endl;
    // 陀螺仪偏置

    os << "ba: " << state.ba.transpose() << std::endl;
    // 加速度计偏置

    os << "g: " << state.g.transpose() << std::endl;
    // 重力向量

    os << "===============END================" << std::endl;

    return os;
}

// ---------------- IESKF 的预测步骤 ----------------

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    // inp：IMU输入（陀螺仪gyro，加速度acc,dt：时间间隔,Q：过程噪声协方差矩阵 (12x12)

    const State before = m_x;
    m_F = fast_time::integrate(m_x, inp.acc, inp.gyro, dt);
    // 控制输入矩阵 G
    m_G.setZero();
    // 姿态噪声
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro - before.bg) * dt) * dt;

    // 加速度噪声
    m_G.block<3, 3>(12, 3) = -before.r_wi * dt;

    // 陀螺仪偏置随机游走
    m_G.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;

    // 加速度偏置随机游走
    m_G.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    // 更新状态

    // 更新协方差 P
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
    m_P(21, 21) += time_options.td_noise * time_options.td_noise * dt;

}

// ---------------- IESKF 的更新步骤 ----------------

bool IESKF::update() {

    report = TimeOffsetReport();
    const State prior = m_x;
    const M22D prior_cov = m_P;
    Eigen::LDLT<M22D> ps(prior_cov);
    if (ps.info() != Eigen::Success || !ps.isPositive()) {
        report.reason = "invalid_covariance";
        return false;
    }
    const M22D pinv = ps.solve(M22D::Identity());
    SharedState obs;
    m_loss_func(m_x, obs);
    if (!obs.valid) {
        report.matches = obs.count;
        report.fitness = obs.fitness;
        report.rms = obs.res;
        report.reason = obs.reason;
        return false;
    }
    Eigen::Matrix<double, 6, 6> pose_info;
    Eigen::Matrix<double, 6, 1> c;
    const int idx[6] = {0, 1, 2, 3, 4, 5};
    for (int i = 0; i < 6; ++i) {
        c(i) = obs.H(idx[i], 21);
        for (int j = 0; j < 6; ++j)
            pose_info(i, j) = obs.H(idx[i], idx[j]);
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(pose_info);
    const auto inv =
        eig.eigenvalues().unaryExpr([](double x) { return x > 1e-8 ? 1 / x : 0.; }).eval();
    report.td_information =
        std::max(0., obs.H(21, 21) - c.dot(eig.eigenvectors() * inv.asDiagonal() *
                                           eig.eigenvectors().transpose() * c));
    const bool estimate = time_excited &&
                          report.td_information >= time_options.min_td_information;
    // Until time is observable, use the frozen nominal timing for the geometric update.
    // Keeping its Jacobian here lets static IMU noise change the pose solve through
    // a latent time correction that is subsequently discarded.
    if (!estimate) {
        obs.H.row(21).setZero();
        obs.H.col(21).setZero();
        obs.b(21) = 0;
    }
    const double low = std::max(-time_options.max_abs_td, prior.td - time_options.max_td_step);
    const double high = std::min(time_options.max_abs_td, prior.td + time_options.max_td_step);
    for (size_t i = 0; i < m_max_iter; ++i) {
        if (i) {
            m_loss_func(m_x, obs);
            if (!estimate) {
                obs.H.row(21).setZero();
                obs.H.col(21).setZero();
                obs.b(21) = 0;
            }
        }
        if (!obs.valid) {
            m_x = prior;
            report.reason = obs.reason;
            return false;
        }
        const V22D e = m_x - prior;
        M22D j = M22D::Identity();
        j.block<3, 3>(0, 0) = JrInv(e.segment<3>(0));
        j.block<3, 3>(6, 6) = JrInv(e.segment<3>(6));
        const M22D normal = j.transpose() * pinv * j + obs.H;
        Eigen::LDLT<M22D> solver(normal);
        if (solver.info() != Eigen::Success || !solver.isPositive()) {
            m_x = prior;
            report.reason = "singular_update";
            return false;
        }
        const V22D rhs = -j.transpose() * pinv * e - obs.b;
        V22D delta = solver.solve(rhs);
        // A frozen coordinate uses a Schmidt update, preserving its uncertainty.
        if (!estimate)
            delta(21) = 0;
        const double bounded = std::clamp(m_x.td + delta(21), low, high);
        if (estimate && bounded != m_x.td + delta(21)) {
            delta(21) = bounded - m_x.td;
            const Eigen::Matrix<double, 21, 21> active = normal.template topLeftCorner<21, 21>();
            delta.template head<21>() = active.ldlt().solve(
                rhs.template head<21>() - normal.template block<21, 1>(0, 21) * delta(21));
            report.td_limited = true;
        }
        if (!delta.allFinite()) {
            m_x = prior;
            report.reason = "nonfinite_update";
            return false;
        }
        m_x += delta;

        report.iterations = i + 1;
        if (m_stop_func(delta) && std::abs(delta(21)) < 1e-6)
            break;
    }
    m_loss_func(m_x, obs);
    if (!estimate) {
        obs.H.row(21).setZero();
        obs.H.col(21).setZero();
        obs.b(21) = 0;
    }
    if (!obs.valid) {
        m_x = prior;
        report.reason = "poor_final_match";
        return false;
    }
    const V22D e = m_x - prior;
    M22D j = M22D::Identity();
    j.block<3, 3>(0, 0) = JrInv(e.segment<3>(0));
    j.block<3, 3>(6, 6) = JrInv(e.segment<3>(6));
    const M22D normal = j.transpose() * pinv * j + obs.H;
    M22D gain = normal.ldlt().solve(M22D::Identity());
    const bool boundary =
        estimate && (std::abs(m_x.td - low) < 1e-10 || std::abs(m_x.td - high) < 1e-10);
    if (boundary) {
        const Eigen::Matrix<double, 21, 21> active = normal.template topLeftCorner<21, 21>();
        gain.setZero();
        gain.template topLeftCorner<21, 21>() =
            active.ldlt().solve(Eigen::Matrix<double, 21, 21>::Identity());
    } else if (!estimate)
        gain.row(21).setZero();
    const M22D ji = j.inverse(), plocal = ji * prior_cov * ji.transpose();
    const M22D a = M22D::Identity() - gain * obs.H;
    M22D post = a * plocal * a.transpose() + gain * obs.H * gain.transpose();
    post = (.5 * (post + post.transpose())).eval();
    if (!post.allFinite() || Eigen::LLT<M22D>(post).info() != Eigen::Success) {
        m_x = prior;
        report.reason = "invalid_posterior";
        return false;
    }
    m_P = post;

    report.accepted = true;
    report.td_updated = estimate;
    report.matches = obs.count;
    report.fitness = obs.fitness;
    report.rms = obs.res;
    report.reason = estimate ? (boundary ? "tracking_td_limited" : "tracking")
                             : "dt_unobservable";
    return true;
}
