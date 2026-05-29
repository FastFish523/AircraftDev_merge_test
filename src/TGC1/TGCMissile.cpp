//
// Created by 17298 on 2026/4/22.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "TGC/TGCMissile.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::TGC {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor

    Missile::Missile() {
        _fileSaver = std::make_shared<FileSaver>("./Results/TGC/");
        _maxLoad   = 30;
        _s         = 0.223;
        _l         = 6.55;
        _b         = 6.55;
        _mass      = 1000;
        _inertia << 18.010, 0.0, 0.0, 0.0, 1191.985, 0.0, 0.0, 0.0, 1191.985;
        _kinematics._dynamics._aerodynamics.computeAeroCoefficientsCB =
                [this](const double alpha, const double beta, const double dx, const double dy, const double dz, const double Ma, double &CD, double &CL, double &CZ, double &Cl,
                       double &Cm, double &Cn) {
                    // 1. 计算马赫数相关的法向力导数
                    double ma = Ma;
                    if (Ma < 0.1) {
                        ma = 0.1;
                    }
                    double CN = 0.3 + 0.6 * ma * ma / (1 + 0.8 * ma * ma * ma * ma) + 4.0 / sqrt(1 + (ma * ma - 1) * (ma * ma - 1));
                    // 2. 计算马赫数相关的轴向力参数
                    const double CA0 = 0.03 + 0.0005 * ma;
                    const double k   = 0.8 + 0.0005 * ma;
                    // 3. 气动力系数
                    CD = -(CA0 + k * (alpha * alpha + beta * beta));
                    CL = CN * alpha;
                    CZ = -CN * beta;
                    // 4. 气动力矩系数
                    constexpr double cg_cf       = 0.5;
                    constexpr double rudder_rate = 1;
                    Cl                           = -CN * dx * cg_cf * 0.001;
                    Cm                           = -CL * cg_cf - CN * dz * cg_cf * rudder_rate;
                    Cn                           = CZ * cg_cf - CN * dy * cg_cf * rudder_rate;
                };
    }

    Missile::~Missile() = default;

// endregion

// region Public Methods
    void Missile::init(const double step, const Eigen::Vector3d &lla) {
        _step = step;
        _flyTime = 0.0;
        _launchFlag = false;
        _engine.reset();
        _currentRouteId = 0;
        _routePoints.clear();
        distance_deque.clear();
        _phase = GuidancePhase::Boost;
        _state.posEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(lla);
        _state.velEcf.setZero();
        _state.wnb_b.setZero();
        _state.qbn.setIdentity();
        _launchLLA = lla;
    }

    void Missile::launch(const double theta_f_d, const double psi_f_d) {
        Eigen::Vector3d lla            = this->lla();
        _state.qbn                     = ModelDevelop::Utils::CoordinateHelper::euler231ToQuaternion(psi_f_d, theta_f_d, 0);
        const Eigen::Vector3d vel_body = {30, 0, 0};
        const Eigen::Vector3d vel_nue  = ModelDevelop::Utils::CoordinateHelper::bodyToNueVelocity(vel_body, _state.qbn);
        _state.velEcf                  = ModelDevelop::Utils::CoordinateHelper::nueToEcefVelocity(vel_nue, lla.x(), lla.y());

        _launchFlag = true;
        _phase = GuidancePhase::Boost;
    }

    void Missile::setTargetEcf(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const bool clearQueue) {
        if (clearQueue) {
            distance_deque.clear();
        }
        _targetPosEcf = targetPosEcf;
        _targetVelEcf = targetVelEcf;
    }

    void Missile::setTargetLLA(const Eigen::Vector3d &targetPosLLa, const Eigen::Vector3d &targetVelEcf, const bool clearQueue) {
        if (clearQueue) {
            distance_deque.clear();
        }
        _targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetPosLLa);
        _targetVelEcf = targetVelEcf;
    }

    void Missile::setRoutePoints(const std::deque<Eigen::Vector3d> &routes) {
        if (routes.size() <= 1) {
            std::cout << "min route point count is 2" << std::endl;
            return;
        }
        _currentRouteId = 0;
        _routePoints.clear();
        _routePoints.push_back(lla());
        for (const auto &r: routes) {
            _routePoints.push_back(r);
        }
    }

    double Missile::update() {
        if (!_launchFlag)
            return -1;
        _rudder.setZero();
        _p_body.setZero();
        _m_body.setZero();

        // 推力 质量 转动惯量更新
        const auto [mass, P_body, inertia] = _engine.getEigenInfo(_step, _flyTime, _state);
        _p_body                            = P_body;
        _totalMass                         = _mass + mass;
        _inertia                           = _inertia + inertia;

        if (_targetPosEcf.has_value()) {
            LosInfo losInfo{};

            const auto gcInfo = _guidance.getMissionGCInfo(flyTime(), _p_body.norm(), _totalMass, _targetPosEcf.value(), _targetVelEcf, _state, _maxLoad, _routePoints, _currentRouteId);
            losInfo           = gcInfo.losInfo;
            const Eigen::Vector3d acc_cmd_v = gcInfo.acc_cmd_v;
            _phase            = gcInfo.phase;
            if (_currentRouteId == -1) {
                _routePoints.clear();
            }
            auto acc_cmd_b        = ModelDevelop::Utils::CoordinateHelper::velocityToBodyAcceleration(acc_cmd_v, this->alpha() / 57.3, this->beta() / 57.3);
            _acc_cmd_b_y          = acc_cmd_b.y();
            _acc_cmd_b_z          = acc_cmd_b.z();
            _sigma_az_dot         = losInfo.sigma_az_dot;
            _sigma_elv_dot        = losInfo.sigma_elv_dot;
            _sigma_elv            = losInfo.sigma_elv;
            _sigma_az             = losInfo.sigma_az;
            const auto [fst, snd] = _control.P6dof_Control(_step, acc_cmd_v, _state, _totalMass, _p_body, _imu_info, 1, 0, _s);
            _rudder               = fst;
            _m_body               = snd;
        }
        // rk4更新
        const Eigen::Vector3d acc_ecf = rk4(_rudder, _p_body, _m_body);
        _imu_info                     = _imu.getImuInfoBody(_state, acc_ecf);

        _flyTime += _step;

        _fileSaver->save_traj(this);
        _fileSaver->save_aero(this);

        const auto dis = targetDis();
        distance_deque.emplace_back(dis);
        if (distance_deque.size() > 4) {
            distance_deque.pop_front();
        }
        if (dis < 2000 && distance_deque.size() >= 4) {
            bool increasing = false;
            for (auto it = distance_deque.begin(); it + 1 != distance_deque.end(); ++it) {
                if (*it < *(it + 1)) {
                    increasing = true;
                    break;
                }
            }
            if (increasing || lla().z() <= 0) {
                const auto terminal_dis = *std::min_element(distance_deque.begin(), distance_deque.end());
                return terminal_dis;
            }
        }
        return -1;
    }

// endregion

// region Get/Set选择器
// endregion

// region Private Methods
    Eigen::Vector3d Missile::rk4(const Eigen::Vector3d &_rudder, const Eigen::Vector3d &P_body, const Eigen::Vector3d &M_body) {
        auto yn = _state;
        auto k1 = _kinematics.cal_d_state(_state, _totalMass, _inertia, P_body, M_body, _rudder, _s, _l, _b);
        State yn_1;
        yn_1.posEcf       = yn.posEcf + _step * 0.5 * k1.d_posEcf;
        yn_1.velEcf       = yn.velEcf + _step * 0.5 * k1.d_velEcf;
        yn_1.qbn.coeffs() = yn.qbn.coeffs() + _step * 0.5 * k1.d_qbn.coeffs();
        yn_1.wnb_b        = yn.wnb_b + _step * 0.5 * k1.d_wnb_b;

        auto k2 = _kinematics.cal_d_state(yn_1, _totalMass, _inertia, P_body, M_body, _rudder, _s, _l, _b);
        State yn_2;
        yn_2.posEcf       = yn.posEcf + _step * 0.5 * k2.d_posEcf;
        yn_2.velEcf       = yn.velEcf + _step * 0.5 * k2.d_velEcf;
        yn_2.qbn.coeffs() = yn.qbn.coeffs() + _step * 0.5 * k2.d_qbn.coeffs();
        yn_2.wnb_b        = yn.wnb_b + _step * 0.5 * k2.d_wnb_b;

        auto k3 = _kinematics.cal_d_state(yn_2, _totalMass, _inertia, P_body, M_body, _rudder, _s, _l, _b);
        State yn_3;
        yn_3.posEcf       = yn.posEcf + _step * 1 * k3.d_posEcf;
        yn_3.velEcf       = yn.velEcf + _step * 1 * k3.d_velEcf;
        yn_3.qbn.coeffs() = yn.qbn.coeffs() + _step * 1 * k3.d_qbn.coeffs();
        yn_3.wnb_b        = yn.wnb_b + _step * 1 * k3.d_wnb_b;

        auto k4 = _kinematics.cal_d_state(yn_3, _totalMass, _inertia, P_body, M_body, _rudder, _s, _l, _b);

        Eigen::Vector3d d_state_posEcf = (k1.d_posEcf + 2 * k2.d_posEcf + 2 * k3.d_posEcf + k4.d_posEcf) / 6;
        _state.posEcf += d_state_posEcf * _step;

        Eigen::Vector3d d_state_velEcf = (k1.d_velEcf + 2 * k2.d_velEcf + 2 * k3.d_velEcf + k4.d_velEcf) / 6;
        _state.velEcf += d_state_velEcf * _step;
        Eigen::Quaterniond d_qbn;
        d_qbn.coeffs() = (k1.d_qbn.coeffs() + 2 * k2.d_qbn.coeffs() + 2 * k3.d_qbn.coeffs() + k4.d_qbn.coeffs()) / 6;
        _state.qbn.coeffs() += d_qbn.coeffs() * _step;

        Eigen::Vector3d d_wn_bb = (k1.d_wnb_b + 2 * k2.d_wnb_b + 2 * k3.d_wnb_b + k4.d_wnb_b) / 6;
        _state.wnb_b += d_wn_bb * _step;

        return d_state_velEcf;
    }


    Derivative Missile::derivative() const {
        Derivative derivative;
        auto theta            = velocityTheta() / 57.3;
        auto pitch            = ModelDevelop::Utils::CoordinateHelper::quaternionToEuler231(_state.qbn).y() / 57.3;
        auto alpha            = this->alpha() / 57.3;
        auto beta             = this->beta() / 57.3;
        auto lla              = this->lla();
        auto rho              = ModelDevelop::Utils::Aerodynamics::calculateAtmosphereDensity(lla.z());
        auto dynamic_pressure = 0.5 * rho * V() * V();
        double dx             = rudder().x() / 57.3;
        double dy             = rudder().y() / 57.3;
        double dz             = rudder().z() / 57.3;
        double ma             = this->Ma();

        auto computeAeroCoefficientsCB = _kinematics._dynamics._aerodynamics.computeAeroCoefficientsCB;
        if (computeAeroCoefficientsCB == nullptr)
            return derivative;

        constexpr auto delta_angle    = 1 / 57.3;
        constexpr auto delta_velocity = 100;

        double CD, CL, CZ, Cl, Cm, Cn;
        computeAeroCoefficientsCB(alpha, beta, dx, dy, dz, ma, CD, CL, CZ, Cl, Cm, Cn);

        double CD0 = CD;
        double CL0 = CL;
        double CZ0 = CZ;
        double Cl0 = Cl;
        double Cn0 = Cn;
        double Cm0 = Cm;

        computeAeroCoefficientsCB(alpha + delta_angle, beta, dx, dy, dz, ma, CD, CL, CZ, Cl, Cm, Cn);

        double CD_alpha = CD;
        double CL_alpha = CL;
        double Cm_alpha = Cm;

        computeAeroCoefficientsCB(alpha, beta + delta_angle, dx, dy, dz, ma, CD, CL, CZ, Cl, Cm, Cn);


        double CZ_beta = CZ;
        double Cl_beta = Cl;
        double Cn_beta = Cn;

        computeAeroCoefficientsCB(alpha, beta, dx, dy, dz + delta_angle, ma, CD, CL, CZ, Cl, Cm, Cn);

        double CD_dz = CD;
        double CL_dz = CL;
        double Cm_dz = Cm;

        computeAeroCoefficientsCB(alpha, beta, dx, dy + delta_angle, dz, ma, CD, CL, CZ, Cl, Cm, Cn);

        double CZ_dy = CZ;
        double Cl_dy = Cl;
        double Cn_dy = Cn;

        computeAeroCoefficientsCB(alpha, beta, dx + delta_angle, dy, dz, ma, CD, CL, CZ, Cl, Cm, Cn);

        double Cl_dx = Cl;


        computeAeroCoefficientsCB(alpha, beta, dx + delta_angle, dy, dz, (V() + delta_velocity) / 340.0, CD, CL, CZ, Cl, Cm, Cn);

        double CD_ma = CD;
        double CL_ma = CL;
        double Cm_ma = Cm;

        double X_alpha  = (CD_alpha - CD0) / delta_angle * dynamic_pressure * _s;
        double Mz_alpha = (Cm_alpha - Cm0) / delta_angle * dynamic_pressure * _s * _l;
        double Y_alpha  = (CL_alpha - CL0) / delta_angle * dynamic_pressure * _s;

        double X_dz  = (CD_dz - CD0) / delta_angle * dynamic_pressure * _s;
        double Mz_dz = (Cm_dz - Cm0) / delta_angle * dynamic_pressure * _s * _l;
        double Y_dz  = (CL_dz - CL0) / delta_angle * dynamic_pressure * _s;

        double X_V  = (CD_ma - CD0) / delta_velocity * dynamic_pressure * _s;
        double Mz_V = (Cm_ma - Cm0) / delta_velocity * dynamic_pressure * _s * _l;
        double Y_V  = (CL_ma - CL0) / delta_velocity * dynamic_pressure * _s;

        double Z_beta  = (CZ_beta - CZ0) / delta_angle * dynamic_pressure * _s;
        double Z_dy    = (CZ_dy - CZ0) / delta_angle * dynamic_pressure * _s;
        double Mx_beta = (Cl_beta - Cl0) / delta_angle * dynamic_pressure * _s * _b;
        double My_beta = (Cn_beta - Cn0) / delta_angle * dynamic_pressure * _s * _l;
        double Mx_dx   = (Cl_dx - Cl0) / delta_angle * dynamic_pressure * _s * _b;
        double Mx_dy   = (Cl_dy - Cl0) / delta_angle * dynamic_pressure * _s * _b;
        double My_dy   = (Cn_dy - Cn0) / delta_angle * dynamic_pressure * _s * _l;

        derivative.a11 = (0 - X_V) / mass();
        derivative.a12 = 0;
        derivative.a13 = -9.8 * cos(theta);
        derivative.a14 = -(X_alpha + P() * alpha) / mass();
        derivative.a15 = -X_dz / mass();
        derivative.a16 = 1 / mass();
        derivative.a21 = Mz_V / _inertia(2, 2);
        derivative.a22 = -1.67 * dynamic_pressure * _s * _l * _l / _inertia(2, 2) / V();
        derivative.a23 = 0;
        derivative.a24 = Mz_alpha / _inertia(2, 2);
        derivative.a25 = Mz_dz / _inertia(2, 2);
        derivative.a26 = 1 / _inertia(2, 2);
        derivative.a31 = (0 + Y_V) / mass() / V();
        derivative.a32 = 0;
        derivative.a33 = 9.8 * sin(theta) / V();
        derivative.a34 = (P() + Y_alpha) / mass() / V();
        derivative.a35 = Y_dz / mass() / V();
        derivative.a36 = 1 / mass() / V();


        derivative.b11 = -0.05 * dynamic_pressure * _s * _b * _b / _inertia(0, 0) / V();
        derivative.b12 = 0;
        derivative.b13 = 0;
        derivative.b14 = Mx_beta / _inertia(0, 0);
        derivative.b15 = Mx_dy / _inertia(0, 0);
        derivative.b16 = 0;
        derivative.b17 = Mx_dx / _inertia(0, 0);
        derivative.b18 = 1 / _inertia(0, 0);

        derivative.b21 = 0;
        derivative.b22 = -1.67 * dynamic_pressure * _s * _l * _l / _inertia(1, 1) / V();
        derivative.b23 = 0;
        derivative.b24 = My_beta / _inertia(1, 1);
        derivative.b25 = My_dy / _inertia(1, 1);
        derivative.b26 = 0;
        derivative.b27 = 0;
        derivative.b28 = 1 / _inertia(1, 1);

        derivative.b31 = 0;
        derivative.b32 = -cos(theta) / cos(pitch);
        derivative.b33 = 0;
        derivative.b34 = (P() - Z_beta) / mass() / V();
        derivative.b35 = -Z_dy / mass() / V();
        derivative.b36 = -9.8 * cos(pitch) / V();
        derivative.b37 = 0;
        derivative.b38 = -1 / mass() / V();

        return derivative;
    }

    const char *Missile::phaseName() const {
        switch (_phase) {
            case GuidancePhase::Boost:
                return "boost";
            case GuidancePhase::Climb:
                return "climb";
            case GuidancePhase::Glide:
                return "glide";
            case GuidancePhase::Handover:
                return "handover";
            case GuidancePhase::Terminal:
                return "terminal";
            default:
                return "unknown";
        }
    }

// endregion
}
#undef PRETTY_FILE_NAME
