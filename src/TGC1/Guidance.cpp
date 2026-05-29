//
// Created by 17298 on 2026/4/22.
//

// region Include
// region STL
#include <algorithm>
#include <cmath>
// endregion
// region ThirdParty
// endregion
// region Self
#include "TGC/Guidance.h"
#include "TGC/Seeker.h"
#include "Aerodynamics.h"
#include "CoordinateHelper.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    GCInfo Guidance::getMissionGCInfo(const double flyTime, const double P, const double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf,
                                      const State &state, const double maxLoad, const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex) {
        constexpr double safeSeparationTime = 0.6;
        constexpr double seekerAcquireDistance = 20000.0;
        constexpr double handoverEndDistance = 12000.0;
        constexpr double seekerHalfFov = 60.0 / 57.3;
        constexpr double climbThetaCmd = 18.0 / 57.3;

        GCInfo gc_info;
        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto selfVel_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double theta = ModelDevelop::Utils::CoordinateHelper::getTheta(selfVel_nue);
        const double psi = ModelDevelop::Utils::CoordinateHelper::getPsi(selfVel_nue);
        const double target_dis = (targetPosEcf - state.posEcf).norm();
        (void) waypoints;
        (void) currentWpIndex;

        const auto los_midcourse = getLOSInfo(targetPosEcf, targetVelEcf, state);
        const auto los_terminal = _seeker.getLOSInfo(targetPosEcf, targetVelEcf, state);
        const bool seekerInFov = std::abs(los_terminal.sigma_az_b) <= seekerHalfFov && std::abs(los_terminal.sigma_elv_b) <= seekerHalfFov;
        //const bool seekerLocked = target_dis <= seekerAcquireDistance && seekerInFov;
        const bool seekerLocked = target_dis <= seekerAcquireDistance;

        Eigen::Vector3d acc_cmd_v = Eigen::Vector3d::Zero();
        double handoverRatio = 0.0;

        if (flyTime < safeSeparationTime) {
            gc_info.phase = GuidancePhase::Boost;
            gc_info.losInfo = los_midcourse;
        } else if (P > 1.0e3) {
            gc_info.phase = GuidancePhase::Climb;
            gc_info.losInfo = los_midcourse;

            const double desired_h = 10000.0;
            const double heading_error = wrapAngle(los_midcourse.sigma_az - psi);
            const double lateral_acc = 2.0 * selfVel_nue.norm() * selfVel_nue.norm() * std::sin(heading_error) / std::max(target_dis, 1000.0);

            const double theta_error = climbThetaCmd - theta;
            const double altitude_error = desired_h - lla.z();
            acc_cmd_v.y() = 9.8 * std::cos(theta) + 0.015 * altitude_error + 1.8 * selfVel_nue.norm() * theta_error - 0.45 * selfVel_nue.y();
            acc_cmd_v.z() = lateral_acc;
        } else if (!seekerLocked) {
            gc_info = getGCInfoAnalyticMidcourse(flyTime, targetPosEcf, targetVelEcf, state, Mass, maxLoad);
            return gc_info;
        } else {
            const auto terminal_acc = guidance_pn(theta, los_terminal.sigma_az_dot, los_terminal.sigma_elv_dot, los_terminal.dis_dot);
            gc_info.losInfo = los_terminal;

            if (seekerLocked && target_dis > handoverEndDistance) {
                auto routeInfo = getGCInfoAnalyticMidcourse(flyTime, targetPosEcf, targetVelEcf, state, Mass, maxLoad);
                handoverRatio = computeBlendRatio(target_dis, seekerAcquireDistance, handoverEndDistance);
                acc_cmd_v = (1.0 - handoverRatio) * routeInfo.acc_cmd_v + handoverRatio * terminal_acc;
                gc_info.phase = GuidancePhase::Handover;
            } else {
                acc_cmd_v = terminal_acc;
                gc_info.phase = GuidancePhase::Terminal;
            }
        }

        acc_cmd_v.y() = clamp(acc_cmd_v.y(), -9.8 * maxLoad, 9.8 * maxLoad);
        acc_cmd_v.z() = clamp(acc_cmd_v.z(), -9.8 * maxLoad, 9.8 * maxLoad);
        gc_info.acc_cmd_v = acc_cmd_v;
        gc_info.handoverRatio = handoverRatio;
        return gc_info;
    }

    LosInfo Guidance::getLOSInfo(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state) {
        const Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);

        const auto target_position_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, lla.x(), lla.y());
        const auto position_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(state.posEcf, lla.x(), lla.y());
        const auto target_velocity_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(targetVelEcf, lla.x(), lla.y());
        const auto velocity_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());

        const auto rel_pos = (target_position_nue - position_nue).eval();
        const auto rel_vel = (target_velocity_nue - velocity_nue).eval();
        const auto rel_w = (rel_pos.cross(rel_vel) / rel_pos.squaredNorm()).eval();
        const auto dis = (target_position_nue - position_nue).norm();

        const auto theta = ModelDevelop::Utils::CoordinateHelper::getTheta(velocity_nue);
        const auto psi = ModelDevelop::Utils::CoordinateHelper::getPsi(velocity_nue);

        const auto sigma_az_dot = -rel_w.x() * std::sin(theta) * std::cos(psi) + rel_w.y() * std::cos(theta) + rel_w.z() * std::sin(theta) * std::sin(psi);
        const auto sigma_elv_dot = rel_w.x() * std::sin(psi) + rel_w.z() * std::cos(psi);
        const auto dis_dot = rel_pos.dot(rel_vel) / dis;
        const auto rel_pos_body = ModelDevelop::Utils::CoordinateHelper::nueToBodyVector(rel_pos, state.qbn);

        LosInfo los_info{};
        los_info.dis_dot = dis_dot;
        los_info.sigma_az_dot = sigma_az_dot;
        los_info.sigma_elv_dot = sigma_elv_dot;
        los_info.sigma_elv = ModelDevelop::Utils::CoordinateHelper::getTheta(rel_pos);
        los_info.sigma_az = ModelDevelop::Utils::CoordinateHelper::getPsi(rel_pos);
        los_info.sigma_elv_b = std::atan2(rel_pos_body.y(), rel_pos_body.x());
        los_info.sigma_az_b = std::atan2(-rel_pos_body.z(), rel_pos_body.x());
        return los_info;
    }

    GCInfo Guidance::getGCInfoRouteL1(const State &state, const double maxLoad, const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex) {
        auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto selfVel_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double theta = Utils::CoordinateHelper::getTheta(selfVel_nue);
        Eigen::Vector3d acc_cmd_v = {0, 0, 0};
        const double Vy = selfVel_nue.y();

        auto [acc_cmd_vz, desired_h] = calculateL1Guidance(maxLoad, state.posEcf, state.velEcf, waypoints, currentWpIndex);
        lastDesiredH = lastDesiredH + 0.001 * (desired_h - lastDesiredH);
        acc_cmd_v.z() = acc_cmd_vz;
        acc_cmd_v.y() = 9.8 * std::cos(theta) + 0.2 * (desired_h - lla.z()) - 2 * 4 * 0.2 * Vy;

        if (acc_cmd_v.y() > 9.8 * maxLoad)
            acc_cmd_v.y() = 9.8 * maxLoad;
        if (acc_cmd_v.y() < -9.8 * maxLoad)
            acc_cmd_v.y() = -9.8 * maxLoad;
        if (acc_cmd_v.z() > 9.8 * maxLoad)
            acc_cmd_v.z() = 9.8 * maxLoad;
        if (acc_cmd_v.z() < -9.8 * maxLoad)
            acc_cmd_v.z() = -9.8 * maxLoad;
        GCInfo gc_info;
        gc_info.acc_cmd_v = acc_cmd_v;
        return gc_info;
    }

    std::pair<double, double> Guidance::calculateL1Guidance(const double maxLoad, const Eigen::Vector3d &currentPosEcf, const Eigen::Vector3d &currentVelEcf,
                                                            const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex) {
        if (waypoints.empty() || currentWpIndex < 0 || currentWpIndex >= static_cast<int>(waypoints.size()) - 1) {
            currentWpIndex = -1;
            return {0.0, 0.0};
        }

        const auto &wp_start_lla = waypoints[currentWpIndex];
        const auto &wp_end_lla = waypoints[currentWpIndex + 1];
        Eigen::Vector3d wp_start_ecf = Utils::CoordinateHelper::llaToEcef(wp_start_lla);
        Eigen::Vector3d wp_end_ecf = Utils::CoordinateHelper::llaToEcef(wp_end_lla);

        Eigen::Vector3d pos_nue = Utils::CoordinateHelper::ecefToNuePosition(currentPosEcf, wp_start_lla.x(), wp_start_lla.y()) -
                                  Utils::CoordinateHelper::ecefToNuePosition(wp_start_ecf, wp_start_lla.x(), wp_start_lla.y());
        Eigen::Vector3d wp_start_nue(0, 0, 0);
        Eigen::Vector3d wp_end_nue = Utils::CoordinateHelper::ecefToNuePosition(wp_end_ecf, wp_start_lla.x(), wp_start_lla.y()) -
                                     Utils::CoordinateHelper::ecefToNuePosition(wp_start_ecf, wp_start_lla.x(), wp_start_lla.y());

        Eigen::Vector3d vel_nue3 = Utils::CoordinateHelper::ecefToNueVelocity(currentVelEcf, wp_start_lla.x(), wp_start_lla.y());
        Eigen::Vector2d vel_nue(vel_nue3.x(), vel_nue3.z());

        double speed = vel_nue.norm();
        double R_min = (speed * speed) / (maxLoad * 9.8);
        double L1_distance = std::max(3.0 * R_min, 1000.0);

        Eigen::Vector2d pos_2d(pos_nue[0], pos_nue[2]);
        Eigen::Vector2d wp_start_2d(wp_start_nue[0], wp_start_nue[2]);
        Eigen::Vector2d wp_end_2d(wp_end_nue[0], wp_end_nue[2]);

        double acc = calculateL1GuidanceNUE(pos_2d, vel_nue, wp_start_2d, wp_end_2d, L1_distance);

        if (currentWpIndex <= static_cast<int>(waypoints.size()) - 2) {
            Eigen::Vector2d to_end = wp_end_2d - pos_2d;
            double dist_to_end = to_end.norm();
            if (dist_to_end < std::max(0.5 * L1_distance, 1000.0)) {
                currentWpIndex++;
                if (currentWpIndex >= static_cast<int>(waypoints.size()) - 1) {
                    currentWpIndex = -1;
                }
            }
        }

        return {acc, wp_end_lla.z()};
    }

    double Guidance::calculateL1GuidanceNUE(const Eigen::Vector2d &pos_nue, const Eigen::Vector2d &vel_nue, const Eigen::Vector2d &wp_start, const Eigen::Vector2d &wp_end,
                                            double L1_distance) {
        Eigen::Vector2d segment = wp_end - wp_start;
        double seg_length = segment.norm();
        if (seg_length < 1e-6) {
            return 0.0;
        }

        Eigen::Vector2d seg_unit = segment / seg_length;
        Eigen::Vector2d rel_pos = pos_nue - wp_start;
        double s = rel_pos.dot(seg_unit);
        Eigen::Vector2d proj_point = wp_start + seg_unit * s;
        double s_L1 = s + L1_distance;

        Eigen::Vector2d L1_point = s_L1 > seg_length ? wp_end : wp_start + seg_unit * s_L1;
        Eigen::Vector2d vec_to_L1 = L1_point - pos_nue;
        double dist_to_L1 = vec_to_L1.norm();
        if (dist_to_L1 < 1e-6) {
            return 0.0;
        }

        double V = vel_nue.norm();
        if (V < 1e-6) {
            return 0.0;
        }

        double cross_y = vel_nue[0] * vec_to_L1[1] - vel_nue[1] * vec_to_L1[0];
        double sin_eta = cross_y / (V * dist_to_L1);
        return 2.0 * V * V * sin_eta / dist_to_L1;
    }

    Eigen::Vector3d Guidance::guidance_pn(const double theta, const double sigma_az_dot, const double sigma_elv_dot, const double dis_dot) {
        constexpr double K = 4;
        constexpr double gravity = 9.8;
        const auto ny_tc = K * std::fabs(dis_dot) * sigma_elv_dot + gravity * std::cos(theta);
        const auto nz_tc = -K * std::fabs(dis_dot) * sigma_az_dot;
        return Eigen::Vector3d(0, ny_tc, nz_tc);
    }

    double Guidance::wrapAngle(double angle) {
        while (angle > 3.14159265358979323846) {
            angle -= 2.0 * 3.14159265358979323846;
        }
        while (angle < -3.14159265358979323846) {
            angle += 2.0 * 3.14159265358979323846;
        }
        return angle;
    }

    double Guidance::clamp(const double value, const double minValue, const double maxValue) {
        return std::max(minValue, std::min(value, maxValue));
    }

    double Guidance::computeBlendRatio(const double targetDistance, const double startDistance, const double endDistance) {
        if (startDistance <= endDistance) {
            return 1.0;
        }
        return clamp((startDistance - targetDistance) / (startDistance - endDistance), 0.0, 1.0);
    }

    GCInfo Guidance::getGCInfoAnalyticGlide(const double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, const double maxLoad) {
        return getGCInfoAnalyticMidcourse(0.0, targetPosEcf, targetVelEcf, state, Mass, maxLoad);
    }

    GCInfo Guidance::getGCInfoAnalyticMidcourse(const double flyTime, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, const double mass,
                                                const double maxLoad) {
        (void) flyTime;

        constexpr double gravity = 9.80665;
        constexpr double minTerminalSpeed = 900.0;
        constexpr double maxTerminalSpeed = 2200.0;
        constexpr double minDrag = 0.05;
        constexpr double scaleHeight = 7200.0;

        GCInfo gc_info;
        gc_info.phase = GuidancePhase::Glide;
        gc_info.losInfo = getLOSInfo(targetPosEcf, targetVelEcf, state);

        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto targetLla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(targetPosEcf);
        const auto velNue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const auto relNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, lla.x(), lla.y()) -
                            ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(state.posEcf, lla.x(), lla.y());

        const double speed = std::max(velNue.norm(), 1.0);
        const double rangeToGo = std::max(Eigen::Vector2d(relNue.x(), relNue.z()).norm(), 1000.0);
        const double theta = ModelDevelop::Utils::CoordinateHelper::getTheta(velNue);
        const double psi = ModelDevelop::Utils::CoordinateHelper::getPsi(velNue);
        const double headingError = wrapAngle(gc_info.losInfo.sigma_az - psi);

        const double terminalSpeedUpper = std::max(minTerminalSpeed, std::min(maxTerminalSpeed, speed - 10.0));
        const double terminalSpeed = clamp(targetVelEcf.norm() > 100.0 ? targetVelEcf.norm() : speed * 0.65, minTerminalSpeed, terminalSpeedUpper);
        const double currentDrag = std::max(estimateDragAcceleration(state, mass), minDrag);
        const double terminalRho = ModelDevelop::Utils::Aerodynamics::calculateAtmosphereDensity(std::max(targetLla.z(), 0.0));
        const double terminalDrag = std::max(0.5 * terminalRho * terminalSpeed * terminalSpeed * 0.223 * 0.08 / std::max(mass, 1.0), minDrag);
        const double dc = solveDragProfileMidpoint(speed, terminalSpeed, currentDrag, terminalDrag, rangeToGo);
        const double refDrag = dragProfile(speed, speed, terminalSpeed, currentDrag, dc, terminalDrag);
        const double refDragSlope = dragProfileSlope(speed, speed, terminalSpeed, currentDrag, dc, terminalDrag);
        const double refDragDot = -refDragSlope * refDrag;

        const double refRho = clamp(2.0 * std::max(mass, 1.0) * refDrag / (speed * speed * 0.223 * 0.08), 1.0e-6, 1.225);
        const double dragAltitudeRef = -scaleHeight * std::log(refRho / 1.225);
        const double lineAltitudeRef = targetLla.z() + (lla.z() - targetLla.z()) * clamp(rangeToGo / std::max(rangeToGo + 50000.0, 1.0), 0.0, 1.0);
        const double altitudeRef = 0.65 * dragAltitudeRef + 0.35 * lineAltitudeRef;

        const double altitudeError = clamp(altitudeRef - lla.z(), -20000.0, 20000.0);
        const double dragError = clamp(currentDrag - refDrag, -20.0, 20.0);
        const double thetaRef = clamp(std::atan2(targetLla.z() - lla.z(), rangeToGo) + 0.018 * dragError, -25.0 / 57.3, 15.0 / 57.3);
        const double thetaDotCmd = 0.75 * wrapAngle(thetaRef - theta) + 0.00035 * altitudeError - 0.0005 * velNue.y() - 0.015 * refDragDot;

        const double liftAcc = estimateLiftAcceleration(state, mass, maxLoad);
        const double verticalAcc = clamp(gravity * std::cos(theta) + speed * thetaDotCmd, -gravity * maxLoad, gravity * maxLoad);
        const double cosSigma = clamp((verticalAcc - gravity * std::cos(theta)) / std::max(liftAcc, gravity), -1.0, 1.0);
        const double bankMagnitude = std::acos(cosSigma);

        const double corridor = clamp((3.0 + 17.0 * rangeToGo / 200000.0) / 57.3, 2.0 / 57.3, 25.0 / 57.3);
        if (std::fabs(headingError) >= corridor) {
            lastBankSign = guidanceHeadingSign(headingError, corridor);
        }
        const double sign = std::fabs(headingError) < 0.15 * corridor ? 0.0 : lastBankSign;
        const double headingRatio = clamp(std::fabs(headingError) / corridor, 0.0, 1.0);
        const double lateralAcc = sign * headingRatio * std::min(liftAcc * std::sin(bankMagnitude), gravity * maxLoad);

        gc_info.acc_cmd_v = Eigen::Vector3d(0.0, verticalAcc, clamp(lateralAcc, -gravity * maxLoad, gravity * maxLoad));
        return gc_info;
    }

    double Guidance::estimateDragAcceleration(const State &state, const double mass) const {
        constexpr double referenceArea = 0.223;
        double alpha = 0.0;
        double beta = 0.0;
        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto velNue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        ModelDevelop::Utils::CoordinateHelper::calculateAngleOfAttack(velNue, state.qbn, alpha, beta);

        const double speed = velNue.norm();
        const double ma = std::max(speed / av, 0.1);
        const double ca0 = 0.03 + 0.0005 * ma;
        const double k = 0.8 + 0.0005 * ma;
        const double cd = ca0 + k * (alpha * alpha + beta * beta);
        const double rho = ModelDevelop::Utils::Aerodynamics::calculateAtmosphereDensity(lla.z());
        return 0.5 * rho * speed * speed * referenceArea * cd / std::max(mass, 1.0);
    }

    double Guidance::estimateLiftAcceleration(const State &state, const double mass, const double maxLoad) const {
        constexpr double referenceArea = 0.223;
        constexpr double maxAlpha = 20.0 / 57.3;
        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto velNue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double speed = velNue.norm();
        const double ma = std::max(speed / av, 0.1);
        const double cn = 0.3 + 0.6 * ma * ma / (1.0 + 0.8 * ma * ma * ma * ma) + 4.0 / std::sqrt(1.0 + (ma * ma - 1.0) * (ma * ma - 1.0));
        const double rho = ModelDevelop::Utils::Aerodynamics::calculateAtmosphereDensity(lla.z());
        const double availableLift = 0.5 * rho * speed * speed * referenceArea * cn * maxAlpha / std::max(mass, 1.0);
        return clamp(availableLift, 9.80665, 9.80665 * maxLoad);
    }

    double Guidance::solveDragProfileMidpoint(const double v0, const double vf, const double d0, const double df, const double rangeToGo) const {
        double dc = std::max(0.5 * (d0 + df), 0.05);
        for (int i = 0; i < 8; ++i) {
            const double range = profileRange(v0, vf, d0, dc, df);
            const double delta = std::max(0.02 * dc, 0.01);
            const double gradient = (profileRange(v0, vf, d0, dc + delta, df) - range) / delta;
            if (std::fabs(gradient) < 1.0e-6) {
                break;
            }
            dc = clamp(dc + 0.5 * (rangeToGo - range) / gradient, 0.05, 80.0);
        }
        return dc;
    }

    double Guidance::dragProfile(const double v, const double v0, const double vf, const double d0, const double dc, const double df) const {
        const double vm = 0.5 * (v0 + vf);
        const double l0 = (v - vm) * (v - vf) / ((v0 - vm) * (v0 - vf));
        const double l1 = (v - v0) * (v - vf) / ((vm - v0) * (vm - vf));
        const double l2 = (v - v0) * (v - vm) / ((vf - v0) * (vf - vm));
        return std::max(0.05, d0 * l0 + dc * l1 + df * l2);
    }

    double Guidance::dragProfileSlope(const double v, const double v0, const double vf, const double d0, const double dc, const double df) const {
        constexpr double dv = 1.0;
        return (dragProfile(v + dv, v0, vf, d0, dc, df) - dragProfile(v - dv, v0, vf, d0, dc, df)) / (2.0 * dv);
    }

    double Guidance::profileRange(const double v0, const double vf, const double d0, const double dc, const double df) const {
        if (v0 <= vf + 1.0) {
            return 0.0;
        }
        constexpr int segments = 64;
        const double dv = (v0 - vf) / segments;
        double range = 0.0;
        for (int i = 0; i < segments; ++i) {
            const double va = v0 - i * dv;
            const double vb = va - dv;
            const double vm = 0.5 * (va + vb);
            range += vm / dragProfile(vm, v0, vf, d0, dc, df) * dv;
        }
        return range;
    }

    double Guidance::guidanceHeadingSign(const double headingError, const double corridor) {
        if (std::fabs(headingError) < 0.15 * corridor) {
            return 0.0;
        }
        return headingError > 0.0 ? 1.0 : -1.0;
    }

    void Guidance::Lambert_Resolve_Dv1(const Eigen::Vector3d &r_m, const Eigen::Vector3d &r_pip, double &T_pip, double vd_m[3], double &Range) const {
        double v_m[3];

        double gamma_min, gamma_max, gamma0;
        double V0, lambda, temp;
        Eigen::Vector3d i_vec, j_vec, Temp_Vec;

        double t0 = 0;
        double R_m = r_m.norm();
        double R_pip = r_pip.norm();
        double R_m_pip = r_m.dot(r_pip);
        double theta_f = acos(R_m_pip / R_m / R_pip);

        Range = theta_f * earth_ae / 1000;
        double Ve = sqrt(2 * c_dMiu / R_m);
        if (Range < 9000) {
            gamma_min = atan((cos(theta_f) - R_m / R_pip) / sin(theta_f));
            gamma_max = atan((sin(theta_f) + sqrt((1 - cos(theta_f)) * 2 * R_m / R_pip)) / (1 - cos(theta_f)));

            gamma0 = (gamma_min + gamma_max) / 2;
            V0 = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m / (R_m * (cos(gamma0) * cos(gamma0)) - R_pip * cos(theta_f + gamma0) * cos(gamma0));
            V0 = sqrt(V0);
            lambda = R_m * V0 * V0 / c_dMiu;

            if ((lambda > 0) && (lambda < 2)) {
                t0 = (tan(gamma0) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
                t0 = t0 + 2 * cos(gamma0) * atan(sqrt(2 / lambda - 1) / (cos(gamma0) * (1 / tan(theta_f / 2)) - sin(gamma0))) / lambda / pow((2 / lambda - 1), 1.5);
                t0 = t0 * R_m / V0 / cos(gamma0);
            }

            i_vec = r_m / R_m;
            Temp_Vec = r_m.cross(r_pip);
            j_vec = Temp_Vec.cross(r_m);
            temp = j_vec.norm();
            j_vec = j_vec / temp;

            vd_m[0] = V0 * sin(gamma0) * i_vec[0] + V0 * cos(gamma0) * j_vec[0];
            vd_m[1] = V0 * sin(gamma0) * i_vec[1] + V0 * cos(gamma0) * j_vec[1];
            vd_m[2] = V0 * sin(gamma0) * i_vec[2] + V0 * cos(gamma0) * j_vec[2];
            v_m[0] = vd_m[0];
            v_m[1] = vd_m[1];
            v_m[2] = vd_m[2];
            T_pip = t0;
        } else {
            T_pip = 2500 - (13000 - Range) / 7;

            double mask, t_delt, kesi, gamma_d, Vd;
            int n;
            double gamma[1000], t_ff[1000], V[1000];

            kesi = 0.001;
            mask = 0;
            gamma_d = 0;
            Vd = 0;
            gamma_min = atan((cos(theta_f) - R_m / R_pip) / sin(theta_f));
            gamma_max = atan((sin(theta_f) + sqrt((1 - cos(theta_f)) * 2 * R_m / R_pip)) / (1 - cos(theta_f)));

            gamma0 = (gamma_min + gamma_max) / 2;
            V0 = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m / (R_m * (cos(gamma0) * cos(gamma0)) - R_pip * cos(theta_f + gamma0) * cos(gamma0));
            V0 = sqrt(V0);
            lambda = R_m * V0 * V0 / c_dMiu;

            if ((lambda > 0) && (lambda < 2)) {
                t0 = (tan(gamma0) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
                t0 = t0 + 2 * cos(gamma0) * atan(sqrt(2 / lambda - 1) / (cos(gamma0) * (1 / tan(theta_f / 2)) - sin(gamma0))) / lambda / pow((2 / lambda - 1), 1.5);
                t0 = t0 * R_m / V0 / cos(gamma0);
            }
            if (t0 > T_pip) {
                gamma[0] = (gamma_min + gamma0) / 2;
                mask = 1;
            } else {
                gamma[0] = (gamma_max + gamma0) / 2;
                mask = -1;
            }
            t_delt = T_pip;
            n = 0;
            while (abs(t_delt) > kesi) {
                V[n] = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m /
                       (R_m * (cos(gamma[n]) * cos(gamma[n])) - R_pip * cos(theta_f + gamma[n]) * cos(gamma[n]));
                V[n] = sqrt(V[n]);
                lambda = R_m * V[n] * V[n] / c_dMiu;
                if (lambda > 0 && lambda < 2) {
                    t_ff[n] = (tan(gamma[n]) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
                    t_ff[n] = t_ff[n] + 2 * cos(gamma[n]) * atan(sqrt(2 / lambda - 1) / (cos(gamma[n]) * (1 / tan(theta_f / 2)) - sin(gamma[n]))) /
                              lambda / pow((2 / lambda - 1), 1.5);
                    t_ff[n] = t_ff[n] * R_m / V[n] / cos(gamma[n]);
                } else {
                    t_ff[n] = t_ff[n - 1];
                }
                t_delt = T_pip - t_ff[n];
                if (abs(t_delt) > kesi) {
                    if (n == 0) {
                        gamma[n + 1] = gamma[n] + (gamma[n] - gamma0) * (T_pip - t_ff[n]) / (t_ff[n] - t0);
                    } else {
                        gamma[n + 1] = gamma[n] + (gamma[n] - gamma[n - 1]) * (T_pip - t_ff[n]) / (t_ff[n] - t_ff[n - 1]);
                    }
                    if ((gamma[n + 1] < gamma_min) || (gamma[n + 1] > gamma_max)) {
                        if (mask == 1) {
                            if (n == 0) {
                                gamma[n + 1] = (std::min(gamma[n], gamma0) + gamma_min) / 2;
                            } else {
                                gamma[n + 1] = (std::min(gamma[n], gamma[n - 1]) + gamma_min) / 2;
                            }
                        } else {
                            if (n == 0) {
                                gamma[n + 1] = (std::max(gamma[n], gamma0) + gamma_max) / 2;
                            } else {
                                gamma[n + 1] = (std::max(gamma[n], gamma[n - 1]) + gamma_max) / 2;
                            }
                        }
                    }
                } else {
                    gamma_d = gamma[n];
                    Vd = V[n];
                }

                n = n + 1;
            }
            i_vec = r_m / R_m;
            Temp_Vec = r_m.cross(r_pip);
            j_vec = Temp_Vec.cross(r_m);
            temp = j_vec.norm();
            j_vec = j_vec / temp;

            vd_m[0] = Vd * sin(gamma_d) * i_vec[0] + Vd * cos(gamma_d) * j_vec[0];
            vd_m[1] = Vd * sin(gamma_d) * i_vec[1] + Vd * cos(gamma_d) * j_vec[1];
            vd_m[2] = Vd * sin(gamma_d) * i_vec[2] + Vd * cos(gamma_d) * j_vec[2];

            v_m[0] = vd_m[0];
            v_m[1] = vd_m[1];
            v_m[2] = vd_m[2];
        }
    }

    void Guidance::Lambert_Resolve_Dv(const Eigen::Vector3d &r_m, const Eigen::Vector3d &r_pip, double &T_pip, double vd_m[3], double &Range) const {
        double v_m[3];
        Eigen::Vector3d i_vec, j_vec, Temp_Vec;

        double t0 = 0;
        double R_m = r_m.norm();
        double R_pip = r_pip.norm();
        double R_m_pip = r_m.dot(r_pip);
        double theta_f = acos(R_m_pip / R_m / R_pip);

        Range = theta_f * earth_ae / 1000;
        double Ve = sqrt(2 * c_dMiu / R_m);

        double gamma[10000], t_ff[10000], V[10000];

        double kesi = 0.0001;
        double mask = 0;
        double gamma_d = 0;
        double Vd = 0;
        double gamma_min = atan((cos(theta_f) - R_m / R_pip) / sin(theta_f));
        double gamma_max = atan((sin(theta_f) + sqrt((1 - cos(theta_f)) * 2 * R_m / R_pip)) / (1 - cos(theta_f)));

        double gamma0 = (gamma_min + gamma_max) / 2;
        double V0 = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m /
                    (R_m * (cos(gamma0) * cos(gamma0)) - R_pip * cos(theta_f + gamma0) * cos(gamma0));
        V0 = sqrt(V0);
        double lambda = R_m * V0 * V0 / c_dMiu;

        if ((lambda > 0) && (lambda < 2)) {
            t0 = (tan(gamma0) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
            t0 = t0 + 2 * cos(gamma0) * atan(sqrt(2 / lambda - 1) / (cos(gamma0) * (1 / tan(theta_f / 2)) - sin(gamma0))) / lambda / pow((2 / lambda - 1), 1.5);
            t0 = t0 * R_m / V0 / cos(gamma0);
        }
        if (t0 > T_pip) {
            gamma[0] = (gamma_min + gamma0) / 2;
            mask = 1;
        } else {
            gamma[0] = (gamma_max + gamma0) / 2;
            mask = -1;
        }
        double t_delt = T_pip;
        int n = 0;
        while (abs(t_delt) > kesi) {
            V[n] = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m / (R_m * (cos(gamma[n]) * cos(gamma[n])) - R_pip * cos(theta_f + gamma[n]) * cos(gamma[n]));
            V[n] = sqrt(V[n]);
            lambda = R_m * V[n] * V[n] / c_dMiu;
            if (lambda > 0 && lambda < 2) {
                t_ff[n] = (tan(gamma[n]) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
                t_ff[n] = t_ff[n] + 2 * cos(gamma[n]) * atan(sqrt(2 / lambda - 1) / (cos(gamma[n]) * (1 / tan(theta_f / 2)) - sin(gamma[n]))) /
                          lambda / pow((2 / lambda - 1), 1.5);
                t_ff[n] = t_ff[n] * R_m / V[n] / cos(gamma[n]);
            } else {
                t_ff[n] = t_ff[n - 1];
                Vd = 12e3;
                break;
            }

            t_delt = T_pip - t_ff[n];
            if (abs(t_delt) > kesi) {
                if (n == 0) {
                    gamma[n + 1] = gamma[n] + (gamma[n] - gamma0) * (T_pip - t_ff[n]) / (t_ff[n] - t0);
                } else {
                    gamma[n + 1] = gamma[n] + (gamma[n] - gamma[n - 1]) * (T_pip - t_ff[n]) / (t_ff[n] - t_ff[n - 1]);
                }
                if ((gamma[n + 1] < gamma_min) || (gamma[n + 1] > gamma_max)) {
                    if (mask == 1) {
                        if (n == 0) {
                            gamma[n + 1] = (std::min(gamma[n], gamma0) + gamma_min) / 2;
                        } else {
                            gamma[n + 1] = (std::min(gamma[n], gamma[n - 1]) + gamma_min) / 2;
                        }
                    } else {
                        if (n == 0) {
                            gamma[n + 1] = (std::max(gamma[n], gamma0) + gamma_max) / 2;
                        } else {
                            gamma[n + 1] = (std::max(gamma[n], gamma[n - 1]) + gamma_max) / 2;
                        }
                    }
                }
            } else {
                gamma_d = gamma[n];
                Vd = V[n];
            }
            n = n + 1;
        }

        i_vec = r_m / R_m;
        Temp_Vec = r_m.cross(r_pip);
        j_vec = Temp_Vec.cross(r_m);
        double temp = j_vec.norm();
        j_vec = j_vec / temp;

        vd_m[0] = Vd * sin(gamma_d) * i_vec[0] + Vd * cos(gamma_d) * j_vec[0];
        vd_m[1] = Vd * sin(gamma_d) * i_vec[1] + Vd * cos(gamma_d) * j_vec[1];
        vd_m[2] = Vd * sin(gamma_d) * i_vec[2] + Vd * cos(gamma_d) * j_vec[2];

        v_m[0] = vd_m[0];
        v_m[1] = vd_m[1];
        v_m[2] = vd_m[2];
    }
}

#undef PRETTY_FILE_NAME
