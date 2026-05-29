//
// Created by MikuSoft on 2025/11/11.
// Copyright (c) 2025 JiuTianAoXiang All rights reserved.
//
// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "CoordinateHelper.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Utils/CoordinateHelper"
// endregion

// region Using NameSpace
// endregion

namespace ModelDevelop::Utils {
    Eigen::Vector3d CoordinateHelper::calculateGravity(const Eigen::Vector3d &position_ecf)  {
        const double r = position_ecf.norm();
        return -Constants::EARTH_GM / (r * r * r) * position_ecf;
    }

    Eigen::Vector3d CoordinateHelper::nueToEcefVector(const Eigen::Vector3d &nue_vec, const double _lon,
    const double _lat) {
        const double lat     = _lat * Constants::DEG_TO_RAD;
        const double lon     = _lon * Constants::DEG_TO_RAD;
        const double sin_lat = sin(lat);
        const double cos_lat = cos(lat);
        const double sin_lon = sin(lon);
        const double cos_lon = cos(lon);
        Eigen::Matrix3d R;
        R << -sin_lat * cos_lon, cos_lat * cos_lon, -sin_lon,
                -sin_lat * sin_lon, cos_lat * sin_lon, cos_lon,
                cos_lat, sin_lat, 0;
        return R * nue_vec;
    }

    Eigen::Vector3d CoordinateHelper::ecefToNueVector(const Eigen::Vector3d &ecef_vec, const double _lon,
    const double _lat) {
        const double lat     = _lat * Constants::DEG_TO_RAD;
        const double lon     = _lon * Constants::DEG_TO_RAD;
        const double sin_lat = sin(lat);
        const double cos_lat = cos(lat);
        const double sin_lon = sin(lon);
        const double cos_lon = cos(lon);
        Eigen::Matrix3d R;
        R << -sin_lat * cos_lon, cos_lat * cos_lon, -sin_lon,
                -sin_lat * sin_lon, cos_lat * sin_lon, cos_lon,
                cos_lat, sin_lat, 0;
        return R.transpose() * ecef_vec;
    }

    Eigen::Vector3d CoordinateHelper::ecefToEciVector(const Eigen::Vector3d &ecef_vec, const DateTime &dt) {
        const auto jul        = CJulian(dt);
        const double gmst     = jul.toGMST();
        const double cos_gmst = cos(gmst);
        const double sin_gmst = sin(gmst);
        Eigen::Vector3d eci;
        eci[0] = ecef_vec[0] * cos_gmst - ecef_vec[1] * sin_gmst;
        eci[1] = ecef_vec[0] * sin_gmst + ecef_vec[1] * cos_gmst;
        eci[2] = ecef_vec[2];
        return eci;
    }

    Eigen::Vector3d CoordinateHelper::eciToEcefVector(const Eigen::Vector3d &eci_vec, const DateTime &dt) {
        const auto jul        = CJulian(dt);
        const double gmst     = jul.toGMST();
        const double cos_gmst = cos(gmst);
        const double sin_gmst = sin(gmst);
        Eigen::Vector3d ecef;
        ecef[0] = eci_vec[0] * cos_gmst + eci_vec[1] * sin_gmst;
        ecef[1] = -eci_vec[0] * sin_gmst + eci_vec[1] * cos_gmst;
        ecef[2] = eci_vec[2];
        return ecef;
    }

    Eigen::Vector3d CoordinateHelper::bodyToNueVector(const Eigen::Vector3d &body_vec, const Eigen::Quaterniond &qbn) {
        const auto cbn = qbn.toRotationMatrix();
        return cbn * body_vec;
    }

    Eigen::Vector3d CoordinateHelper::nueToBodyVector(const Eigen::Vector3d &nue_vec, const Eigen::Quaterniond &qbn) {
        const auto cnb = qbn.toRotationMatrix().transpose().eval();
        return cnb * nue_vec;
    }

    Eigen::Vector3d CoordinateHelper::velocityToBodyVector(const Eigen::Vector3d &vec_vel, const double alpha,
    const double beta) {
        const Eigen::AngleAxisd beteAngle(beta, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd alphaAngle(alpha, Eigen::Vector3d::UnitZ());
        const Eigen::Quaterniond Qvb =  (beteAngle*alphaAngle).conjugate();
        const auto Cvb               = Qvb.toRotationMatrix();
        return Cvb * vec_vel;
    }

    Eigen::Vector3d CoordinateHelper::bodyToVelocityVector(const Eigen::Vector3d &vec_body, const double alpha,
    const double beta) {
        const Eigen::AngleAxisd beteAngle(beta, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd alphaAngle(alpha, Eigen::Vector3d::UnitZ());
        const Eigen::Quaterniond Qvb =  (beteAngle*alphaAngle).conjugate();
        auto Cvb = Qvb.toRotationMatrix();
        return Cvb.transpose() * vec_body;
    }

    Eigen::Vector3d CoordinateHelper::ecefToLla(const Eigen::Vector3d &ecef_pos) {
        const double P   = sqrt(pow(ecef_pos.x(), 2) + pow(ecef_pos.y(), 2));
        const double U   = atan(Constants::DEF_AL_2 * ecef_pos.z() / Constants::DEF_AL_3 / P);
        const double lon = atan2(ecef_pos.y(), ecef_pos.x()); //[-pi,pi]
        const double lat = atan((ecef_pos.z() + Constants::DEF_AL_3 * Constants::DEF_EP2_WGS84 * pow(sin(U), 3))
                                / (P - Constants::DEF_AL_2 * Constants::DEF_E2_WGS84 * pow(cos(U), 3))); //[-pi/2,pi/2]
        const double alt = P * cos(lat) + ecef_pos.z() * sin(lat) -
                           Constants::DEF_AL_2 * sqrt(1 - Constants::DEF_E2_WGS84 * pow(sin(lat), 2));
        return {lon / Constants::DEG_TO_RAD, lat / Constants::DEG_TO_RAD, alt};
    }

    Eigen::Vector3d CoordinateHelper::llaToEcef(const Eigen::Vector3d &lla)  {
        const double N = Constants::DEF_AL_2 / sqrt(
                             1 - Constants::DEF_E2_WGS84 * sin(lla.y() * Constants::DEG_TO_RAD) * sin(lla.y() * Constants::DEG_TO_RAD));
        return {
            (N + lla.z()) * cos(lla.y() * Constants::DEG_TO_RAD) * cos(lla.x() * Constants::DEG_TO_RAD),
            (N + lla.z()) * cos(lla.y() * Constants::DEG_TO_RAD) * sin(lla.x() * Constants::DEG_TO_RAD),
            (N * (1 - Constants::DEF_E2_WGS84) + lla.z()) * sin(lla.y() * Constants::DEG_TO_RAD)
        };
    }

    Eigen::Vector3d CoordinateHelper::eciToEcefPosition(const Eigen::Vector3d &eci_pos, const DateTime &dt) {
        const auto jul        = CJulian(dt);
        const double gmst     = jul.toGMST();
        const double cos_gmst = cos(gmst);
        const double sin_gmst = sin(gmst);

        Eigen::Vector3d ecef;
        ecef[0] = eci_pos[0] * cos_gmst + eci_pos[1] * sin_gmst;
        ecef[1] = -eci_pos[0] * sin_gmst + eci_pos[1] * cos_gmst;
        ecef[2] = eci_pos[2];

        return ecef;
    }

    Eigen::Vector3d CoordinateHelper::ecefToEciPosition(const Eigen::Vector3d &ecef_pos, const DateTime &dt)  {
        const auto jul        = CJulian(dt);
        const double gmst     = jul.toGMST();
        const double cos_gmst = cos(gmst);
        const double sin_gmst = sin(gmst);

        Eigen::Vector3d eci;
        eci[0] = ecef_pos[0] * cos_gmst - ecef_pos[1] * sin_gmst;
        eci[1] = ecef_pos[0] * sin_gmst + ecef_pos[1] * cos_gmst;
        eci[2] = ecef_pos[2];

        return eci;
    }

    Eigen::Vector3d CoordinateHelper::nueToEcefPosition(const Eigen::Vector3d &nue_pos, double lon, double lat){
        // 1. 计算参考点的ECEF坐标
        const Eigen::Vector3d ref_ecef = llaToEcef({lon, lat, 0});
        // 2. 将NUE位置向量转换到ECEF系
        const Eigen::Vector3d nue_offset_ecef = nueToEcefVector(nue_pos, lon, lat);
        // 3. 在ECEF系中相加得到目标点ECEF坐标
        return ref_ecef + nue_offset_ecef;
    }

    Eigen::Vector3d CoordinateHelper::ecefToNuePosition(const Eigen::Vector3d &ecef_pos, const double lon,
    const double lat) {
        // 1. 计算参考点的ECEF坐标
        const Eigen::Vector3d ref_ecef = llaToEcef({lon, lat, 0});
        // 2. 计算目标点相对于参考点的ECEF向量
        const Eigen::Vector3d rel_pos_ecef = (ecef_pos - ref_ecef).eval();
        // 3. 将相对位置向量转换到NUE系
        return ecefToNueVector(rel_pos_ecef, lon, lat);
    }

    Eigen::Vector3d CoordinateHelper::ecefToEciVelocity(const Eigen::Vector3d &ecef_vel,
    const Eigen::Vector3d &ecef_pos, const DateTime &dt)  {
        // ECI速度 = ECF速度 + ω × r
        const Eigen::Vector3d omega_earth(0, 0, Constants::EARTH_OMEGA);
        const Eigen::Vector3d earth_rotation_vel = omega_earth.cross(ecef_pos);

        const Eigen::Vector3d eci_vel = ecef_vel + earth_rotation_vel;
        // 旋转到ECI系
        return ecefToEciVector(eci_vel, dt);
    }

    Eigen::Vector3d CoordinateHelper::eciToEcefVelocity(const Eigen::Vector3d &eci_vel, const Eigen::Vector3d &eci_pos,
    const DateTime &dt)  {
        // 先旋转到ECF系
        const Eigen::Vector3d ecef_vel = eciToEcefVector(eci_vel, dt);
        // ECF速度 = ECI速度 - ω × r
        const Eigen::Vector3d ecef_pos = eciToEcefPosition(eci_pos, dt);
        const Eigen::Vector3d omega_earth(0, 0, Constants::EARTH_OMEGA);
        const Eigen::Vector3d earth_rotation_vel = omega_earth.cross(ecef_pos);
        return ecef_vel - earth_rotation_vel;
    }

    Eigen::Vector3d CoordinateHelper::nueToEcefVelocity(const Eigen::Vector3d &nue_vel, const double lon,
    const double lat)  {
        // 在ECF系中，速度转换是纯旋转变换
        // 不需要考虑地球自转效应，因为ECF系随地球一起旋转
        return nueToEcefVector(nue_vel, lon, lat);
    }

    Eigen::Vector3d CoordinateHelper::ecefToNueVelocity(const Eigen::Vector3d &ecef_vel, const double lon,
    const double lat) {
        // 在ECF系中，速度转换是纯旋转变换
        return ecefToNueVector(ecef_vel, lon, lat);
    }

    Eigen::Vector3d CoordinateHelper::
    bodyToNueVelocity(const Eigen::Vector3d &body_vel, const Eigen::Quaterniond &qbn) {
        // 速度是向量，直接进行旋转变换
        return bodyToNueVector(body_vel, qbn);
    }

    Eigen::Vector3d CoordinateHelper::nueToBodyVelocity(const Eigen::Vector3d &nue_vel, const Eigen::Quaterniond &qbn) {
        // 速度是向量，直接进行旋转变换
        return nueToBodyVector(nue_vel, qbn);
    }

    Eigen::Quaterniond CoordinateHelper::euler231ToQuaternion(double yaw, double pitch, double roll) {
        yaw   = yaw * Constants::DEG_TO_RAD;
        pitch = pitch * Constants::DEG_TO_RAD;
        roll  = roll * Constants::DEG_TO_RAD;
        const Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());//1
        const Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitZ());//2
        const Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());//0
        const Eigen::Quaterniond qnb = (yawAngle*pitchAngle*rollAngle).conjugate();
        auto qbn = qnb.normalized().conjugate();
        return qbn;
    }

    Eigen::Vector3d CoordinateHelper::quaternionToEuler231(const Eigen::Quaterniond &qbn) {
        auto Cbn           = qbn.toRotationMatrix();
        const double roll  = atan2(-Cbn(1, 2), Cbn(1, 1));
        const double yaw   = atan2(-Cbn(2, 0), Cbn(0, 0));
        const double pitch = asin(Cbn(1, 0));
        return {yaw * Constants::RAD_TO_DEG, pitch * Constants::RAD_TO_DEG, roll * Constants::RAD_TO_DEG};
    }

    void CoordinateHelper::calculateAngleOfAttack(const Eigen::Vector3d &velocity_nue,
    const Eigen::Quaterniond &attitude, double &alpha, double &beta) {
        // 1. 将NUE系速度转换到体坐标系
        Eigen::Vector3d velocityBody = nueToBodyVelocity(velocity_nue, attitude);


        const auto u                 = velocityBody.x();
        const auto v                 = velocityBody.y();
        const auto w                 = velocityBody.z();


        // 计算速度大小

        const double V = velocityBody.norm();
        const double V1 = velocity_nue.norm();

        if (V < 1e-12) {
            alpha = 0;
            beta  = 0;
            return;
        }


        alpha = std::atan2(-v, u);
        beta  = std::asin(w / V);
    }

    Eigen::Vector3d CoordinateHelper::nueToEcefAcceleration(const Eigen::Vector3d &nue_accel, const double lon,
    const double lat)  {
        // 在ECF系中，加速度转换是纯旋转变换
        // 不需要考虑科里奥利和离心加速度，因为ECF系是非惯性系
        return nueToEcefVector(nue_accel, lon, lat);
    }

    Eigen::Vector3d CoordinateHelper::ecefToNueAcceleration(const Eigen::Vector3d &ecef_accel, const double lon,
    const double lat)  {
        // 在ECF系中，加速度转换是纯旋转变换
        return ecefToNueVector(ecef_accel, lon, lat);
    }

    Eigen::Vector3d CoordinateHelper::bodyToNueAcceleration(const Eigen::Vector3d &body_accel,
    const Eigen::Quaterniond &qbn)  {
        // 加速度是向量，直接进行旋转变换
        return bodyToNueVector(body_accel, qbn);
    }

    Eigen::Vector3d CoordinateHelper::nueToBodyAcceleration(const Eigen::Vector3d &nue_accel,
    const Eigen::Quaterniond &qbn)  {
        // 加速度是向量，直接进行旋转变换
        return nueToBodyVector(nue_accel, qbn);
    }

    Eigen::Vector3d CoordinateHelper::velocityToBodyAcceleration(const Eigen::Vector3d &vel_accel, const double alpha,
    const double beta) {
        // 加速度是向量，直接进行旋转变换
        return velocityToBodyVector(vel_accel, alpha, beta);
    }

    Eigen::Vector3d CoordinateHelper::bodyToVelocityAcceleration(const Eigen::Vector3d &body_accel, const double alpha,
    const double beta)  {
        // 加速度是向量，直接进行旋转变换
        return bodyToVelocityVector(body_accel, alpha, beta);
    }

    Eigen::Vector3d CoordinateHelper::velocityToNue(const Eigen::Vector3d &vel_data, const Eigen::Vector3d &vel_nue) {
        const auto theta = getTheta(vel_nue);
        const auto psi = getPsi(vel_nue);
        Eigen::Matrix3d Cnv;
        Cnv<<
            cos(theta)*cos(psi),          sin(theta),               -cos(theta)*sin(psi),
            -sin(theta)*cos(psi),         cos(theta),                sin(theta)*sin(psi),
            sin(psi),                      0,                        cos(psi);
        return Cnv.transpose()*vel_data;
    }

    Eigen::Vector3d CoordinateHelper::nueToVelocity(const Eigen::Vector3d &nue_data, const Eigen::Vector3d &vel_nue) {
        const auto theta = getTheta(vel_nue);
        const auto psi = getPsi(vel_nue);
        Eigen::Matrix3d Cnv;
        Cnv<<
            cos(theta)*cos(psi),          sin(theta),               -cos(theta)*sin(psi),
            -sin(theta)*cos(psi),         cos(theta),                sin(theta)*sin(psi),
            sin(psi),                      0,                        cos(psi);
        return Cnv*nue_data;
    }

    double CoordinateHelper::getTheta(Eigen::Vector3d vec_nue) {
        const auto V = vec_nue.norm();
        return asin(vec_nue.y()/V);
    }

    double CoordinateHelper::getPsi(Eigen::Vector3d vec_nue) {
        return atan2(-vec_nue.z(),vec_nue.x());
    }
}
#undef PRETTY_FILE_NAME