//
// Created by 17298 on 2026/4/22.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "TGC/FileSaver.h"
#include <filesystem>
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

    namespace {
        void write_route_marker_row(FILE *fp_traj, const Eigen::Vector3d &routePointNue) {
            fprintf(
                fp_traj,
                "-1.000000 %.6f %.6f %.6f nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan\n",
                routePointNue.x(),
                routePointNue.y(),
                routePointNue.z()
            );
        }
    }

// region USING/FRIEND
// endregion

    FileSaver::FileSaver(const std::string &path) {
        const std::filesystem::path outputPath(path);
        if (outputPath.is_absolute()) {
            _directory = outputPath.string();
        } else {
            const auto projectRoot = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
            _directory = (projectRoot / outputPath).lexically_normal().string();
        }
        if (!_directory.empty() && _directory.back() != '/' && _directory.back() != '\\') {
            _directory += std::filesystem::path::preferred_separator;
        }
    }

// region Constructor
    FileSaver::~FileSaver() {
        if (result_fp_traj() != nullptr) {
            fclose(result_fp_traj());
        }
        if (result_fp_aero() != nullptr) {
            fclose(result_fp_aero());
        }
    }

// endregion

// region Public Methods
    void FileSaver::save_traj(const Missile *missile) {
        save_route_points(missile);
        fprintf(
            result_fp_traj(),
            "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
            missile->flyTime(),
            missile->positionLaunchNUE().x(),
            missile->positionLaunchNUE().y(),
            missile->positionLaunchNUE().z(),
            missile->V(),
            missile->attitudeEuler().x(),
            missile->attitudeEuler().y(),
            missile->attitudeEuler().z(),
            missile->velocityTheta(),
            missile->velocityPsi(),
            missile->alpha(),
            missile->beta(),
            missile->accelerationBody().x(),
            missile->accelerationBody().y(),
            missile->accelerationBody().z(),
            missile->mass(),
            missile->P(),
            missile->acc_cmd_b_y(),
            missile->acc_cmd_b_z(),
            missile->targetPositionLaunchNUE().x(),
            missile->targetPositionLaunchNUE().y(),
            missile->targetPositionLaunchNUE().z(),
            missile->rudder().x(),
            missile->rudder().y(),
            missile->rudder().z(),
            missile->velocityNUE().x(),
            missile->velocityNUE().y(),
            missile->velocityNUE().z(),
            missile->w_xyz().x(),
            missile->w_xyz().y(),
            missile->w_xyz().z(),
            missile->lla().x(),
            missile->lla().y(),
            missile->lla().z(),
            missile->sigmaElv(),
            missile->sigmaAz(),
            missile->sigmaElvDot(),
            missile->sigmaAzDot()
        );
    }

    void FileSaver::save_aero(const Missile *missile) {
        const auto [a11, a12, a13, a14, a15, a16,
            a21, a22, a23, a24, a25, a26,
            a31, a32, a33, a34, a35, a36,
            b11, b12, b13, b14, b15, b16, b17, b18,
            b21, b22, b23, b24, b25, b26, b27, b28,
            b31, b32, b33, b34, b35, b36, b37, b38] = missile->derivative();

        fprintf(result_fp_aero(), "%.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f"
                "\n",
                missile->flyTime(),
                a11, a12, a13, a14, a15, a16,
                a21, a22, a23, a24, a25, a26,
                a31, a32, a33, a34, a35, a36,
                missile->V(),
                b11, b12, b13, b14, b15, b16, b17, b18,
                b21, b22, b23, b24, b25, b26, b27, b28,
                b31, b32, b33, b34, b35, b36, b37, b38
        );
    }

// endregion

// region Get/Set选择器
// endregion

// region Private Methods
    auto FileSaver::result_fp_aero() -> FILE * {
        if (!std::filesystem::exists(_directory)) {
            std::filesystem::create_directories(_directory);
        }
        std::string filename = _directory + "air.dat";
        if (!fp_aero) {
            fp_aero = fopen(filename.c_str(), "w");
            if (!fp_aero) {
                printf("错误：无法打开文件\n");
                exit(-1);
            }
        }
        return fp_aero;
    }

    auto FileSaver::result_fp_traj() -> FILE * {
        if (!std::filesystem::exists(_directory)) {
            std::filesystem::create_directories(_directory);
        }
        std::string filename = _directory + "result.dat";
        if (!fp_traj) {
            fp_traj = fopen(filename.c_str(), "w");
            if (!fp_traj) {
                printf("错误：无法打开文件\n");
                exit(-1);
            }
        }
        return fp_traj;
    }

    void FileSaver::save_route_points(const Missile *missile) {
        if (_routePointsSaved) {
            return;
        }

        const auto routePointsNue = missile->routePointsLaunchNUE();
        if (routePointsNue.empty()) {
            _routePointsSaved = true;
            return;
        }

        auto *fp_traj = result_fp_traj();
        for (const auto &routePointNue: routePointsNue) {
            write_route_marker_row(fp_traj, routePointNue);
        }
        _routePointsSaved = true;
    }

// endregion
}
#undef PRETTY_FILE_NAME
