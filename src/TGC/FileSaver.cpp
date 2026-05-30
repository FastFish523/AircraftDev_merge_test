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
#include "TGC/Engine.h"
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

    FileSaver::FileSaver(const std::string &path) {
        _directory = path;
    }

// region Constructor
    FileSaver::~FileSaver() {
        if (result_fp_traj() != nullptr) {
            fclose(result_fp_traj());
        }
        if (result_fp_aero() != nullptr) {
            fclose(result_fp_aero());
        }
        if (fp_boost != nullptr) {
            fclose(fp_boost);
        }
    }

// endregion

// region Public Methods
    void FileSaver::save_traj(const Missile *missile) {
        fprintf(
            result_fp_traj(),
            "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d\n",
            missile->flyTime(),
            missile->positionLaunchNUE().x(),
            missile->positionLaunchNUE().y(),
            missile->positionLaunchNUE().z(),
            missile->V(),
            missile->attitudeEuler().x(),
            missile->attitudeEuler().y(),
            missile->attitudeEuler().z(),
            missile->velocityTheta(),
            missile->thetaCmd(),
            missile->velocityPsi(),
            missile->alpha(),
            missile->beta(),
            missile->alphaCmd(),
            missile->betaCmd(),
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
            missile->sigmaAzDot(),
            missile->phaseId()
        );
        save_boost_events(missile);
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

    auto FileSaver::result_fp_boost() -> FILE * {
        if (!std::filesystem::exists(_directory)) {
            std::filesystem::create_directories(_directory);
        }
        std::string filename = _directory + "boost_summary.dat";
        if (!fp_boost) {
            fp_boost = fopen(filename.c_str(), "w");
            if (!fp_boost) {
                printf("error: failed to open file\n");
                exit(-1);
            }
            fprintf(fp_boost, "event time altitude velocity gamma tvc_dx tvc_dy tvc_dz\n");
        }
        return fp_boost;
    }

    void FileSaver::save_boost_events(const Missile *missile) {
        const auto burnOutTimes = Engine::boostBurnOutTimes();
        for (size_t i = 0; i < burnOutTimes.size(); ++i) {
            if (!_boostEventsSaved[i] && missile->flyTime() + 1.0e-5 >= burnOutTimes[i]) {
                const auto tvc = missile->tvcCommand();
                fprintf(
                    result_fp_boost(),
                    "%zu %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                    i + 1,
                    missile->flyTime(),
                    missile->lla().z(),
                    missile->V(),
                    missile->velocityTheta(),
                    tvc.x(),
                    tvc.y(),
                    tvc.z()
                );
                _boostEventsSaved[i] = true;
            }
        }

        if (!_boostEventsSaved[3] && missile->flyTime() > burnOutTimes.back() && missile->lla().z() <= 100000.0) {
            const auto tvc = missile->tvcCommand();
            fprintf(
                result_fp_boost(),
                "4 %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                missile->flyTime(),
                missile->lla().z(),
                missile->V(),
                missile->velocityTheta(),
                tvc.x(),
                tvc.y(),
                tvc.z()
            );
            _boostEventsSaved[3] = true;
        }
    }

// endregion
}
#undef PRETTY_FILE_NAME
