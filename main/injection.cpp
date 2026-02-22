#include "injection.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "broadcast_data_store.h"
#include "purge_zero.h"
#include <cmath>
#include "esp_log.h"

namespace Injection {
    static InjectionPhase phase = DONE;
    static bool stateEntry = false;
    static uint64_t stateEnterTime = 0;
    static uint64_t phaseStartTime = 0;
    static bool complete = false;
    static bool error = false;
    static uint64_t lastCommandTime = 0;
    static bool pressureSensorChecked = false;

    static actualMouldParams_t currentMould;

    static float injectStartPos = 0.0f;
    static float packStartPos = 0.0f;
    static float targetInjectPos = 0.0f;
    static float targetPackPos = 0.0f;
    static bool torqueMode = false;

    static float volToTurns(float cm3) {
        return cm3 * TURNS_PER_CM3_VOL;
    }

    static float turnsToVol(float turns) {
        if (TURNS_PER_CM3_VOL == 0.0f) return 0.0f;
        return turns / TURNS_PER_CM3_VOL;
    }

    void begin(const actualMouldParams_t& mouldParams) {
        currentMould = mouldParams;
        phase = FILLING;
        stateEntry = true;
        stateEnterTime = 0;
        phaseStartTime = 0;
        complete = false;
        error = false;
        lastCommandTime = time_utils::micros();
        pressureSensorChecked = false;
        injectStartPos = 0.0f;
        packStartPos = 0.0f;
        torqueMode = (currentMould.injectMode == INJECT_MODE_3D);
    }

    bool update(CanBus& motor) {
        uint64_t now = time_utils::micros();
        uint64_t stateElapsed = (now - stateEnterTime) / 1000ULL;
        uint64_t phaseElapsed = (now - phaseStartTime) / 1000ULL;

        if (stateEntry) {
            if (!MotorWrapper::canStartMove()) {
                return false;
            }
            pressureSensorChecked = true;

            injectStartPos = PurgeZero::getPurgeZeroPosition();
            const float maxInjectTurns = POS_BOTTOM_MAX - injectStartPos;
            const float maxInjectVolume = turnsToVol(maxInjectTurns);

            if (!torqueMode && currentMould.fillVolume > maxInjectVolume) {
#if APP_DEBUG
                ESP_LOGW("INJECT", "FILL_VOLUME_TOO_LARGE: req=%.2f max=%.2f", currentMould.fillVolume, maxInjectVolume);
#endif
                error = true;
                complete = true;
                stateEntry = false;
                return true;
            }

            if (torqueMode) {
                targetInjectPos = 0.0f;
                MotorWrapper::setMotorLimits(motor, INJECT_FILL_CONTROLLER_VEL_LIMIT, GENERAL_MACHINE_CURRENT_LIMIT, MODULE_INJECT, "Inject Torque");
                if (!MotorWrapper::setModeAndMove(motor, 1, 6, currentMould.injectTorque, MODULE_INJECT, "Inject Torque")) {
                    return false;
                }
            } else {
                targetInjectPos = injectStartPos + volToTurns(currentMould.fillVolume);
                if (targetInjectPos > POS_BOTTOM_MAX) targetInjectPos = POS_BOTTOM_MAX;

                float fillCurrent = currentMould.fillPressure > 0.0f ? currentMould.fillPressure : GENERAL_MACHINE_CURRENT_LIMIT;
                float fillVel = currentMould.fillSpeed > 0.0f ? currentMould.fillSpeed : INJECT_FILL_TRAP_VEL_LIMIT;
                float fillAccel = currentMould.fillTrapAccel > 0.0f ? currentMould.fillTrapAccel : TRAP_TRAJ_GENERAL_ACCEL_DECEL;
                float fillDecel = currentMould.fillTrapDecel > 0.0f ? currentMould.fillTrapDecel : TRAP_TRAJ_GENERAL_ACCEL_DECEL;

                MotorWrapper::setMotorLimits(motor, INJECT_FILL_CONTROLLER_VEL_LIMIT, fillCurrent, MODULE_INJECT, "Inject Fill");
                MotorWrapper::setTrapTrajParams(motor, fillVel, fillAccel, fillDecel, MODULE_INJECT, "Fill Traj");
                if (!MotorWrapper::setModeAndMove(motor, 3, 5, targetInjectPos, MODULE_INJECT, "Pos Inject")) {
                    return false;
                }
            }
            lastCommandTime = now;
            stateEnterTime = now;
            phaseStartTime = now;
            stateEntry = false;
            return false;
        }

        if (phase == FILLING) {
            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            bool velocityLow = std::fabs(bds.getVelocity()) < INJECT_VEL_THRESHOLD;
            bool positionClose = std::fabs(bds.getPosition() - targetInjectPos) < INJECT_POS_TOLERANCE;
            bool timingStable = phaseElapsed > INJECT_STABLE_TIME_MS;

            if (torqueMode) {
                if (velocityLow && timingStable) {
                    MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_INJECT, "Inject Torque Stop");
                    phase = DONE;
                    complete = true;
                    return true;
                }
                return false;
            }

            // If we reached target, transition immediately (no delay).
            if (velocityLow && positionClose) {
                phase = PACKING;
                phaseStartTime = now;
                packStartPos = bds.getPosition();
                targetPackPos = packStartPos + volToTurns(currentMould.packVolume);
                if (targetPackPos > POS_BOTTOM_MAX) targetPackPos = POS_BOTTOM_MAX;

                float packCurrent = currentMould.packPressure > 0.0f ? currentMould.packPressure : GENERAL_MACHINE_CURRENT_LIMIT;
                float packVel = currentMould.packSpeed > 0.0f ? currentMould.packSpeed : INJECT_PACK_TRAP_VEL_LIMIT;
                float packAccel = currentMould.packTrapAccel > 0.0f ? currentMould.packTrapAccel : TRAP_TRAJ_GENERAL_ACCEL_DECEL;
                float packDecel = currentMould.packTrapDecel > 0.0f ? currentMould.packTrapDecel : TRAP_TRAJ_GENERAL_ACCEL_DECEL;

                MotorWrapper::setMotorLimits(motor, INJECT_PACK_CONTROLLER_VEL_LIMIT, packCurrent, MODULE_INJECT, "Inject Pack");
                MotorWrapper::setTrapTrajParams(motor, packVel, packAccel, packDecel, MODULE_INJECT, "Pack Traj");
                MotorWrapper::setModeAndMove(motor, 3, 5, targetPackPos, MODULE_INJECT, "Pos Pack");
                lastCommandTime = now;
                return false;
            }

            // If we didn't reach target, allow exit when motion is stable for a while.
            if (velocityLow && timingStable) {
                phase = PACKING;
                phaseStartTime = now;
                packStartPos = bds.getPosition();
                targetPackPos = packStartPos + volToTurns(currentMould.packVolume);
                if (targetPackPos > POS_BOTTOM_MAX) targetPackPos = POS_BOTTOM_MAX;

                float packCurrent = currentMould.packPressure > 0.0f ? currentMould.packPressure : GENERAL_MACHINE_CURRENT_LIMIT;
                float packVel = currentMould.packSpeed > 0.0f ? currentMould.packSpeed : INJECT_PACK_TRAP_VEL_LIMIT;
                float packAccel = currentMould.packTrapAccel > 0.0f ? currentMould.packTrapAccel : TRAP_TRAJ_GENERAL_ACCEL_DECEL;
                float packDecel = currentMould.packTrapDecel > 0.0f ? currentMould.packTrapDecel : TRAP_TRAJ_GENERAL_ACCEL_DECEL;

                MotorWrapper::setMotorLimits(motor, INJECT_PACK_CONTROLLER_VEL_LIMIT, packCurrent, MODULE_INJECT, "Inject Pack");
                MotorWrapper::setTrapTrajParams(motor, packVel, packAccel, packDecel, MODULE_INJECT, "Pack Traj");
                MotorWrapper::setModeAndMove(motor, 3, 5, targetPackPos, MODULE_INJECT, "Pos Pack");
                lastCommandTime = now;
                return false;
            }

            if (phaseElapsed > INJECT_FILL_TIMEOUT_MS) {
                error = true;
                complete = true;
                MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_INJECT, "Inject Timeout Stop");
                return true;
            }
            return false;
        }

        if (phase == PACKING) {
            uint64_t packTimeMs = static_cast<uint64_t>(currentMould.packTime * 1000.0f);
            if (phaseElapsed >= packTimeMs) {
                phase = DONE;
                complete = true;
                return true;
            }
            return false;
        }

        return complete;
    }

    bool isComplete() { return complete && !error; }
    bool hasError() { return error; }
    InjectionPhase getPhase() { return phase; }

    float getInjectStartPos() { return injectStartPos; }
    float getTargetInjectPos() { return targetInjectPos; }
    float getTargetPackPos() { return targetPackPos; }

    float getInjectVolumeFromPos(float currentPos) {
        return turnsToVol(currentPos - injectStartPos);
    }

    float getTargetInjectVolume() {
        return turnsToVol(targetInjectPos - injectStartPos);
    }

    float getInjectTargetPos() { return targetInjectPos; }
    float getInjectTorque() { return currentMould.injectTorque; }
    bool isTorqueMode() { return torqueMode; }

    void reset() {
        stateEntry = true;
        phase = DONE;
        complete = false;
        error = false;
        pressureSensorChecked = false;
    }
}
