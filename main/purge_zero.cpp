#include "purge_zero.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "broadcast_data_store.h"
#include "injector_fsm.h"
#include <cmath>

extern commonInjectParams_t commonParams;

namespace PurgeZero {
    static bool stateEntry = false;
    static bool complete = false;
    static bool error = false;
    static bool buttonsReleased = false;
    static uint64_t lastCommandTime = 0;
    static int lastButtonState = 0;
    static float purgeZeroPosition = 0.0f;
    static bool preRetractDone = false;
    static bool preRetractCommanded = false;
    static float preRetractTarget = 0.0f;

    void begin() {
        stateEntry = true;
        complete = false;
        error = false;
        buttonsReleased = false;
        lastCommandTime = time_utils::micros();
        lastButtonState = 0;
        preRetractDone = false;
        preRetractCommanded = false;
        preRetractTarget = 0.0f;
    }

    bool update(CanBus& motor, bool buttonUp, bool buttonDown, bool buttonCenterReleased) {
        uint64_t now = time_utils::micros();

        if (!preRetractDone) {
            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            if (!preRetractCommanded) {
                if (!MotorWrapper::canStartMove()) {
                    return false;
                }
                preRetractTarget = bds.getPosition() + PURGE_PRE_RETRACT_TURNS;
                MotorWrapper::setMotorLimits(motor, REFILL_CONTROLLER_VEL_LIMIT, REFILL_CURRENT_LIMIT, MODULE_PURGE_ZERO, "Purge PreRetract");
                MotorWrapper::setTrapTrajParams(motor, REFILL_TRAP_VEL_LIMIT,
                                                commonParams.trapTrajAccelDecel, commonParams.trapTrajAccelDecel,
                                                MODULE_PURGE_ZERO, "Purge PreRetract Traj");
                if (!MotorWrapper::setModeAndMove(motor, 3, 5, preRetractTarget, MODULE_PURGE_ZERO, "Purge PreRetract")) {
                    return false;
                }
                preRetractCommanded = true;
                return false;
            }

            bool trajectoryComplete = bds.isTrajectoryComplete();
            bool motorStopped = std::fabs(bds.getVelocity()) < 0.1f;
            bool targetReached = std::fabs(bds.getPosition() - preRetractTarget) < 0.5f;
            if ((trajectoryComplete && motorStopped) || (targetReached && motorStopped)) {
                preRetractDone = true;
                stateEntry = true;
            } else {
                return false;
            }
        }

        if (stateEntry) {
            float purgeVelLimit = PURGE_VEL_LIMIT;
            float upLimit = std::fabs(commonParams.purgeVelUp);
            float downLimit = std::fabs(commonParams.purgeVelDown);
            if (upLimit > purgeVelLimit) purgeVelLimit = upLimit;
            if (downLimit > purgeVelLimit) purgeVelLimit = downLimit;
            MotorWrapper::setMotorLimits(motor, purgeVelLimit, commonParams.purgeCurrentLimit, MODULE_PURGE_ZERO, "PurgeZero");
            lastCommandTime = now;
            stateEntry = false;
        }

        if (!buttonsReleased) {
            if (!buttonUp && !buttonDown) {
                // both released (active low)
                buttonsReleased = true;
            }
            return false;
        }

        int currentButtonState = 0;
        if (buttonUp) {
            currentButtonState = 1;
        } else if (buttonDown) {
            currentButtonState = 2;
        }

        if (currentButtonState != lastButtonState) {
            if (currentButtonState == 1) {
                MotorWrapper::setModeAndMove(motor, 2, 1, commonParams.purgeVelUp, MODULE_PURGE_ZERO, "Purge Up");
            } else if (currentButtonState == 2) {
                MotorWrapper::setModeAndMove(motor, 2, 1, commonParams.purgeVelDown, MODULE_PURGE_ZERO, "Purge Down");
            } else {
                MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_PURGE_ZERO, "Purge Stop");
            }
            lastButtonState = currentButtonState;
            lastCommandTime = now;
        }

        if (buttonCenterReleased) {
            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            purgeZeroPosition = bds.getPosition();
            MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_PURGE_ZERO, "Purge Confirm");
            lastCommandTime = now;
            complete = true;
            return true;
        }

        return false;
    }

    bool isComplete() { return complete && !error; }
    bool hasError() { return error; }
    float getPurgeZeroPosition() { return purgeZeroPosition; }

    void reset() {
        stateEntry = true;
        complete = false;
        error = false;
        buttonsReleased = false;
        preRetractDone = false;
        preRetractCommanded = false;
        preRetractTarget = 0.0f;
    }
}
