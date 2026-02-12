#include "compression.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "injector_fsm.h"
#include "broadcast_data_store.h"
#include <cmath>

extern commonInjectParams_t commonParams;

namespace Compression {
    static CompressionMode currentMode = MODE_1_TRAVEL;
    static enum { PRESSURE_CHECK, TRAVEL_DOWN, DONE } step = DONE;

    static bool stateEntry = false;
    static uint64_t stateEnterTime = 0;
    static uint64_t stepTimer = 0;
    static bool complete = false;
    static bool isErrorFlag = false;
    static bool isTimeoutFlag = false;
    static bool pressureSensorChecked = false;
    static uint64_t lastCommandTime = 0;
    static bool torqueCommandSent = false;

    void begin(CompressionMode mode) {
        currentMode = mode;
        stateEntry = true;
        stateEnterTime = time_utils::micros();
        stepTimer = time_utils::micros();
        complete = false;
        isErrorFlag = false;
        isTimeoutFlag = false;
        pressureSensorChecked = false;
        lastCommandTime = time_utils::micros();
        torqueCommandSent = false;

        if (mode == MODE_1_TRAVEL) step = PRESSURE_CHECK;
        else step = TRAVEL_DOWN;
    }

    bool update(CanBus& motor) {
        uint64_t now = time_utils::micros();
        uint64_t stepElapsed = now - stepTimer;

        if (step == PRESSURE_CHECK) {
            if (stateEntry) {
                pressureSensorChecked = true;
                stateEntry = false;
            }
            if (stepElapsed > 50) {
                step = TRAVEL_DOWN;
                stepTimer = now;
                stateEntry = true;
            }
            return false;
        }

        if (step == TRAVEL_DOWN) {
            if (stateEntry) {
                if (!MotorWrapper::canStartMove()) {
                    return false;
                }
                MotorWrapper::setMotorLimits(motor, COMPRESS_TRAVEL_VEL_LIMIT, COMPRESS_TRAVEL_CURRENT, MODULE_COMPRESSION, "Compress Travel");
                if (!MotorWrapper::setModeAndMove(motor, 1, 6, COMPRESS_TRAVEL_TORQUE, MODULE_COMPRESSION, "Compress Travel Down Torque")) {
                    return false;
                }
                lastCommandTime = time_utils::micros();
                stepTimer = time_utils::micros();
                stateEntry = false;
            }

            uint64_t travelElapsed = time_utils::micros() - stepTimer;
            BroadcastDataStore& bds = BroadcastDataStore::getInstance();
            bool stallDetected = bds.getAxisError() != 0;
            const TimestampedIq* iqData = bds.getLatestIq();
            bool torqueExceeded = false;
            if (iqData) {
                torqueExceeded = std::fabs(bds.getVelocity()) < 0.1f &&
                                 iqData->iqMeasured > COMPRESS_CONTACT_IQ_THRESHOLD;
            }

            if (stallDetected || torqueExceeded) {
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_COMPRESSION, "Compress Stop");
                complete = true;
                step = DONE;
                return true;
            }

            return false;
        }

        return complete;
    }

    bool isComplete() { return complete && !isErrorFlag && !isTimeoutFlag; }
    bool hasError() { return isErrorFlag; }
    bool isTimeout() { return isTimeoutFlag; }

    void reset() {
        stateEntry = true;
        complete = false;
        isErrorFlag = false;
        isTimeoutFlag = false;
        pressureSensorChecked = false;
        torqueCommandSent = false;
    }
}
