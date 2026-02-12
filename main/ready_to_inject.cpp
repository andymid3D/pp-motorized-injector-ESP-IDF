#include "ready_to_inject.h"
#include "config.h"
#include "motor_wrapper.h"
#include "time_utils.h"
#include "broadcast_data_store.h"
#include "injector_fsm.h"
#include "esp_log.h"
#include "temperature.h"
#include <cmath>

extern commonInjectParams_t commonParams;

namespace ReadyToInject {
    static enum { IDLE_WAITING, MICRO_COMPRESSING } state = IDLE_WAITING;
    static bool stateEntry = false;
    static uint64_t stateEnterTime = 0;
    static uint64_t lastAutoCompressionTime = 0;
    static uint64_t compressionStartTime = 0;
    static bool error = false;
    static uint64_t lastCommandTime = 0;
    static bool torqueCommandSent = false;

    void begin() {
        state = IDLE_WAITING;
        stateEntry = true;
        stateEnterTime = time_utils::micros();
        lastAutoCompressionTime = time_utils::micros();
        error = false;
        lastCommandTime = time_utils::micros();
        torqueCommandSent = false;
    }

    bool update(CanBus& motor) {
        BroadcastDataStore& bds = BroadcastDataStore::getInstance();
        uint64_t now = time_utils::micros();
        static uint64_t lastSkipLogUs = 0;
        static uint64_t lastStateLogUs = 0;

        if (stateEntry) {
            MotorWrapper::setMotorLimits(motor, REFILL_CONTROLLER_VEL_LIMIT, REFILL_CURRENT_LIMIT, MODULE_READY_TO_INJECT, "ReadyIdle");
            MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_READY_TO_INJECT, "Idle Stop");
            stateEntry = false;
        }

        if (state == IDLE_WAITING) {
            uint64_t timeSinceLastCompress = now - lastAutoCompressionTime;
#if APP_DEBUG
            if (now - lastStateLogUs > 5000000ULL) {
                int tempC = readTemperatureC();
                ESP_LOGI("READY_TO_INJECT", "MICRO idle: since=%llums interval=%ums canStart=%d temp=%d vel=%.2f",
                         (unsigned long long)(timeSinceLastCompress / 1000ULL),
                         READY_MICRO_INTERVAL_MS,
                         MotorWrapper::canStartMove(),
                         tempC,
                         bds.getVelocity());
                lastStateLogUs = now;
            }
#endif
            if (timeSinceLastCompress >= (READY_MICRO_INTERVAL_MS * 1000ULL)) {
                if (!MotorWrapper::canStartMove()) {
#if APP_DEBUG
                    if (now - lastSkipLogUs > 1000000ULL) {
                        int tempC = readTemperatureC();
                        ESP_LOGW("READY_TO_INJECT", "MICRO skipped: canStartMove=false temp=%d vel=%.2f",
                                 tempC, bds.getVelocity());
                        lastSkipLogUs = now;
                    }
#endif
                    return false;
                }
                MotorWrapper::setMotorLimits(motor, COMPRESS_MICRO_VEL_LIMIT, COMPRESS_MICRO_CURRENT, MODULE_READY_TO_INJECT, "MicroCompress");
                motor.setControllerModes(odrive_can::ControlMode::TORQUE_CONTROL, odrive_can::InputMode::TORQUE_RAMP);
                state = MICRO_COMPRESSING;
                compressionStartTime = time_utils::micros();
                lastCommandTime = time_utils::micros();
                torqueCommandSent = false;
            }
            return false;
        }

        if (state == MICRO_COMPRESSING) {
            uint64_t compressionElapsed = now - compressionStartTime;
            float rampDuration = READY_MICRO_DURATION_MS / 1000.0f;
            float elapsedSec = compressionElapsed / 1000000.0f;
            float targetTorque = (commonParams.compressMicroCurrent / rampDuration) * elapsedSec;
            if (targetTorque > commonParams.compressMicroCurrent) targetTorque = commonParams.compressMicroCurrent;

            if (!torqueCommandSent || (now - lastCommandTime) > 200000ULL) {
                bool commandQueued = MotorWrapper::setModeAndMoveWithRetry(
                    motor, 1, 6, targetTorque, MODULE_READY_TO_INJECT,
                    "MicroCompress", MotorWrapper::PRIORITY_NORMAL
                );
                if (commandQueued) {
                    if (!torqueCommandSent) {
#if APP_DEBUG
                        ESP_LOGI("READY_TO_INJECT", "MICRO start: target=%.2f vel=%.2f",
                                 targetTorque, bds.getVelocity());
#endif
                    }
                    torqueCommandSent = true;
                    lastCommandTime = now;
                }
            }

            bool completedByTime = compressionElapsed >= (READY_MICRO_DURATION_MS * 1000ULL);
            const TimestampedIq* iqData = bds.getLatestIq();
            bool completedByTorque = false;
            if (iqData && compressionElapsed > 200000ULL) {
                completedByTorque = std::fabs(iqData->iqMeasured) >= (commonParams.compressMicroCurrent * 0.9f);
            }

            if (completedByTime || completedByTorque) {
                MotorWrapper::setModeAndMove(motor, 1, 6, 0, MODULE_READY_TO_INJECT, "MicroCompress Release");
                lastAutoCompressionTime = now;
                state = IDLE_WAITING;
                torqueCommandSent = false;
                lastCommandTime = now;
#if APP_DEBUG
                ESP_LOGI("READY_TO_INJECT", "MICRO done: by=%s elapsed=%llums",
                         completedByTime ? "time" : "torque",
                         (unsigned long long)(compressionElapsed / 1000ULL));
#endif
            }
            return false;
        }

        return false;
    }

    bool isComplete() { return false; }
    bool isMicroCompressing() { return state == MICRO_COMPRESSING; }
    bool hasError() { return error; }

    void reset() {
        state = IDLE_WAITING;
        stateEntry = true;
        error = false;
        torqueCommandSent = false;
    }
}
