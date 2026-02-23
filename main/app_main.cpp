#include <cmath>
#include <cstring>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "antidrip.h"
#include "broadcast_data_store.h"
#include "can_bus.h"
#include "can_rx.h"
#include "compression.h"
#include "config.h"
#include "debounced_input.h"
#include "debug_console.h"
#include "display_comms.h"
#include "error_manager.h"
#include "homing.h"
#include "injection.h"
#include "injector_fsm.h"
#include "leds.h"
#include "motor_wrapper.h"
#include "nvs_flash.h"
#include "purge_zero.h"
#include "ready_to_inject.h"
#include "refill.h"
#include "safety_manager.h"
#include "temperature.h"
#include "time_utils.h"

static const char *TAG = "APP";

// FSM globals
fsm_inputs_t fsm_inputs = {};
fsm_outputs_t fsm_outputs = {};
fsm_state_t fsm_state = {};
volatile InjectorStates requestedRemoteState = ERROR_STATE;

// Hardware objects
SafetyManager safety;
CanBus motor;

DebouncedInput btnCenter;
DebouncedInput btnUpper;
DebouncedInput btnLower;

MachineFlags flags = {false, false, false, false};

static uint64_t bootTime = 0;
static uint64_t stateTimer = 0;
static bool buttonLock = false;
static int lastFsmState = -1;
static bool stateEntry = false;
static bool moveLockActive = false;
static bool tempCriticalStopIssued = false;
static uint64_t lastTempOnlyLog = 0;
static uint64_t lastButtonLogUs = 0;
static uint64_t lastCanOfflineLogUs = 0;
static bool injectTargetLogged = false;
static bool peakInit = false;
static float peakPos = 0.0f;
static float peakVel = 0.0f;
static float peakTemp = 0.0f;
static float peakIqSet = 0.0f;
static float peakIqMeas = 0.0f;
static long peakPressure = 0;
static float peakBusV = 0.0f;
static float peakBusI = 0.0f;

static bool isPowerCutError(uint16_t err) {
  return err == ERR_ESTOP || err == ERR_BARREL_POSITION_LOST ||
         err == ERR_BOTTOM_ENDSTOP_COLLISION ||
         err == ERR_TOP_ENDSTOP_COLLISION;
}

// Common params (runtime-writable)
commonInjectParams_t commonParams = {TRAP_TRAJ_GENERAL_ACCEL_DECEL,
                                     COMPRESS_TRAVEL_TORQUE,
                                     READY_MICRO_INTERVAL_MS,
                                     READY_MICRO_DURATION_MS,
                                     PURGE_VEL_UP,
                                     PURGE_VEL_DOWN,
                                     PURGE_CURRENT_LIMIT,
                                     ANTIDRIP_VEL,
                                     ANTIDRIP_CURRENT_LIMIT,
                                     RELEASE_DIST,
                                     RELEASE_TRAP_VEL_LIMIT,
                                     RELEASE_CURRENT_LIMIT};

actualMouldParams_t currentMould = {"maceta",
                                    55.0f,
                                    25.0f,
                                    20.0f,
                                    5.0f,
                                    20.0f,
                                    20.0f,
                                    2.0f,
                                    TRAP_TRAJ_GENERAL_ACCEL_DECEL,
                                    TRAP_TRAJ_GENERAL_ACCEL_DECEL,
                                    TRAP_TRAJ_GENERAL_ACCEL_DECEL,
                                    TRAP_TRAJ_GENERAL_ACCEL_DECEL,
                                    INJECT_MODE_3D,
                                    40.0f};

static const char *getStateName(int state) {
  switch (state) {
  case ERROR_STATE:
    return "ERROR_STATE";
  case INIT_HEATING:
    return "INIT_HEATING";
  case INIT_HOT_NOT_HOMED:
    return "INIT_HOT_WAIT";
  case INIT_HOMING:
    return "INIT_HOMING";
  case INIT_HOMED_ENCODER_ZEROED:
    return "INIT_HOMED_ENCODER_ZEROED";
  case REFILL:
    return "REFILL";
  case COMPRESSION:
    return "COMPRESSION";
  case READY_TO_INJECT:
    return "READY_TO_INJECT";
  case PURGE_ZERO:
    return "PURGE_ZERO";
  case ANTIDRIP:
    return "ANTIDRIP";
  case INJECT:
    return "INJECT";
  case HOLD_INJECTION:
    return "HOLD_PACKING";
  case RELEASE:
    return "RELEASE";
  case CONFIRM_MOULD_REMOVAL:
    return "CONFIRM_REMOVAL";
  default:
    return "UNKNOWN";
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "ESP-IDF injector starting");

  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  MotorWrapper::init();

  motor.begin();
  CanRxHandler &canRx = CanRxHandler::getInstance();
  canRx.begin();
  canRx.startCore0Task();

  safety.begin();
  leds_init();

  btnUpper.begin((gpio_num_t)PIN_BTN_UPPER, true, 10, true);
  btnCenter.begin((gpio_num_t)PIN_BTN_CENTER, true, 10, true);
  btnLower.begin((gpio_num_t)PIN_BTN_LOWER, true, 10, true);

  fsm_state.currentState = InjectorStates::INIT_HEATING;
  bootTime = time_utils::micros();

  debug_console_init(&motor);
  DisplayComms::begin();

  uint64_t lastDebugTime = 0;

  while (true) {
    uint64_t loopNowUs = time_utils::micros();
    motor.loop();
    safety.updateInputs();
    fsm_inputs.nozzleTemperature = readTemperatureC();

    BroadcastDataStore &bds = BroadcastDataStore::getInstance();
    bds.setTemperature(fsm_inputs.nozzleTemperature);
    MotorWrapper::updateMotionState(fsm_inputs.nozzleTemperature,
                                    bds.getVelocity());

    if (safety.isMotorPowerEnabled() && bds.isHeartbeatStale(2000000)) {
      if (loopNowUs - lastCanOfflineLogUs > 1000000ULL) {
        ESP_LOGW(TAG, "CAN_OFFLINE: no heartbeat >2000ms");
        lastCanOfflineLogUs = loopNowUs;
      }
    } else {
      lastCanOfflineLogUs = 0;
    }

    // Always update buttons, comms and console even if machine is cold
    btnCenter.update();
    btnUpper.update();
    btnLower.update();
    debug_console_poll();
    DisplayComms::update();

    if (MotorWrapper::isTempCritical()) {
      if (loopNowUs - lastTempOnlyLog > 1000000ULL) {
        lastTempOnlyLog = loopNowUs;
        long prRaw = safety.getLoadCellRaw();
        ESP_LOGI(TAG, "TEMP_ONLY Temp:%d Contactor:%s Pr:%ld Cyc:%u",
                 fsm_inputs.nozzleTemperature,
                 safety.isMotorPowerEnabled() ? "ON" : "OFF", prRaw,
                 safety.getContactorCycles());
      }
      if (!tempCriticalStopIssued && safety.isMotorPowerEnabled()) {
        MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_SAFETY_MANAGER,
                                     "TempCritical Stop");
        tempCriticalStopIssued = true;
      }
      safety.enableMotorPower(false);
      if (fsm_state.currentState != INIT_HEATING) {
        fsm_state.currentState = INIT_HEATING;
      }
      goto loop_end;
    } else {
      tempCriticalStopIssued = false;
    }

#if APP_DEBUG
    if (btnUpper.fell())
      ESP_LOGI(TAG, "BTN_UP pressed");
    if (btnUpper.rose())
      ESP_LOGI(TAG, "BTN_UP released");
    if (btnCenter.fell())
      ESP_LOGI(TAG, "BTN_CENTER pressed");
    if (btnCenter.rose())
      ESP_LOGI(TAG, "BTN_CENTER released");
    if (btnLower.fell())
      ESP_LOGI(TAG, "BTN_LOWER pressed");
    if (btnLower.rose())
      ESP_LOGI(TAG, "BTN_LOWER released");
    if ((btnUpper.isPressed() && btnLower.isPressed()) &&
        (loopNowUs - lastButtonLogUs > 500000ULL)) {
      ESP_LOGI(TAG, "BTN_COMBO: Upper+Lower");
      lastButtonLogUs = loopNowUs;
    } else if ((btnCenter.isPressed() && btnLower.isPressed()) &&
               (loopNowUs - lastButtonLogUs > 500000ULL)) {
      ESP_LOGI(TAG, "BTN_COMBO: Center+Lower");
      lastButtonLogUs = loopNowUs;
    }
#endif

    // Track 1s peak values for logging
    {
      BroadcastDataStore &bds = BroadcastDataStore::getInstance();
      float pos = bds.getPosition();
      float vel = std::fabs(bds.getVelocity());
      float iqSet = std::fabs(bds.getIqSetpoint());
      float iqMeas = std::fabs(bds.getIqMeasured());
      long prRaw = std::labs(safety.getLoadCellRaw());
      float busV = bds.getBusVoltage();
      float busI = std::fabs(bds.getBusCurrent());
      float temp = static_cast<float>(fsm_inputs.nozzleTemperature);

      if (!peakInit) {
        peakPos = pos;
        peakVel = vel;
        peakTemp = temp;
        peakIqSet = iqSet;
        peakIqMeas = iqMeas;
        peakPressure = prRaw;
        peakBusV = busV;
        peakBusI = busI;
        peakInit = true;
      } else {
        if (pos > peakPos)
          peakPos = pos;
        if (vel > peakVel)
          peakVel = vel;
        if (temp > peakTemp)
          peakTemp = temp;
        if (iqSet > peakIqSet)
          peakIqSet = iqSet;
        if (iqMeas > peakIqMeas)
          peakIqMeas = iqMeas;
        if (prRaw > peakPressure)
          peakPressure = prRaw;
        if (busV > peakBusV)
          peakBusV = busV;
        if (busI > peakBusI)
          peakBusI = busI;
      }
    }

    if (fsm_state.currentState != lastFsmState) {
      stateEntry = true;
#if APP_DEBUG
      const char *prevState =
          (lastFsmState < 0) ? "BOOT" : getStateName(lastFsmState);
      ESP_LOGI(TAG, "FSM %s -> %s (Err:0x%X)", prevState,
               getStateName(fsm_state.currentState), fsm_state.error);
#endif
      lastFsmState = fsm_state.currentState;
      stateTimer = time_utils::micros();
      if (fsm_state.currentState == INJECT) {
        injectTargetLogged = false;
      }
    } else {
      stateEntry = false;
    }

    if (btnUpper.isPressed() && btnLower.isPressed())
      buttonLock = true;
    else if (btnCenter.isPressed() && btnLower.isPressed())
      buttonLock = true;
    if (buttonLock && !btnUpper.isPressed() && !btnLower.isPressed() &&
        !btnCenter.isPressed())
      buttonLock = false;

    static uint64_t resetTime = 0;
    if (buttonLock && resetTime > 0 &&
        (time_utils::micros() - resetTime > 500000ULL)) {
      buttonLock = false;
      resetTime = 0;
    }

    // Safety and error checks after 3s boot
    if (time_utils::micros() - bootTime > 3000000ULL) {
      bool movingDown = bds.getVelocity() > ENDSTOP_VEL_MIN;

      uint32_t axisErr = bds.getAxisError();
      uint64_t motorErr = bds.getMotorErrorSafe();
      uint32_t encoderErr = bds.getEncoderError();
      uint32_t controllerErr = bds.getControllerErrorSafe();

      if (hasAnyError(axisErr, motorErr, encoderErr, controllerErr) &&
          fsm_state.currentState != INIT_HEATING &&
          fsm_state.currentState != INIT_HOT_NOT_HOMED &&
          fsm_state.currentState != INIT_HOMING &&
          fsm_state.currentState != ERROR_STATE) {

        logError(axisErr, motorErr, encoderErr, controllerErr,
                 fsm_state.currentState);
        ErrorSeverity severity =
            classifyError(axisErr, motorErr, encoderErr, controllerErr);

        switch (severity) {
        case ERR_EXPECTED_TRANSIENT:
        case ERR_RECOVERABLE_RETRY: {
          bool moveComplete = (std::fabs(bds.getVelocity()) < 0.1f &&
                               bds.isTrajectoryComplete());
          handleRecoverableError(severity, axisErr, motorErr, encoderErr,
                                 controllerErr, moveComplete);
          if (errorManagerNeedsShutdown()) {
            fsm_state.currentState = ERROR_STATE;
            fsm_state.error = safety.getLastError();
            resetErrorManagerState();
            goto loop_end;
          }
          break;
        }
        case ERR_RECOVERABLE_HOMING: {
          bool moveComplete = (std::fabs(bds.getVelocity()) < 0.1f &&
                               bds.isTrajectoryComplete());
          handleRecoverableError(severity, axisErr, motorErr, encoderErr,
                                 controllerErr, moveComplete);
          if (errorManagerNeedsShutdown()) {
            fsm_state.currentState = ERROR_STATE;
            fsm_state.error = safety.getLastError();
            resetErrorManagerState();
            goto loop_end;
          }
          fsm_state.currentState = INIT_HEATING; // restart init path
          flags.calibrationDone = false;
          break;
        }
        case ERR_SAFETY_CRITICAL:
        default:
          fsm_state.currentState = ERROR_STATE;
          fsm_state.error = safety.getLastError();
          resetErrorManagerState();
          break;
        }
      }

      if (fsm_state.currentState != INIT_HEATING &&
          fsm_state.currentState != ERROR_STATE &&
          !errorManagerNeedsShutdown()) {
        if (!safety.check(bds.getVelocity(), movingDown)) {
          fsm_state.currentState = ERROR_STATE;
          fsm_state.error = safety.getLastError();
          if (isPowerCutError(fsm_state.error)) {
            Homing::invalidateCalibration();
          }
          goto loop_end;
        }
      }
    }

    {
      bool ignoreButtons = (time_utils::micros() - stateTimer < 500000ULL);

      // Handle remote state requests from display UI
      if (requestedRemoteState != ERROR_STATE) {
        if (!ignoreButtons && !buttonLock) {
          bool validTransition = false;
          switch (fsm_state.currentState) {
          case COMPRESSION:
            if (requestedRemoteState == REFILL) {
              Compression::reset();
              moveLockActive = true;
              validTransition = true;
            }
            break;
          case READY_TO_INJECT:
            if (requestedRemoteState == REFILL) {
              ReadyToInject::reset();
              moveLockActive = true;
              validTransition = true;
            } else if (requestedRemoteState == PURGE_ZERO) {
              if (MotorWrapper::canStartMove()) {
                ReadyToInject::reset();
                validTransition = true;
              }
            }
            break;
          case PURGE_ZERO:
            if (requestedRemoteState == READY_TO_INJECT) {
              if (MotorWrapper::canStartMove()) {
                PurgeZero::reset();
                MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_PURGE_ZERO,
                                             "Remote Stop");
                validTransition = true;
              }
            }
            break;
          case INIT_HOMED_ENCODER_ZEROED:
          case INIT_HOT_NOT_HOMED:
            if (requestedRemoteState == INIT_HOMING) {
              if (fsm_inputs.nozzleTemperature >= TEMP_MIN_MOVE) {
                validTransition = true;
              }
            }
            break;
          case REFILL:
            if (requestedRemoteState == COMPRESSION) {
              if (MotorWrapper::canStartMove()) {
                validTransition = true;
              }
            }
            break;
          default:
            break;
          }

          if (validTransition) {
            ESP_LOGI(TAG, "UART requested transition: %s -> %s",
                     getStateName(fsm_state.currentState),
                     getStateName(requestedRemoteState));
            fsm_state.currentState = requestedRemoteState;
            stateEntry = true;
            stateTimer = time_utils::micros();
            moveLockActive =
                false; // Unlock physical buttons on remote transition
          } else {
            ESP_LOGW(TAG, "Ignored invalid UART transition: %s -> %s",
                     getStateName(fsm_state.currentState),
                     getStateName(requestedRemoteState));
          }
        }
        requestedRemoteState = ERROR_STATE;
      }

      switch (fsm_state.currentState) {
      case ERROR_STATE: {
        if (stateEntry) {
          safety.enableMotorPower(false);
          flags.initialHomingDone = false;
          if (fsm_state.error != ERR_ESTOP) {
            if (safety.isMotorPowerEnabled()) {
              MotorWrapper::setModeAndMove(motor, 2, 1, 0, MODULE_COMPRESSION,
                                           "Stop");
            }
          }
        }
        if (fsm_state.error != ERR_ESTOP) {
          if (safety.isEStopPressed() || safety.isBarrelOpen())
            safety.enableMotorPower(false);
          else
            safety.enableMotorPower(true);
        }
        if (!ignoreButtons && !buttonLock && btnCenter.rose()) {
#if APP_DEBUG
          ESP_LOGI(TAG, "RESET pressed");
#endif
          if (!safety.isEStopPressed() && !safety.isBarrelOpen()) {
            safety.resetError();
            fsm_state.error = 0;
            motor.clearErrors();
            btnCenter.update();
            btnUpper.update();
            btnLower.update();
            buttonLock = true;
            resetTime = time_utils::micros();
            // Reset goes to safe init waiting path
            fsm_state.currentState = INIT_HEATING;
#if APP_DEBUG
            ESP_LOGI(TAG, "RESET accepted -> INIT_HEATING");
#endif
          } else {
#if APP_DEBUG
            ESP_LOGW(TAG, "RESET denied (EStop:%d Barrel:%d)",
                     safety.isEStopPressed(), safety.isBarrelOpen());
#endif
          }
        }
        break;
      }

      case INIT_HEATING: {
        if (stateEntry) {
          BroadcastDataStore &bds = BroadcastDataStore::getInstance();
          if (bds.getAxisState() != 0) {
            motor.clearErrors();
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
        if (fsm_inputs.nozzleTemperature >= TEMP_CRITICAL_RECOVER) {
          fsm_state.currentState = INIT_HOT_NOT_HOMED;
        } else {
          safety.enableMotorPower(false);
        }
        break;
      }

      case INIT_HOMED_ENCODER_ZEROED:
      case INIT_HOT_NOT_HOMED: {
        safety.enableMotorPower(true);
        if (!ignoreButtons && !buttonLock && btnUpper.rose()) {
          if (fsm_inputs.nozzleTemperature >= TEMP_MIN_MOVE) {
            fsm_state.currentState = INIT_HOMING;
          }
        }
        break;
      }

      case INIT_HOMING: {
        if (stateEntry) {
          Homing::begin(motor, safety);
          stateEntry = false;
        }
        Homing::update(motor, safety);
        if (Homing::isComplete()) {
          flags.initialHomingDone = true;
          fsm_state.currentState = REFILL;
        } else if (Homing::hasError()) {
          fsm_state.currentState = ERROR_STATE;
          fsm_state.error = 0xFF;
        }
        break;
      }

      case REFILL: {
        if (stateEntry) {
          Refill::begin();
          moveLockActive = !Refill::isComplete();
          stateEntry = false;
        }
        if (Refill::update(motor)) {
          moveLockActive = false;
        }
        if (moveLockActive && Refill::isComplete()) {
          moveLockActive = false;
        }

        if (!ignoreButtons && !moveLockActive) {
          static uint64_t togglePressTime = 0;
          static bool toggleProcessed = false;
          if (btnUpper.isPressed() && btnLower.isPressed()) {
            if (!toggleProcessed) {
              if (togglePressTime == 0)
                togglePressTime = time_utils::micros();
              if (time_utils::micros() - togglePressTime >=
                  (UI_BUTTON_TOGGLE_DELAY_MS * 1000ULL)) {
                flags.endOfDay = !flags.endOfDay;
                toggleProcessed = true;
              }
            }
          } else {
            togglePressTime = 0;
            toggleProcessed = false;
          }
          if (btnCenter.rose()) {
            if (MotorWrapper::canStartMove()) {
              fsm_state.currentState = COMPRESSION;
            }
          }
        }
        if (Refill::hasError()) {
          fsm_state.currentState = ERROR_STATE;
          fsm_state.error = 0xFE;
        }
        break;
      }

      case COMPRESSION: {
        if (stateEntry) {
          motor.clearErrors(); // Clear stale stall/torque from previous runs
          vTaskDelay(pdMS_TO_TICKS(50));
          Compression::begin(Compression::MODE_1_TRAVEL);
          stateEntry = false;
        }
        if (Compression::update(motor)) {
          fsm_state.currentState = READY_TO_INJECT;
        }
        if (!ignoreButtons && !buttonLock && btnUpper.rose()) {
          fsm_state.currentState = REFILL;
          moveLockActive = true;
          Compression::reset();
        }
        if (!ignoreButtons && !buttonLock && btnLower.rose()) {
          fsm_state.currentState = READY_TO_INJECT;
          Compression::reset();
        }
        if (Compression::hasError()) {
          fsm_state.currentState = ERROR_STATE;
          fsm_state.error = 0xFD;
        }
        break;
      }

      case READY_TO_INJECT: {
        if (stateEntry) {
          ReadyToInject::begin();
          stateEntry = false;
        }
        ReadyToInject::update(motor);
        if (!ignoreButtons && btnUpper.isPressed() && btnLower.isPressed()) {
          if (MotorWrapper::canStartMove()) {
            fsm_state.currentState = PURGE_ZERO;
            ReadyToInject::reset();
          }
        } else if (!ignoreButtons && !buttonLock && btnCenter.rose()) {
          if (MotorWrapper::canStartMove()) {
            fsm_state.currentState = REFILL;
            moveLockActive = true;
            ReadyToInject::reset();
          }
        }
        if (ReadyToInject::hasError()) {
          fsm_state.currentState = ERROR_STATE;
          fsm_state.error = 0xFC;
        }
        break;
      }

      case PURGE_ZERO: {
        if (stateEntry) {
          motor.clearErrors();
          vTaskDelay(pdMS_TO_TICKS(50));
          PurgeZero::begin();
          stateEntry = false;
        }
        if (PurgeZero::update(motor, btnUpper.isPressed(), btnLower.isPressed(),
                              btnCenter.rose())) {
          if (MotorWrapper::canStartMove()) {
            fsm_state.currentState = ANTIDRIP;
            PurgeZero::reset();
          }
        }
        if (PurgeZero::hasError()) {
          fsm_state.currentState = ERROR_STATE;
          fsm_state.error = 0xFB;
        }
        break;
      }

      case ANTIDRIP: {
        if (stateEntry) {
          AntiDrip::begin();
          stateEntry = false;
        }
        AntiDrip::update(motor);
        if (!ignoreButtons && btnCenter.isPressed() && btnLower.isPressed()) {
          if (MotorWrapper::canStartMove()) {
            fsm_state.currentState = INJECT;
            AntiDrip::reset();
          }
        } else if (!ignoreButtons && !buttonLock && btnUpper.rose()) {
          fsm_state.currentState = READY_TO_INJECT;
          AntiDrip::reset();
        }
        if (AntiDrip::hasError()) {
          fsm_state.currentState = ERROR_STATE;
          fsm_state.error = 0xFA;
        }
        break;
      }

      case INJECT: {
        if (stateEntry) {
          Injection::begin(currentMould);
          stateEntry = false;
        }
        if (Injection::update(motor)) {
          fsm_state.currentState = HOLD_INJECTION;
        }
        if (!injectTargetLogged) {
          if (Injection::isTorqueMode()) {
            ESP_LOGI(TAG, "INJECT_TARGET: mode=3D torque=%.2f",
                     Injection::getInjectTorque());
          } else {
            ESP_LOGI(TAG, "INJECT_TARGET: purgeZero=%.2f target=%.2f vol=%.2f",
                     PurgeZero::getPurgeZeroPosition(),
                     Injection::getInjectTargetPos(),
                     Injection::getTargetInjectVolume());
          }
          injectTargetLogged = true;
        }
        if (!ignoreButtons && !buttonLock && btnUpper.rose()) {
          if (MotorWrapper::canStartMove()) {
            fsm_state.currentState = RELEASE;
            Injection::reset();
          }
        }
        if (Injection::hasError()) {
          fsm_state.currentState = ERROR_STATE;
          fsm_state.error = 0xF9;
        }
        break;
      }

      case HOLD_INJECTION: {
        Injection::update(motor);
        if (Injection::isComplete()) {
          if (MotorWrapper::canStartMove()) {
            fsm_state.currentState = RELEASE;
            Injection::reset();
          }
        }
        if (!ignoreButtons && !buttonLock && btnUpper.rose()) {
          if (MotorWrapper::canStartMove()) {
            fsm_state.currentState = RELEASE;
            Injection::reset();
          }
        }
        break;
      }

      case RELEASE: {
        if (stateEntry) {
          BroadcastDataStore &bds = BroadcastDataStore::getInstance();
          MotorWrapper::setMotorLimits(motor, RELEASE_CONTROLLER_VEL_LIMIT,
                                       commonParams.releaseCurrentLimit,
                                       MODULE_RELEASE, "RELEASE");
          MotorWrapper::setTrapTrajParams(
              motor, commonParams.releaseTrapVelLimit,
              commonParams.trapTrajAccelDecel, commonParams.trapTrajAccelDecel,
              MODULE_RELEASE, "RELEASE_TRAJ");
          float releaseTarget = bds.getPosition() + commonParams.releaseDist;
          if (MotorWrapper::setModeAndMove(motor, 3, 5, releaseTarget,
                                           MODULE_RELEASE, "Pos Release")) {
            stateEntry = false;
          }
        }
        if (!stateEntry && time_utils::micros() - stateTimer > 2000000ULL) {
          fsm_state.currentState = CONFIRM_MOULD_REMOVAL;
        }
        break;
      }

      case CONFIRM_MOULD_REMOVAL: {
        static uint64_t confirmButtonTime = 0;
        if (!ignoreButtons && !buttonLock &&
            (btnUpper.rose() || btnLower.rose())) {
          if (confirmButtonTime == 0)
            confirmButtonTime = time_utils::micros();
        }
        if (confirmButtonTime > 0 &&
            (time_utils::micros() - confirmButtonTime >= 1000000ULL)) {
          if (flags.endOfDay) {
            fsm_state.currentState = READY_TO_INJECT;
          } else {
            if (MotorWrapper::canStartMove()) {
              fsm_state.currentState = REFILL;
              moveLockActive = true;
            }
          }
          confirmButtonTime = 0;
        }
        break;
      }
      }
    }

    // LED update (placeholder)
    leds_update(fsm_state.currentState, flags.endOfDay,
                btnUpper.isPressed() || btnCenter.isPressed() ||
                    btnLower.isPressed(),
                safety.isMotorPowerEnabled());

    if (time_utils::micros() - lastDebugTime > 1000000ULL) {
      lastDebugTime = time_utils::micros();
      if (!safety.isMotorPowerEnabled()) {
        long prRaw = safety.getLoadCellRaw();
        ESP_LOGI(TAG, "TEMP_ONLY Temp:%d Contactor:%s Pr:%ld Cyc:%u",
                 fsm_inputs.nozzleTemperature,
                 safety.isMotorPowerEnabled() ? "ON" : "OFF", prRaw,
                 safety.getContactorCycles());
      } else {
        BroadcastDataStore &bds = BroadcastDataStore::getInstance();
        ESP_LOGI(TAG,
                 "State:%s Pos:%.2f Vel:%.2f Temp:%.0f Err:0x%X IQ_s:%.2f "
                 "IQ_m:%.2f Pr:%ld V:%.2f I:%.2f Cyc:%u",
                 getStateName(fsm_state.currentState), peakPos, peakVel,
                 peakTemp, fsm_state.error, peakIqSet, peakIqMeas, peakPressure,
                 peakBusV, peakBusI, safety.getContactorCycles());
      }
      peakInit = false;
    }

  loop_end:
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
