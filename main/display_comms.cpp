#include "display_comms.h"

#if APP_DISPLAY_UART

#include "broadcast_data_store.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "safety_manager.h"
#include "time_utils.h"
#include <cmath>
#include <cstdlib>
#include <cstring>

static const char *TAG = "DISPLAY_COMMS";

extern commonInjectParams_t commonParams;
extern actualMouldParams_t currentMould;
extern fsm_state_t fsm_state;
extern MachineFlags flags;
extern SafetyManager safety;
extern volatile InjectorStates requestedRemoteState;

namespace DisplayComms {

static unsigned long broadcastInterval = DISPLAY_BROADCAST_INTERVAL_MS;
static char rxBuffer[512];
static size_t rxLen = 0;

struct DisplayCommsState {
  unsigned long lastEncoderBroadcast;
  InjectorStates lastStateBroadcast;
  uint16_t lastErrorBroadcast;
};

static DisplayCommsState state = {0, InjectorStates::ERROR_STATE, 0xFFFF};

static void uart_send(const char *msg) {
  if (!msg)
    return;
  // ESP_LOGI(TAG, "TX> %s", msg);
  uart_write_bytes(UART_NUM_2, msg, strlen(msg));
}

static bool isSafeForParamUpdate() {
  switch (fsm_state.currentState) {
  case INIT_HEATING:
  case INIT_HOT_NOT_HOMED:
  case REFILL:
  case READY_TO_INJECT:
  case PURGE_ZERO:
  case CONFIRM_MOULD_REMOVAL:
    break;
  default:
    return false;
  }
  BroadcastDataStore &bds = BroadcastDataStore::getInstance();
  return std::fabs(bds.getVelocity()) < 0.1f;
}

static const char *getStateName(InjectorStates st) {
  switch (st) {
  case INIT_HEATING:
    return "INIT_HEATING";
  case INIT_HOT_NOT_HOMED:
    return "INIT_HOT_WAIT";
  case INIT_HOMING:
    return "INIT_HOMING";
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
    return "HOLD_INJECTION";
  case RELEASE:
    return "RELEASE";
  case CONFIRM_MOULD_REMOVAL:
    return "CONFIRM_REMOVAL";
  case ERROR_STATE:
    return "ERROR_STATE";
  default:
    return "UNKNOWN";
  }
}

static const char *getErrorName(uint16_t err) {
  switch (err) {
  case ERR_NONE:
    return "NONE";
  case ERR_ESTOP:
    return "E-STOP";
  case ERR_BARREL_POSITION_LOST:
    return "BARREL_OPEN_OR_LOST";
  case ERR_NOZZLE_NOT_BLOCKED:
    return "NOZZLE_NOT_BLOCKED";
  case ERR_OVER_TEMP:
    return "OVER_TEMP";
  case ERR_HARD_LIMIT:
    return "HARD_LIMIT";
  case ERR_UNDER_TEMP:
    return "UNDER_TEMP";
  case ERR_BOTTOM_ENDSTOP_COLLISION:
    return "BOTTOM_ENDSTOP";
  case ERR_TOP_ENDSTOP_COLLISION:
    return "TOP_ENDSTOP";
  case ERR_BROADCAST_STALE:
    return "COMMS_STALE";
  case ERR_CAN_RTR_FAILURE:
    return "CAN_MOTOR_ERROR";
  default:
    return "UNKNOWN_ERROR";
  }
}

void begin() {
  uart_config_t uart_config = {};
  uart_config.baud_rate = DISPLAY_BAUD_RATE;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_APB;

  uart_param_config(UART_NUM_2, &uart_config);
  uart_set_pin(UART_NUM_2, PIN_UART_TX, PIN_UART_RX, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_2, 1024, 0, 0, nullptr, 0);

  rxLen = 0;
  state.lastEncoderBroadcast = 0;
  state.lastStateBroadcast = InjectorStates::ERROR_STATE;
  state.lastErrorBroadcast = 0xFFFF;

  ESP_LOGI(TAG, "UART2 initialized (TX=%d RX=%d baud=%d)", PIN_UART_TX,
           PIN_UART_RX, DISPLAY_BAUD_RATE);
}

void update() {
  uint8_t data[64];
  int len = uart_read_bytes(UART_NUM_2, data, sizeof(data), 0);
  for (int i = 0; i < len; ++i) {
    char c = (char)data[i];
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuffer[rxLen] = '\0';
        parseIncomingMessage(rxBuffer);
        rxLen = 0;
      }
    } else {
      if (rxLen < sizeof(rxBuffer) - 1) {
        rxBuffer[rxLen++] = c;
      } else {
        rxLen = 0; // overflow, reset
      }
    }
  }

  unsigned long now = (unsigned long)time_utils::millis();
  if (now - state.lastEncoderBroadcast >= broadcastInterval) {
    BroadcastDataStore &bds = BroadcastDataStore::getInstance();
    if (fsm_state.currentState != InjectorStates::ERROR_STATE &&
        flags.initialHomingDone) {
      broadcastEncoder(bds.getPosition());
    }
    broadcastTemp(bds.getTemperature());
    broadcastState(fsm_state.currentState);
    broadcastEndOfDay(flags.endOfDay);
    state.lastEncoderBroadcast = now;
  }
}

void broadcastEncoder(float position) {
  char msg[48];
  snprintf(msg, sizeof(msg), "ENC|%.3f\n", position);
  uart_send(msg);
}

void broadcastTemp(float tempC) {
  char msg[48];
  snprintf(msg, sizeof(msg), "TEMP|%.2f\n", tempC);
  uart_send(msg);
}

void broadcastState(InjectorStates st) {
  if (st == state.lastStateBroadcast)
    return;
  char msg[96];
  snprintf(msg, sizeof(msg), "STATE|%s|%llu\n", getStateName(st),
           (unsigned long long)time_utils::millis());
  uart_send(msg);
  state.lastStateBroadcast = st;
}

void broadcastError(uint16_t errorCode, const char *errorMsg) {
  if (errorCode == state.lastErrorBroadcast)
    return;
  const char *finalMsg =
      (errorMsg && errorMsg[0] != '\0') ? errorMsg : getErrorName(errorCode);
  char msg[128];
  snprintf(msg, sizeof(msg), "ERROR|0x%X|%s\n", errorCode, finalMsg);
  uart_send(msg);
  state.lastErrorBroadcast = errorCode;
}

void broadcastEndOfDay(bool active) {
  char msg[16];
  snprintf(msg, sizeof(msg), "EOD|%d\n", active ? 1 : 0);
  uart_send(msg);
}

void sendMouldParamsConfirm(const actualMouldParams_t &params) {
  char msg[512];
  const char *modeStr = (params.injectMode == INJECT_MODE_3D) ? "3D" : "2D";
  snprintf(msg, sizeof(msg),
           "MOULD_OK|%s|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|"
           "%.3f|%s|%.3f",
           params.mouldName, params.fillVolume, params.fillSpeed,
           params.fillPressure, params.packVolume, params.packSpeed,
           params.packPressure, params.packTime, params.fillTrapAccel,
           params.fillTrapDecel, params.packTrapAccel, params.packTrapDecel,
           modeStr, params.injectTorque);
  uart_send(msg);
}

void sendCommonParamsConfirm() {
  char msg[512];
  snprintf(msg, sizeof(msg),
           "COMMON_OK|%.3f|%.3f|%lu|%lu|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%."
           "3f|%lu|%lu\n",
           commonParams.trapTrajAccelDecel, commonParams.compressTravelTorque,
           (unsigned long)commonParams.microIntervalMs,
           (unsigned long)commonParams.microDurationMs, commonParams.purgeVelUp,
           commonParams.purgeVelDown, commonParams.purgeCurrentLimit,
           commonParams.antidripVel, commonParams.antidripCurrentLimit,
           commonParams.releaseDist, commonParams.releaseTrapVelLimit,
           commonParams.releaseCurrentLimit,
           (unsigned long)safety.getContactorCycles(),
           (unsigned long)CONTACTOR_CYCLE_LIMIT);
  uart_send(msg);
}

static const char *next_token(const char *str, char *out, size_t out_len,
                              char delim) {
  if (!str || !out || out_len == 0)
    return nullptr;
  const char *p = strchr(str, delim);
  if (p) {
    size_t len = (size_t)(p - str);
    if (len >= out_len)
      len = out_len - 1;
    memcpy(out, str, len);
    out[len] = '\0';
    return p + 1;
  }
  strncpy(out, str, out_len - 1);
  out[out_len - 1] = '\0';
  return nullptr;
}

void parseIncomingMessage(const char *message) {
  if (!message || message[0] == '\0')
    return;

  char cmd[32];
  const char *rest = next_token(message, cmd, sizeof(cmd), '|');

  if (strcmp(cmd, "CMD") == 0) {
    char subcmd[32];
    rest = next_token(rest, subcmd, sizeof(subcmd), '|');
    if (strcmp(subcmd, "GOTO") == 0) {
      char targetState[32];
      next_token(rest, targetState, sizeof(targetState), '|');
      if (strcmp(targetState, "REFILL") == 0)
        requestedRemoteState = REFILL;
      else if (strcmp(targetState, "COMPRESSION") == 0)
        requestedRemoteState = COMPRESSION;
      else if (strcmp(targetState, "READY_TO_INJECT") == 0)
        requestedRemoteState = READY_TO_INJECT;
      else if (strcmp(targetState, "PURGE_ZERO") == 0)
        requestedRemoteState = PURGE_ZERO;
      else if (strcmp(targetState, "HOME") == 0)
        requestedRemoteState = INIT_HOMING;
      else
        ESP_LOGW(TAG, "Unknown GOTO state: %s", targetState);
    } else if (strcmp(subcmd, "TOGGLE") == 0) {
      char toggleTarget[32];
      next_token(rest, toggleTarget, sizeof(toggleTarget), '|');
      if (strcmp(toggleTarget, "EOD") == 0) {
        flags.endOfDay = !flags.endOfDay;
        broadcastEndOfDay(flags.endOfDay);
        ESP_LOGI(TAG, "Toggled EOD to %d", flags.endOfDay);
      }
    }
    return;
  }

  if (strcmp(cmd, "MOULD") == 0) {
    if (!isSafeForParamUpdate()) {
      ESP_LOGW(TAG, "MOULD ignored: unsafe state=%s",
               getStateName(fsm_state.currentState));
      return;
    }
    actualMouldParams_t newParams = currentMould;
    char field[64];
    int fieldIdx = 0;
    while (rest) {
      rest = next_token(rest, field, sizeof(field), '|');
      switch (fieldIdx) {
      case 0:
        strncpy(newParams.mouldName, field, sizeof(newParams.mouldName) - 1);
        newParams.mouldName[sizeof(newParams.mouldName) - 1] = '\0';
        break;
      case 1:
        newParams.fillVolume = (float)atof(field);
        break;
      case 2:
        newParams.fillSpeed = (float)atof(field);
        break;
      case 3:
        newParams.fillPressure = (float)atof(field);
        break;
      case 4:
        newParams.packVolume = (float)atof(field);
        break;
      case 5:
        newParams.packSpeed = (float)atof(field);
        break;
      case 6:
        newParams.packPressure = (float)atof(field);
        break;
      case 7:
        newParams.packTime = (float)atof(field);
        break;
      case 8:
        newParams.fillTrapAccel = (float)atof(field);
        break;
      case 9:
        newParams.fillTrapDecel = (float)atof(field);
        break;
      case 10:
        newParams.packTrapAccel = (float)atof(field);
        break;
      case 11:
        newParams.packTrapDecel = (float)atof(field);
        break;
      case 12:
        if (field[0] == '3' || field[0] == 'T' || field[0] == 't')
          newParams.injectMode = INJECT_MODE_3D;
        else
          newParams.injectMode = INJECT_MODE_2D;
        break;
      case 13:
        newParams.injectTorque = (float)atof(field);
        break;
      default:
        break;
      }
      fieldIdx++;
    }

    if (fieldIdx >= 13) {
      currentMould = newParams;
      sendMouldParamsConfirm(currentMould);
    } else {
      ESP_LOGW(TAG, "MOULD parse failed (fields=%d)", fieldIdx);
    }
    return;
  }

  if (strcmp(cmd, "COMMON") == 0) {
    if (!isSafeForParamUpdate()) {
      ESP_LOGW(TAG, "COMMON ignored: unsafe state=%s",
               getStateName(fsm_state.currentState));
      return;
    }
    commonInjectParams_t newParams = commonParams;
    char field[64];
    int fieldIdx = 0;
    while (rest) {
      rest = next_token(rest, field, sizeof(field), '|');
      switch (fieldIdx) {
      case 0:
        newParams.trapTrajAccelDecel = (float)atof(field);
        break;
      case 1:
        newParams.compressTravelTorque = (float)atof(field);
        break;
      case 2:
        newParams.microIntervalMs = (uint32_t)atol(field);
        break;
      case 3:
        newParams.microDurationMs = (uint32_t)atol(field);
        break;
      case 4:
        newParams.purgeVelUp = (float)atof(field);
        break;
      case 5:
        newParams.purgeVelDown = (float)atof(field);
        break;
      case 6:
        newParams.purgeCurrentLimit = (float)atof(field);
        break;
      case 7:
        newParams.antidripVel = (float)atof(field);
        break;
      case 8:
        newParams.antidripCurrentLimit = (float)atof(field);
        break;
      case 9:
        newParams.releaseDist = (float)atof(field);
        break;
      case 10:
        newParams.releaseTrapVelLimit = (float)atof(field);
        break;
      case 11:
        newParams.releaseCurrentLimit = (float)atof(field);
        break;
      default:
        break;
      }
      fieldIdx++;
    }
    if (fieldIdx >= 12) {
      commonParams = newParams;
      sendCommonParamsConfirm();
    } else {
      ESP_LOGW(TAG, "COMMON parse failed (fields=%d)", fieldIdx);
    }
    return;
  }

  if (strcmp(cmd, "QUERY_MOULD") == 0) {
    sendMouldParamsConfirm(currentMould);
    return;
  }

  if (strcmp(cmd, "QUERY_COMMON") == 0) {
    sendCommonParamsConfirm();
    return;
  }

  if (strcmp(cmd, "QUERY_STATE") == 0) {
    broadcastState(fsm_state.currentState);
    return;
  }

  if (strcmp(cmd, "QUERY_ERROR") == 0) {
    broadcastError(fsm_state.error, fsm_state.errorMsg);
    return;
  }

  // ESP_LOGW(TAG, "Unknown command: %s", cmd);
}

} // namespace DisplayComms

#endif // APP_DISPLAY_UART
