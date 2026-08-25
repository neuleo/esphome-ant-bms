#include "ant_bms_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/version.h"
#include <array>

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 12, 0)
#define ADDR_STR(x) x
#else
#define ADDR_STR(x) (x).c_str()
#endif

namespace esphome::ant_bms_ble {

static const char *const TAG = "ant_bms_ble";

#if ESPHOME_VERSION_CODE < VERSION_CODE(2026, 1, 0)
constexpr size_t format_hex_pretty_size(size_t byte_count) { return byte_count * 3; }

static char *format_hex_pretty_to(char *buffer, size_t buffer_size, const uint8_t *data, size_t length,
                                  char separator = ':') {
  if (length == 0) {
    buffer[0] = '\0';
    return buffer;
  }
  size_t max_bytes = buffer_size / 3;
  if (length > max_bytes)
    length = max_bytes;
  for (size_t i = 0; i < length; i++) {
    uint8_t hi = data[i] >> 4, lo = data[i] & 0x0F;
    buffer[3 * i] = hi >= 10 ? 'A' + (hi - 10) : '0' + hi;
    buffer[3 * i + 1] = lo >= 10 ? 'A' + (lo - 10) : '0' + lo;
    if (i != length - 1)
      buffer[3 * i + 2] = separator;
  }
  buffer[3 * length - 1] = '\0';
  return buffer;
}
#endif

static void log_hex_chunked(const char *tag, const uint8_t *data, size_t size) {
  char buf[format_hex_pretty_size(100)];
  for (size_t i = 0; i < size; i += 100) {
    size_t len = std::min<size_t>(100, size - i);
    ESP_LOGD(tag, "  %s", format_hex_pretty_to(buf, sizeof(buf), data + i, len, '.'));
  }
}

static const uint8_t MAX_NO_RESPONSE_COUNT = 10;

static const uint16_t ANT_BMS_SERVICE_UUID = 0xFFE0;
static const uint16_t ANT_BMS_CHARACTERISTIC_UUID = 0xFFE1;  // Handle 0x10

static const uint16_t MAX_RESPONSE_SIZE = 192;
static const uint16_t MIN_RESPONSE_SIZE = 10;

static const uint8_t ANT_PKT_START_1 = 0x7E;
static const uint8_t ANT_PKT_START_2 = 0xA1;
static const uint8_t ANT_FRAME_TYPE_STATUS = 0x01;
static const uint8_t ANT_FRAME_TYPE_DEVICE_INFO = 0x02;
static const uint8_t ANT_COMMAND_STATUS = 0x01;
static const uint8_t ANT_COMMAND_DEVICE_INFO = 0x02;
static const uint8_t ANT_COMMAND_CONTROL = 0x03;

static const uint8_t ANT_FUNCTION_DISCHARGE = 0x01;
static const uint8_t ANT_FUNCTION_CHARGE = 0x02;
static const uint8_t ANT_FUNCTION_BALANCER = 0x03;

static const uint8_t ANT_VALUE_OFF = 0x00;
static const uint8_t ANT_VALUE_ON = 0x01;

static const uint8_t ANT_REGISTER_MAX_CELL_VOLTAGE = 0x00;
static const uint8_t ANT_REGISTER_MIN_CELL_VOLTAGE = 0x02;
static const uint8_t ANT_REGISTER_MAX_DISCHARGE_CURRENT = 0x04;
static const uint8_t ANT_REGISTER_MAX_CHARGE_CURRENT = 0x06;

static const uint8_t BATTERY_STATUS_SIZE = 14;
static const char *const BATTERY_STATUS[BATTERY_STATUS_SIZE] = {
    "Normal",
    "Overcharge",
    "Over-discharge",
    "Overcurrent",
    "Overheat",
    "Low Temperature",
    "Reserved 6",
    "Reserved 7",
    "Discharge Overcurrent",
    "Short Circuit",
    "Discharge Low Temp",
    "Discharge Mosfet Abnormal",
    "Charge Mosfet Abnormal",
    "Unknown",
};

static const uint8_t MOS_STATE_SIZE = 6;
static const char *const MOS_STATE[MOS_STATE_SIZE] = {
    "Off",
    "On",
    "Overcharge",
    "Over-discharge",
    "Overcurrent",
    "Overheat",
};

static const uint8_t BALANCER_STATUS_SIZE = 11;
static const char *const BALANCER_STATUS[BALANCER_STATUS_SIZE] = {
    "Off",
    "Exceed Limit",
    "Charge Limit",
    "Complete",
    "Balancing",
    "Overheat",
    "Unknown 6",
    "Unknown 7",
    "Unknown 8",
    "Unknown 9",
    "Unknown 10",
};

struct SettingsAddress {
  uint16_t address;
  const char *name;
  float scale;
  const char *unit;
};

static const SettingsAddress SETTINGS_ADDRESSES[] = {
    {0x0000, "CellOvervoltageProtection", 0.001f, "V"},
    {0x0002, "CellOvervoltageRecovery", 0.001f, "V"},
    {0x0004, "CellUndervoltageProtection", 0.001f, "V"},
    {0x0006, "CellUndervoltageRecovery", 0.001f, "V"},
    {0x0008, "TotalVoltageOvervoltageProtection", 0.1f, "V"},
    {0x000a, "TotalVoltageOvervoltageRecovery", 0.1f, "V"},
    {0x000c, "TotalVoltageUndervoltageProtection", 0.1f, "V"},
    {0x000e, "TotalVoltageUndervoltageRecovery", 0.1f, "V"},
    {0x0010, "CellDifferentialVoltageProtection", 0.001f, "V"},
    {0x0012, "DischargeOvercurrentProtection", 0.1f, "A"},
    {0x0014, "DischargeOvercurrentDelay", 1.0f, "S"},
    {0x0016, "ChargeOvercurrentProtection", 0.1f, "A"},
    {0x0018, "ChargeOvercurrentDelay", 1.0f, "S"},
    {0x001a, "BalancedStartVoltage", 0.001f, "V"},
    {0x001c, "BalancedDifferentialVoltage", 0.001f, "V"},
    {0x001e, "PowerTubeOvertemperatureProtection", 1.0f, "°C"},
    {0x0020, "PowerTubeOvertemperatureRecovery", 1.0f, "°C"},
    {0x0022, "TemperatureSensor1OvertemperatureProtection", 1.0f, "°C"},
    {0x0024, "TemperatureSensor1OvertemperatureRecovery", 1.0f, "°C"},
    {0x0026, "TemperatureSensor2OvertemperatureProtection", 1.0f, "°C"},
    {0x0028, "TemperatureSensor2OvertemperatureRecovery", 1.0f, "°C"},
    {0x0032, "ChargeLowTemperatureProtection", 1.0f, "°C"},
    {0x0034, "ChargeLowTemperatureRecovery", 1.0f, "°C"},
    {0x0036, "DischargeLowTemperatureProtection", 1.0f, "°C"},
    {0x0038, "DischargeLowTemperatureRecovery", 1.0f, "°C"},
    {0x0040, "SecondaryProtectionCellOvervoltage", 0.001f, "V"},
    {0x0042, "SecondaryProtectionCellUndervoltage", 0.001f, "V"},
    {0x0044, "SecondaryProtectionTotalOvervoltage", 0.1f, "V"},
    {0x0046, "SecondaryProtectionTotalUndervoltage", 0.1f, "V"},
    {0x0048, "SecondaryProtectionDischargeOvercurrent", 0.1f, "A"},
    {0x004a, "SecondaryProtectionChargeOvercurrent", 0.1f, "A"},
    {0x004c, "SecondaryProtectionMosfetOvertemperature", 1.0f, "°C"},
    {0x004e, "SecondaryProtectionTemperatureSensor1Overtemperature", 1.0f, "°C"},
    {0x0050, "SecondaryProtectionTemperatureSensor2Overtemperature", 1.0f, "°C"},
    {0x0080, "PrechargeTime", 1.0f, "ms"},
    {0x0084, "BluetoothDisconnectionTime", 1.0f, "S"},
    {0x008a, "SecondaryDischargeOvercurrentDelay", 1.0f, "S"},
    {0x008c, "SecondaryChargeOvercurrentDelay", 1.0f, "S"},
    {0x0090, "ContinuousDischargeCurrentLimit", 0.1f, "A"},
    {0x0092, "ContinuousChargeCurrentLimit", 0.1f, "A"},
    {0x0094, "BatteryStrings", 1.0f, "strings"},
    {0x0096, "HardwareOvercurrentDelay", 1.0f, "uS"},
    {0x0098, "ShortCircuitDelay", 1.0f, "uS"},
    {0x009a, "HardwareShortCircuitDelay", 1.0f, "uS"},
    {0x009e, "DischargeOvercurrentDelay2", 1.0f, "S"},
    {0x00a0, "SecondaryDischargeOvercurrentDelay2", 1.0f, "S"},
    {0x00a2, "ChargeOvercurrentDelay2", 1.0f, "S"},
    {0x00a4, "SecondaryChargeOvercurrentDelay2", 1.0f, "S"},
    {0x00a6, "RemainingCapacity", 1.0f, "Ah"},
    {0x00aa, "TotalCycleCapacity", 1.0f, "Ah"},
    {0x00c4, "StateOfChargeMethod", 1.0f, ""},
    {0x017a, "TireLength", 1.0f, "mm"},
    {0x017c, "PulseValue", 1.0f, ""},
    {0x017e, "SecondaryModuleNum", 1.0f, ""},
};

#ifdef USE_ESP32
void AntBmsBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                    esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      // Automatically boost BLE TX power to maximum (+9dBm) on every connection
      esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
      esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
      esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
      esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9);
      esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P9);
      esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, ESP_PWR_LVL_P9);
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      this->node_state = espbt::ClientState::IDLE;

      if (this->characteristic_handle_ != 0) {
        auto status = esp_ble_gattc_unregister_for_notify(
            this->parent()->get_gattc_if(), this->parent()->get_remote_bda(), this->characteristic_handle_);
        if (status) {
          ESP_LOGW(TAG, "esp_ble_gattc_unregister_for_notify failed, status=%d", status);
        }
      }
      this->characteristic_handle_ = 0;

      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      auto *characteristic = this->parent_->get_characteristic(ANT_BMS_SERVICE_UUID, ANT_BMS_CHARACTERISTIC_UUID);
      if (characteristic == nullptr) {
        ESP_LOGE(TAG, "[%s] Characteristic not found", ADDR_STR(this->parent_->address_str()));
        break;
      }
      this->characteristic_handle_ = characteristic->handle;

      auto status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(), this->parent()->get_remote_bda(),
                                                      characteristic->handle);
      if (status) {
        ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed, status=%d", status);
      }

      auto *descr = this->parent_->get_config_descriptor(characteristic->handle);
      if (descr != nullptr) {
        uint8_t notify_en[2] = {0x01, 0x00};
        esp_ble_gattc_write_char_descr(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                       descr->handle, sizeof(notify_en), notify_en,
                                       ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
      }

      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      this->node_state = espbt::ClientState::ESTABLISHED;

      ESP_LOGI(TAG, "Request status frame");
      // 0x7e 0xa1 0x01 0x00 0x00 0xbe ...
      this->send_(ANT_COMMAND_STATUS, 0x0000, 0xbe, false);

      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle != this->characteristic_handle_)
        break;

      ESP_LOGVV(TAG, "Notification received (handle 0x%02X):", param->notify.handle);
      ESP_LOGVV(TAG, "  %s",
                format_hex_pretty_to(this->log_buffer_, sizeof(this->log_buffer_), param->notify.value,
                                     param->notify.value_len, '.'));

      this->assemble_(param->notify.value, param->notify.value_len);
      break;
    }
    default:
      break;
  }
}

void AntBmsBle::assemble_(const uint8_t *data, uint16_t length) {
  if (this->frame_buffer_.size() > MAX_RESPONSE_SIZE) {
    ESP_LOGW(TAG, "Maximum response size of %d bytes reached, resetting buffer...", MAX_RESPONSE_SIZE);
    this->frame_buffer_.clear();
  }

  if (this->frame_buffer_.empty()) {
    if (data[0] != ANT_PKT_START_1 || data[1] != ANT_PKT_START_2) {
      ESP_LOGVV(TAG, "Received invalid header 0x%02X 0x%02X, waiting for 0x%02X 0x%02X", data[0], data[1],
                ANT_PKT_START_1, ANT_PKT_START_2);
      return;
    }
  }

  this->frame_buffer_.insert(this->frame_buffer_.end(), data, data + length);

  if (this->frame_buffer_.size() < MIN_RESPONSE_SIZE) {
    return;
  }

  if (this->frame_buffer_.size() < 4) {
    return;
  }

  uint8_t length_expected = this->frame_buffer_[3] + 4;
  if (this->frame_buffer_.size() < length_expected) {
    return;
  }

  uint8_t end_byte_1 = this->frame_buffer_[length_expected - 2];
  uint8_t end_byte_2 = this->frame_buffer_[length_expected - 1];

  if (end_byte_1 == 0xAA && end_byte_2 == 0x55) {
    uint8_t function = this->frame_buffer_[2];
    uint16_t computed_crc = crc16(this->frame_buffer_.data() + 1, length_expected - 5);
    uint16_t remote_crc = uint16_t(this->frame_buffer_[length_expected - 4]) |
                          (uint16_t(this->frame_buffer_[length_expected - 3]) << 8);

    if (computed_crc != remote_crc) {
      ESP_LOGW(TAG, "CRC check failed! 0x%04X != 0x%04X", computed_crc, remote_crc);
      this->frame_buffer_.clear();
      return;
    }

    std::vector<uint8_t> data(this->frame_buffer_.begin(), this->frame_buffer_.end());

    this->on_ant_bms_ble_data_(function, data);
    this->frame_buffer_.clear();
  }
}

#ifdef USE_ESP32
void AntBmsBle::update() {
  this->track_online_status_();
  if (this->node_state != espbt::ClientState::ESTABLISHED) {
    ESP_LOGW(TAG, "[%s] Not connected", ADDR_STR(this->parent_->address_str()));
    return;
  }

  // 0x7e 0xa1 0x01 0x00 0x00 0xbe 0x18 0x55 0xaa 0x55
  this->send_(ANT_COMMAND_STATUS, 0x0000, 0xbe, false);
}

#else
void AntBmsBle::update() {}
#endif  // USE_ESP32

void AntBmsBle::on_ant_bms_ble_data_(const uint8_t &function, const std::vector<uint8_t> &data) {
  this->reset_online_status_tracker_();

  switch (function) {
    case ANT_FRAME_TYPE_STATUS:
      this->on_status_data_(data);
      break;
    case ANT_FRAME_TYPE_DEVICE_INFO: {
      if (data.size() < 5)
        break;
      uint16_t addr = data[3] | (uint16_t(data[4]) << 8);
      if (addr == 0x026c) {
        this->on_device_info_data_(data);
      } else {
        this->on_settings_data_(data);
      }
      break;
    }
    default:
      ESP_LOGW(TAG, "Unhandled response received (function 0x%02X): %s", function,
               format_hex_pretty_to(this->log_buffer_, sizeof(this->log_buffer_), data.data(), data.size(), '.'));
  }
}

void AntBmsBle::on_status_data_(const std::vector<uint8_t> &data) {
  auto ant_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 0]) << 8) | (uint16_t(data[i + 1]) << 0);
  };
  auto ant_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(ant_get_16bit(i + 0)) << 16) | (uint32_t(ant_get_16bit(i + 2)) << 0);
  };

  ESP_LOGI(TAG, "Status frame (%d bytes) received", data.size());
  log_hex_chunked(TAG, data.data(), data.size());

  //  4  2  0x0080 Total voltage (0.1V)
  //  6 64  0x0000 Cell voltage 1..32 (0.001V)
  // 70  4  0x0000 Current (0.1A) (sign: see current_sign_inversion)
  // 74  1    0x64 SoC (1%)
  // 75  4  0x0000 Total capacity (0.000001Ah)
  // 79  4  0x0000 Remaining capacity (0.000001Ah)
  // 83  4  0x0000 Battery cycle capacity (0.001Ah)
  // 87  4  0x0000 Total runtime (1s)
  // 91  2  0x0000 Temperature 1 (1°C)
  // 93  2  0x0000 Temperature 2 (1°C)
  // 95  2  0x0000 Temperature 3 (1°C)
  // 97  2  0x0000 Temperature 4 (1°C)
  // 99  2  0x0000 Temperature 5 / Mosfet (1°C)
  // 101 2  0x0000 Temperature 6 / Balancer (1°C)
  // 103 1    0x00 Charge mosfet status (1: on, 2: off)
  // 104 1    0x00 Discharge mosfet status (1: on, 2: off)
  // 105 1    0x00 Balancer status (0: off, 1: on)
  // 106 2  0x0000 Tire length (1mm)
  // 108 2  0x0000 Pulse value (1)
  // 110 1    0x00 State Of Health (1%)
  // 111 4  0x0000 Total discharging capacity (0.001Ah)
  // 115 4  0x0000 Total charging capacity (0.001Ah)
  // 119 4  0x0000 Total discharging time (1s)
  // 123 4  0x0000 Total charging time (1s)
  // 127 4  0x0000 Balanced cell bitmask (1..32)
  // 131 2  0x0000 Unknown
  // 133 2  0x0000 Battery strings (1)
  // 135 2  0x0000 Battery status code (bitmask)
  // 137 2  0x0000 Average cell voltage (0.001V)
  // 139 1    0x00 Max cell voltage cell index (1..32)
  // 140 2  0x0000 Max cell voltage (0.001V)
  // 142 1    0x00 Min cell voltage cell index (1..32)
  // 143 2  0x0000 Min cell voltage (0.001V)

  float total_voltage = ant_get_16bit(4) * 0.1f;
  this->publish_state_(this->total_voltage_sensor_, total_voltage);

  float min_cell_voltage = 100.0f;
  float max_cell_voltage = -100.0f;
  for (uint8_t i = 0; i < this->cells_.size(); i++) {
    float cell_voltage = (float) ant_get_16bit(i * 2 + 6) * 0.001f;
    if (cell_voltage > 0) {
      min_cell_voltage = std::min(min_cell_voltage, cell_voltage);
      max_cell_voltage = std::max(max_cell_voltage, cell_voltage);
    }
    this->publish_state_(this->cells_[i].cell_voltage_sensor_, cell_voltage);
  }

  this->publish_state_(this->min_cell_voltage_sensor_, min_cell_voltage);
  this->publish_state_(this->max_cell_voltage_sensor_, max_cell_voltage);
  this->publish_state_(this->delta_cell_voltage_sensor_, max_cell_voltage - min_cell_voltage);

  float current = (float) ((int32_t) ant_get_32bit(70)) * 0.1f;
  this->publish_state_(this->current_sensor_, current);
  this->publish_state_(this->power_sensor_, current * total_voltage);

  this->publish_state_(this->soc_sensor_, data[74]);

  this->publish_state_(this->total_battery_capacity_setting_sensor_, (float) ant_get_32bit(75) * 0.000001f);
  this->publish_state_(this->capacity_remaining_sensor_, (float) ant_get_32bit(79) * 0.000001f);
  this->publish_state_(this->battery_cycle_capacity_sensor_, (float) ant_get_32bit(83) * 0.001f);

  uint32_t total_runtime = ant_get_32bit(87);
  this->publish_state_(this->total_runtime_sensor_, (float) total_runtime);
  this->publish_state_(this->total_runtime_formatted_text_sensor_, this->format_total_runtime_(total_runtime));

  for (uint8_t i = 0; i < this->temperatures_.size(); i++) {
    this->publish_state_(this->temperatures_[i].temperature_sensor_, (float) ((int16_t) ant_get_16bit(i * 2 + 91)));
  }

  uint8_t charge_mosfet_status_code = data[103];
  this->publish_state_(this->charge_mosfet_status_code_sensor_, (float) charge_mosfet_status_code);
  this->publish_state_(this->charge_mosfet_status_text_sensor_,
                       charge_mosfet_status_code < MOS_STATE_SIZE ? MOS_STATE[charge_mosfet_status_code] : "Unknown");
  this->publish_state_(this->charge_mosfet_switch_, charge_mosfet_status_code == 1);

  uint8_t discharge_mosfet_status_code = data[104];
  this->publish_state_(this->discharge_mosfet_status_code_sensor_, (float) discharge_mosfet_status_code);
  this->publish_state_(
      this->discharge_mosfet_status_text_sensor_,
      discharge_mosfet_status_code < MOS_STATE_SIZE ? MOS_STATE[discharge_mosfet_status_code] : "Unknown");
  this->publish_state_(this->discharge_mosfet_switch_, discharge_mosfet_status_code == 1);

  uint8_t balancer_status_code = data[105];
  this->publish_state_(this->balancer_status_code_sensor_, (float) balancer_status_code);
  this->publish_state_(this->balancer_status_text_sensor_, balancer_status_code < BALANCER_STATUS_SIZE
                                                               ? BALANCER_STATUS[balancer_status_code]
                                                               : "Unknown");
  this->publish_state_(this->balancer_switch_, balancer_status_code == 1);

  this->publish_state_(this->state_of_health_sensor_, data[110]);

  this->publish_state_(this->total_discharging_capacity_sensor_, (float) ant_get_32bit(111) * 0.001f);
  this->publish_state_(this->total_charging_capacity_sensor_, (float) ant_get_32bit(115) * 0.001f);

  uint32_t total_discharging_time = ant_get_32bit(119);
  this->publish_state_(this->total_discharging_time_sensor_, (float) total_discharging_time);
  this->publish_state_(this->total_discharging_time_formatted_text_sensor_,
                       this->format_total_runtime_(total_discharging_time));

  uint32_t total_charging_time = ant_get_32bit(123);
  this->publish_state_(this->total_charging_time_sensor_, (float) total_charging_time);
  this->publish_state_(this->total_charging_time_formatted_text_sensor_,
                       this->format_total_runtime_(total_charging_time));

  this->publish_state_(this->balanced_cell_bitmask_sensor_, (float) ant_get_32bit(127));

  this->publish_state_(this->battery_strings_sensor_, ant_get_16bit(133));

  uint16_t battery_status_code = ant_get_16bit(135);
  this->publish_state_(this->battery_status_code_sensor_, (float) battery_status_code);
  this->publish_state_(
      this->battery_status_text_sensor_,
      battery_status_code < BATTERY_STATUS_SIZE ? BATTERY_STATUS[battery_status_code] : "Unknown");

  this->publish_state_(this->average_cell_voltage_sensor_, (float) ant_get_16bit(137) * 0.001f);

  this->publish_state_(this->max_voltage_cell_sensor_, (float) data[139]);
  this->publish_state_(this->min_voltage_cell_sensor_, (float) data[142]);
}

void AntBmsBle::on_device_info_data_(const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "Device info frame received");
  log_hex_chunked(TAG, data.data(), data.size());

  // 140 16 0x0000 Device model
  // 156  8 0x0000 Software version

  std::string device_model(data.begin() + 140, data.begin() + 140 + 16);
  this->publish_state_(this->device_model_text_sensor_, device_model);

  std::string software_version(data.begin() + 156, data.begin() + 156 + 8);
  this->publish_state_(this->software_version_text_sensor_, software_version);
}

void AntBmsBle::on_settings_data_(const std::vector<uint8_t> &data) {
  auto ant_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 0]) << 0) | (uint16_t(data[i + 1]) << 8);
  };
  auto ant_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(ant_get_16bit(i + 0)) << 0) | (uint32_t(ant_get_16bit(i + 2)) << 16);
  };

  ESP_LOGI(TAG, "Settings frame received");
  log_hex_chunked(TAG, data.data(), data.size());

  uint16_t address = ant_get_16bit(3);
  for (auto &setting : SETTINGS_ADDRESSES) {
    if (setting.address == address) {
      float value = (float) ant_get_16bit(5) * setting.scale;
      ESP_LOGI(TAG, "Setting %s (0x%04X): %.3f %s", setting.name, address, value, setting.unit);
      return;
    }
  }
  ESP_LOGW(TAG, "Unknown settings address: 0x%04X", address);
}

void AntBmsBle::track_online_status_() {
  if (this->no_response_count_ < MAX_NO_RESPONSE_COUNT) {
    this->no_response_count_++;
  }
  if (this->no_response_count_ == MAX_NO_RESPONSE_COUNT) {
    this->publish_device_unavailable_();
    this->no_response_count_++;
  }
}

void AntBmsBle::reset_online_status_tracker_() {
  this->no_response_count_ = 0;
  this->publish_state_(this->online_status_binary_sensor_, true);
}

void AntBmsBle::publish_device_unavailable_() {
  this->publish_state_(this->online_status_binary_sensor_, false);
}

void AntBmsBle::dump_config() {
  ESP_LOGCONFIG(TAG, "AntBmsBle:");

  LOG_SENSOR("", "Battery Strings", this->battery_strings_sensor_);
  LOG_SENSOR("", "Total Voltage", this->total_voltage_sensor_);
  LOG_SENSOR("", "Total Runtime", this->total_runtime_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "SoC", this->soc_sensor_);
  LOG_SENSOR("", "Total Battery Capacity Setting", this->total_battery_capacity_setting_sensor_);
  LOG_SENSOR("", "Capacity Remaining", this->capacity_remaining_sensor_);
  LOG_SENSOR("", "Battery Cycle Capacity", this->battery_cycle_capacity_sensor_);
  LOG_SENSOR("", "Average Cell Voltage", this->average_cell_voltage_sensor_);
  LOG_SENSOR("", "Delta Cell Voltage", this->delta_cell_voltage_sensor_);
  LOG_SENSOR("", "Min Cell Voltage", this->min_cell_voltage_sensor_);
  LOG_SENSOR("", "Max Cell Voltage", this->max_cell_voltage_sensor_);
  LOG_SENSOR("", "Min Voltage Cell", this->min_voltage_cell_sensor_);
  LOG_SENSOR("", "Max Voltage Cell", this->max_voltage_cell_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
  LOG_SENSOR("", "Cell Voltage 1", this->cells_[0].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 2", this->cells_[1].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 3", this->cells_[2].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 4", this->cells_[3].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 5", this->cells_[4].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 6", this->cells_[5].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 7", this->cells_[6].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 8", this->cells_[7].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 9", this->cells_[8].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 10", this->cells_[9].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 11", this->cells_[10].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 12", this->cells_[11].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 13", this->cells_[12].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 14", this->cells_[13].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 15", this->cells_[14].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 16", this->cells_[15].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 17", this->cells_[16].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 18", this->cells_[17].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 19", this->cells_[18].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 20", this->cells_[19].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 21", this->cells_[20].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 22", this->cells_[21].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 23", this->cells_[22].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 24", this->cells_[23].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 25", this->cells_[24].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 26", this->cells_[25].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 27", this->cells_[26].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 28", this->cells_[27].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 29", this->cells_[28].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 30", this->cells_[29].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 31", this->cells_[30].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 32", this->cells_[31].cell_voltage_sensor_);
  LOG_SENSOR("", "Charge Mosfet Status Code", this->charge_mosfet_status_code_sensor_);
  LOG_SENSOR("", "Discharge Mosfet Status Code", this->discharge_mosfet_status_code_sensor_);
  LOG_SENSOR("", "Balancer Status Code", this->balancer_status_code_sensor_);
  LOG_SENSOR("", "State Of Health", this->state_of_health_sensor_);
  LOG_SENSOR("", "Battery Status Code", this->battery_status_code_sensor_);
  LOG_SENSOR("", "Total Discharging Capacity", this->total_discharging_capacity_sensor_);
  LOG_SENSOR("", "Total Charging Capacity", this->total_charging_capacity_sensor_);
  LOG_SENSOR("", "Total Discharging Time", this->total_discharging_time_sensor_);
  LOG_SENSOR("", "Total Charging Time", this->total_charging_time_sensor_);
  LOG_SENSOR("", "Balanced Cell Bitmask", this->balanced_cell_bitmask_sensor_);

  LOG_TEXT_SENSOR("", "Discharge Mosfet Status", this->discharge_mosfet_status_text_sensor_);
  LOG_TEXT_SENSOR("", "Charge Mosfet Status", this->charge_mosfet_status_text_sensor_);
  LOG_TEXT_SENSOR("", "Balancer Status", this->balancer_status_text_sensor_);
  LOG_TEXT_SENSOR("", "Total Runtime Formatted", this->total_runtime_formatted_text_sensor_);
  LOG_TEXT_SENSOR("", "Battery Status", this->battery_status_text_sensor_);
  LOG_TEXT_SENSOR("", "Total Discharging Time Formatted", this->total_discharging_time_formatted_text_sensor_);
  LOG_TEXT_SENSOR("", "Total Charging Time Formatted", this->total_charging_time_formatted_text_sensor_);
  LOG_TEXT_SENSOR("", "Device Model", this->device_model_text_sensor_);
  LOG_TEXT_SENSOR("", "Software Version", this->software_version_text_sensor_);
}

void AntBmsBle::publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor == nullptr)
    return;

  binary_sensor->publish_state(state);
}

void AntBmsBle::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;

  sensor->publish_state(value);
}

void AntBmsBle::publish_state_(switch_::Switch *obj, const bool &state) {
  if (obj == nullptr)
    return;

  obj->publish_state(state);
}

void AntBmsBle::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr)
    return;

  text_sensor->publish_state(state);
}

std::array<uint8_t, 10> AntBmsBle::build_frame(uint8_t function, uint16_t address, uint8_t value) {
  std::array<uint8_t, 10> frame{};
  frame[0] = 0x7e;
  frame[1] = 0xa1;
  frame[2] = function;
  frame[3] = address >> 0;
  frame[4] = address >> 8;
  frame[5] = value;
  auto crc = crc16(frame.data() + 1, 5);
  frame[6] = crc >> 0;
  frame[7] = crc >> 8;
  frame[8] = 0xaa;
  frame[9] = 0x55;
  return frame;
}

bool AntBmsBle::send_(uint8_t function, uint16_t address, uint8_t value, bool wait_for_response) {
  auto frame = this->build_frame(function, address, value);
  ESP_LOGI(TAG, "Send frame (function 0x%02X): %s", function,
           format_hex_pretty_to(this->log_buffer_, sizeof(this->log_buffer_), frame.data(), frame.size(), '.'));

  auto status = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                         this->characteristic_handle_, frame.size(), frame.data(),
                                         ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status) {
    ESP_LOGW(TAG, "esp_ble_gattc_write_char failed, status=%d", status);
    return false;
  }

  return true;
}

bool AntBmsBle::write_register(uint8_t address, uint8_t value) {
  return this->send_(ANT_COMMAND_CONTROL, address, value, true);
}

}  // namespace esphome::ant_bms_ble
