# ESPHome ANT BMS (BLE & UART)

ESPHome component to monitor and control **ANT BMS** (Bluetooth Low Energy & UART) with support for modern firmware versions (`24AACB...` and legacy).

## Key Improvements & Fixes in this Fork
* **CCCD Notification Descriptor Fix:** Explicitly enables notifications on characteristic `0xFFE1` by writing `0x0001` to CCCD `0x2902`, ensuring newer ANT BMS Bluetooth modules stream telemetry data reliably.
* **Direct Status Snapshot Request:** Requests the full status snapshot (`0x01`) upon connection rather than stalling on unsupported `DeviceInfo` registers.
* **Persistent Telemetry States:** Preserves last known valid sensor measurements (voltage, current, cell voltages, SoC) in Home Assistant during brief BLE reconnect cycles instead of resetting to `NAN` (Unavailable).

## Usage in ESPHome

```yaml
external_components:
  - source: github://neuleo/esphome-ant-bms@main
    refresh: 0s

esp32_ble_tracker:
  scan_parameters:
    interval: 320ms
    window: 50ms
    active: false

ble_client:
  - id: ant_client
    mac_address: "18:7C:1A:1B:5F:E6"
    auto_connect: true

ant_bms_ble:
  - id: bms0
    ble_client_id: ant_client
    update_interval: 5s

sensor:
  - platform: ant_bms_ble
    ant_bms_ble_id: bms0
    total_voltage:
      name: "Bike Battery Voltage"
    current:
      name: "Bike Battery Current"
    power:
      name: "Bike Battery Power"
    soc:
      name: "Bike Battery SOC"
    capacity_remaining:
      name: "Bike Battery Remaining Capacity"
    battery_strings:
      name: "Bike Battery Cell Count"
    average_cell_voltage:
      name: "Bike Avg Cell Voltage"
    min_cell_voltage:
      name: "Bike Min Cell Voltage"
    max_cell_voltage:
      name: "Bike Max Cell Voltage"
    delta_cell_voltage:
      name: "Bike Delta Cell Voltage"
    temperature_1:
      name: "Bike Temp 1"
    temperature_2:
      name: "Bike Temp 2"
    temperature_5:
      name: "Bike Mosfet Temp"
    temperature_6:
      name: "Bike Balancer Temp"
    cell_voltage_1:
      name: "Cell 1 Voltage"
    cell_voltage_2:
      name: "Cell 2 Voltage"
    cell_voltage_3:
      name: "Cell 3 Voltage"
    cell_voltage_4:
      name: "Cell 4 Voltage"
    cell_voltage_5:
      name: "Cell 5 Voltage"
    cell_voltage_6:
      name: "Cell 6 Voltage"
    cell_voltage_7:
      name: "Cell 7 Voltage"
    cell_voltage_8:
      name: "Cell 8 Voltage"
    cell_voltage_9:
      name: "Cell 9 Voltage"
    cell_voltage_10:
      name: "Cell 10 Voltage"
    cell_voltage_11:
      name: "Cell 11 Voltage"
    cell_voltage_12:
      name: "Cell 12 Voltage"
    cell_voltage_13:
      name: "Cell 13 Voltage"
    cell_voltage_14:
      name: "Cell 14 Voltage"
    cell_voltage_15:
      name: "Cell 15 Voltage"
    cell_voltage_16:
      name: "Cell 16 Voltage"
    cell_voltage_17:
      name: "Cell 17 Voltage"
    cell_voltage_18:
      name: "Cell 18 Voltage"
    cell_voltage_19:
      name: "Cell 19 Voltage"
    cell_voltage_20:
      name: "Cell 20 Voltage"

binary_sensor:
  - platform: ant_bms_ble
    ant_bms_ble_id: bms0
    online_status:
      name: "Bike BMS Online"

text_sensor:
  - platform: ant_bms_ble
    ant_bms_ble_id: bms0
    battery_status:
      name: "Bike Battery Status"
    charge_mosfet_status:
      name: "Bike Charge Mosfet"
    discharge_mosfet_status:
      name: "Bike Discharge Mosfet"
    balancer_status:
      name: "Bike Balancer Status"
```
