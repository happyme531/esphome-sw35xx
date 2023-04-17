#pragma once

#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include <Wire.h>

#include <cstdint>
#include <h1_SW35xx.h>
#include <string>
using namespace h1_SW35xx;
using namespace esphome::sensor;

// singleton wrapper for sensor
class SW35xxWrapper {
private:
  SW35xxWrapper() {}
  SW35xxWrapper(const SW35xxWrapper &) = delete;
  SW35xxWrapper &operator=(const SW35xxWrapper &) = delete;
  SW35xx *sw35xx = nullptr;

public:
  static SW35xxWrapper &getInstance() {
    static SW35xxWrapper instance;
    return instance;
  }
  SW35xx &getSW35xx() { return *sw35xx; }
  void initialize(TwoWire &wire = Wire) {
    if (sw35xx == nullptr) {
      sw35xx = new SW35xx(wire);
      sw35xx->begin();
      // Set the max current if you like.
      // Please note that the max current is limited by the board. If you set a
      // higher value, the chip will likely overheat.
      sw35xx->setMaxCurrentsFixed(5000, 5000, 5000, 5000, 5000);
      sw35xx->setMaxCurrentsPPS(5000, 5000);
    }
  }
  bool isInitialized() { return sw35xx != nullptr; }
};

class ESPHomeSW35xxNumberSensors : public esphome::PollingComponent {
private:
  // TwoWire wire;
  const uint8_t sdaPin;
  const uint8_t sclPin;
  bool maxCurrentsFixedSet = false;
  uint16_t mA_5V, mA_9V, mA_12V, mA_15V, mA_20V;
  bool maxCurrentsPPSSet = false;
  uint16_t mA_pps1, mA_pps2;


public:
  Sensor *inputVoltageSensor = new Sensor();
  Sensor *outputVoltageSensor = new Sensor();
  Sensor *outputCurrent1Sensor = new Sensor();
  Sensor *outputCurrent2Sensor = new Sensor();

  ESPHomeSW35xxNumberSensors(const uint8_t sda, const uint8_t scl)
      : esphome::PollingComponent(10000), sdaPin(sda), sclPin(scl) {}

  float get_setup_priority() const override {
    return esphome::setup_priority::DATA;
  }

  void setup() override {
    Wire.begin(sdaPin, sclPin);
    SW35xxWrapper::getInstance().initialize(Wire);
    // Set the max current if you like.
    if (maxCurrentsFixedSet) {
      setMaxCurrentsFixed(mA_5V, mA_9V, mA_12V, mA_15V, mA_20V);
    }
    if (maxCurrentsPPSSet) {
      setMaxCurrentsPPS(mA_pps1, mA_pps2);
    }
  }

  void setMaxCurrentsFixed(uint16_t mA_5V, uint16_t mA_9V, uint16_t mA_12V,
                           uint16_t mA_15V, uint16_t mA_20V) {
    if (!SW35xxWrapper::getInstance().isInitialized()) {
      maxCurrentsFixedSet = true;
      this->mA_5V = mA_5V;
      this->mA_9V = mA_9V;
      this->mA_12V = mA_12V;
      this->mA_15V = mA_15V;
      this->mA_20V = mA_20V;
      return;
    }
    auto &sw35xx = SW35xxWrapper::getInstance().getSW35xx();
    sw35xx.setMaxCurrentsFixed(mA_5V, mA_9V, mA_12V, mA_15V, mA_20V);
  }
  void setMaxCurrentsPPS(uint16_t mA_pps1, uint16_t mA_pps2) {
    if (!SW35xxWrapper::getInstance().isInitialized()) {
      maxCurrentsPPSSet = true;
      this->mA_pps1 = mA_pps1;
      this->mA_pps2 = mA_pps2;
      return;
    }
    auto &sw35xx = SW35xxWrapper::getInstance().getSW35xx();
    sw35xx.setMaxCurrentsPPS(mA_pps1, mA_pps2);
  }

  void update() override {
    auto &sw35xx = SW35xxWrapper::getInstance().getSW35xx();
    sw35xx.readStatus();
    inputVoltageSensor->publish_state(sw35xx.vin_mV / 1000.0);
    outputVoltageSensor->publish_state(sw35xx.vout_mV / 1000.0);
    outputCurrent1Sensor->publish_state(sw35xx.iout_usba_mA / 1000.0);
    outputCurrent2Sensor->publish_state(sw35xx.iout_usbc_mA / 1000.0);
  }
};

class ESPHomeSW35xxTextSensor : public esphome::PollingComponent,
                                   public TextSensor {
public:
  ESPHomeSW35xxTextSensor() : esphome::PollingComponent(10000) {}

  void setup() override {
    // Got initialized in ESPHomeSW35xxNumberSensors
    //  Wire.begin();
    //  SW35xxWrapper::getInstance().initialize(Wire);
  }

  void update() override {
    if (!SW35xxWrapper::getInstance().isInitialized()) {
      return;
    }
    auto &sw35xx = SW35xxWrapper::getInstance().getSW35xx();
    // sw35xx.readStatus(); //already done in ESPHomeSW35xxNumberSensors
    publish_state(
        fastChargeType2String(sw35xx.fastChargeType, sw35xx.PDVersion));
  }

  std::string fastChargeType2String(SW35xx::fastChargeType_t type,
                                    uint8_t PDVersion) {
    switch (type) {
    case SW35xx::NOT_FAST_CHARGE:
      return "None";
      break;
    case SW35xx::QC2:
      return "QC2.0";
      break;
    case SW35xx::QC3:
      return "QC3.0";
      break;
    case SW35xx::FCP:
      return "FCP";
      break;
    case SW35xx::SCP:
      return "SCP";
      break;
    case SW35xx::PD_FIX:
      switch (PDVersion) {
      case 2:
        return "PD2.0 Fixed";
        break;
      case 3:
        return "PD3.0 Fixed";
        break;
      default:
        return "Unknown PD Fixed";
        break;
      }
      break;
    case SW35xx::PD_PPS:
      switch (PDVersion) {
      case 3:
        return "PD3.0 PPS";
        break;
      default:
        return "Unknown PD PPS";
        break;
      }
      break;
    case SW35xx::MTKPE1:
      return "MTK PE1.1";
      break;
    case SW35xx::MTKPE2:
      return "MTK PE2.0";
      break;
    case SW35xx::LVDC:
      return "LVDC";
      break;
    case SW35xx::SFCP:
      return "SFCP";
      break;
    case SW35xx::AFC:
      return "AFC";
      break;
    default:
      return "Unknown";
      break;
    }
  }
};