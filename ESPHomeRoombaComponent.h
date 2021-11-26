#include "esphome.h"
#include <Roomba.h>

class RoombaComponent : public PollingComponent, public CustomAPIDevice {
  protected:
    uint8_t brcPin;
    uint32_t updateInterval;
    uint8_t chargingState;
    Roomba roomba;

    void brc_wakeup() {
        digitalWrite(this->brcPin, LOW);
        delay(500);
        digitalWrite(this->brcPin, HIGH);
        delay(100);
    }

    void on_command(std::string command) {
        if (command == "turn_on" || command == "turn_off" || command == "start" || command == "stop")
            this->roomba.cover();
        else if (command == "dock" || command == "return_to_base")
            this->roomba.dock();
        else if (command == "locate")
            this->roomba.playSong(1);
        else if (command == "spot" || command == "clean_spot")
            this->roomba.spot();
    }

    inline const char* ToString(uint8_t chargeState)
    {
        switch (chargeState)
        {
            case Roomba::ChargeStateNotCharging:              return "NotCharging";
            case Roomba::ChargeStateReconditioningCharging:   return "ReconditioningCharging";
            case Roomba::ChargeStateFullChanrging:            return "FullChanrging";
            case Roomba::ChargeStateTrickleCharging:          return "TrickleCharging";
            case Roomba::ChargeStateWaiting:                  return "Waiting";
            case Roomba::ChargeStateFault:                    return "Fault";
            default:      return "Unknow Charging State";
        }
    }

    std::string get_activity(uint8_t charging, int16_t current) {
      bool isCharging = charging == Roomba::ChargeStateReconditioningCharging
        || charging == Roomba::ChargeStateFullChanrging
        || charging == Roomba::ChargeStateTrickleCharging;

      if (current > -50)
        return "Docked";
      else if (isCharging)
        return "Charging";
      else if (current < -300)
        return "Cleaning";
      return "Lost";
    }

  public:
    Sensor *distanceSensor;
    Sensor *voltageSensor;
    Sensor *currentSensor;
    Sensor *chargeSensor;
    Sensor *capacitySensor;
    Sensor *batteryPercentSensor;
    TextSensor *chargingSensor;
    TextSensor *activitySensor;

    static RoombaComponent* instance(uint8_t brcPin, uint32_t updateInterval)
    {
        static RoombaComponent* INSTANCE = new RoombaComponent(brcPin, updateInterval);
        return INSTANCE;
    }

    void setup() override
    {
        pinMode(this->brcPin, OUTPUT);
        digitalWrite(this->brcPin, HIGH);

        this->roomba.start();
        register_service(&RoombaComponent::on_command, "command", {"command"});
    }

    void update() override
    {
      int16_t distance;
      uint16_t voltage;
      int16_t current;
      uint16_t charge;
      uint16_t capacity;
      uint8_t charging;

      // Flush serial buffers
      while (Serial.available())
      {
          Serial.read();
      }
      // https://github.com/Ceiku/Roomba/blob/251e677eb506d7bf27fff7e1bfb72798ef7b7340/Roomba.h#L336-L383
      uint8_t sensors[] = {
          Roomba::SensorDistance,       // 2 bytes, mm, signed
          Roomba::SensorChargingState,  // 1 byte
          Roomba::SensorVoltage,        // 2 bytes, mV, unsigned
          Roomba::SensorCurrent,        // 2 bytes, mA, signed
          Roomba::SensorBatteryCharge,  // 2 bytes, mAh, unsigned
          Roomba::SensorBatteryCapacity // 2 bytes, mAh, unsigned
      };
      uint8_t values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

      // Serial reading timeout -- https://community.home-assistant.io/t/add-wifi-to-an-older-roomba/23282/52
      bool success = this->roomba.getSensorsList(sensors, sizeof(sensors), values, sizeof(values));
      if (!success)
          return;

      distance = values[0] * 256 + values[1];
      voltage = values[3] * 256 + values[4];
      current = values[5] * 256 + values[6];
      charge = values[7] * 256 + values[8];
      capacity = values[9] * 256 + values[10];
      charging = values[2];

      float battery_level = 100.0 * ((1.0 * charge) / (1.0 * capacity));
      std::string activity = this->get_activity(charging, current);

      // Only publish new states if there was a change
      if (this->distanceSensor->state != distance)
          this->distanceSensor->publish_state(distance);

      if (this->voltageSensor->state != voltage)
          this->voltageSensor->publish_state(voltage);

      if (this->currentSensor->state != current)
          this->currentSensor->publish_state(current);

      if (this->chargeSensor->state != charge)
          this->chargeSensor->publish_state(charge);

      if (this->capacitySensor->state != capacity)
          this->capacitySensor->publish_state(capacity);

      if (this->batteryPercentSensor->state != battery_level)
        this->batteryPercentSensor->publish_state(battery_level);

      if (this->chargingState != charging) {
        this->chargingState = charging;
        this->chargingSensor->publish_state(ToString(charging));
      }

      if (activity.compare(this->activitySensor->state) != 0)
        this->activitySensor->publish_state(activity);
    }

  private:
    RoombaComponent(uint8_t brcPin, uint32_t updateInterval) :
        PollingComponent(updateInterval), roomba(&Serial, Roomba::Baud115200)
    {
        this->brcPin = brcPin;
        this->updateInterval = updateInterval;

        this->distanceSensor = new Sensor();
        this->voltageSensor = new Sensor();
        this->currentSensor = new Sensor();
        this->chargeSensor = new Sensor();
        this->capacitySensor = new Sensor();
        this->batteryPercentSensor = new Sensor();
        this->chargingSensor = new TextSensor();
        this->activitySensor = new TextSensor();
    }
};
