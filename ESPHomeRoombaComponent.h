#include "esphome.h"
#include <Roomba.h>
#include <SoftwareSerial.h>

#define clamp(value, min, max) (value < min ? min : value > max ? max : value)

class RoombaComponent : public PollingComponent, public CustomAPIDevice {
  protected:
    uint8_t brcPin;
    uint32_t updateInterval;
    Roomba::Baud baud;
    Roomba roomba;
    SoftwareSerial *serial;
    bool IR = false;
    int16_t speed = 0;

    void brc_wakeup() {
        // Roomba Wakeup
        digitalWrite(this->brcPin, LOW);
        delay(500);
        digitalWrite(this->brcPin, HIGH);
        delay(100);
    }

    void on_command(std::string command) {
        this->brc_wakeup();

        if (command == "turn_on" || command == "turn_off" || command == "start" || command == "stop")
            this->roomba.cover();
        else if (command == "dock" || command == "return_to_base")
            this->roomba.dock();
        else if (command == "locate")
            this->roomba.playSong(1);
        else if (command == "spot" || command == "clean_spot")
            this->roomba.spot();
        else if (command == "wakeup" || command == "brc_wakeup")
            this->brc_wakeup();
        else if (command == "go_max") {
ESP_LOGI("roomba", "go max");
            this->alter_speed(1000);
        } else if (command == "go_faster") {
ESP_LOGI("roomba", "go faster");
            this->alter_speed(100);
        } else if (command == "go_slower") {
ESP_LOGI("roomba", "slow it down");
            this->alter_speed(-100);
        } else if (command == "go_right") {
ESP_LOGI("roomba", "easy right");
            this->roomba.drive(this->speed, -250);
        } else if (command == "go_left") {
ESP_LOGI("roomba", "easy left");
            this->roomba.drive(this->speed, 250);
        } else if (command == "halt") {
ESP_LOGI("roomba", "HALT");
            this->speed = 0;
            this->roomba.drive(0,0);
        } else if (command == "drive") {
ESP_LOGI("roomba", "DRIVE mode");
            this->speed = 0;
            this->roomba.safeMode();
        }

ESP_LOGI("roomba", "ACK %s", command.c_str());
    }

    void alter_speed(int16_t step) {
      int16_t speed = this->speed + step;

//      clamp(speed, -500, 500);
      if (speed > 500)
        speed = 500;
      else if (speed < -500)
        speed = -500;

      this->speed = speed;
      this->roomba.drive(this->speed, 0);
    }

    std::string get_activity(uint8_t charging, int16_t current) {
      bool chargingState = charging == Roomba::ChargeStateReconditioningCharging
        || charging == Roomba::ChargeStateFullChanrging
        || charging == Roomba::ChargeStateTrickleCharging;

      if (current > -50)
        return "Docked";
      else if (chargingState)
        return "Charging";
      else if (current < -300)
        return "Cleaning";
      return "Lost";
    }

    std::string get_oimode(uint8_t mode) {
      switch(mode) {
        case 0: return "off";
        case 1: return "passive";
        case 2: return "safe";
        case 3: return "full";
        default: return "unknown";
      }
    }

  public:
    Sensor *distanceSensor;
    Sensor *voltageSensor;
    Sensor *currentSensor;
    Sensor *chargeSensor;
    Sensor *capacitySensor;
    Sensor *batteryPercentSensor;
    TextSensor *activitySensor;

    Sensor *driveSpeedSensor;
    TextSensor *oiModeSensor;

    static RoombaComponent* instance(uint8_t brcPin, uint8_t rxPin, uint8_t txPin, Roomba::Baud baud, uint32_t updateInterval)
    {
        static RoombaComponent* INSTANCE = new RoombaComponent(
                brcPin,
                new SoftwareSerial(rxPin, txPin),
                baud,
                updateInterval
            );
        return INSTANCE;
    }

    void setup() override
    {
        pinMode(this->brcPin, OUTPUT);
        digitalWrite(this->brcPin, HIGH);

//        this->brc_init();

ESP_LOGI("roomba", "Starting OI");
        this->roomba.start();
        register_service(&RoombaComponent::on_command, "command", {"command"});
ESP_LOGI("roomba", "Switching to custom baud rate");
        this->roomba.baud(this->baud);
    }

    void update() override
    {
      int16_t distance;
      uint16_t voltage;
      int16_t current;
      uint16_t charge;
      uint16_t capacity;
      uint8_t charging;
//ESP_LOGI("roomba", "flushing serial buffers");
      // Flush serial buffers
      while (this->serial->available())
      {
          this->serial->read();
      }
      // https://github.com/Ceiku/Roomba/blob/251e677eb506d7bf27fff7e1bfb72798ef7b7340/Roomba.h#L336-L383
      uint8_t sensors[] = {
          Roomba::SensorDistance,       // 2 bytes, mm, signed
          Roomba::SensorChargingState,  // 1 byte
          Roomba::SensorVoltage,        // 2 bytes, mV, unsigned
          Roomba::SensorCurrent,        // 2 bytes, mA, signed
          Roomba::SensorBatteryCharge,  // 2 bytes, mAh, unsigned
          Roomba::SensorBatteryCapacity, // 2 bytes, mAh, unsigned
          Roomba::SensorOIMode          // 1 byte
      };
      uint8_t values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

ESP_LOGI("roomba", "reading data from serial");
      // Serial reading timeout -- https://community.home-assistant.io/t/add-wifi-to-an-older-roomba/23282/52
      bool success = this->roomba.getSensorsList(sensors, sizeof(sensors), values, sizeof(values));
ESP_LOGI("roomba", "getSensorsList result is %d", success);
      if (!success)
          return;

      distance = values[0] * 256 + values[1];
      voltage = values[3] * 256 + values[4];
      current = values[5] * 256 + values[6];
      charge = values[7] * 256 + values[8];
      capacity = values[9] * 256 + values[10];
      charging = values[2];

ESP_LOGI("roomba", "oimode is %d", values[11]);
      std::string oiMode = this->get_oimode(values[11]);

      float battery_level = 100.0 * ((1.0 * charge) / (1.0 * capacity));
      std::string activity = this->get_activity(charging, current);

ESP_LOGI("roomba", "publishing state changes");

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

      if (activity.compare(this->activitySensor->state) == 0)
        this->activitySensor->publish_state(activity);

      if (this->driveSpeedSensor->state != this->speed)
        this->driveSpeedSensor->publish_state(this->speed);

      if (oiMode.compare(this->oiModeSensor->state) == 0)
        this->oiModeSensor->publish_state(oiMode);
    }

  private:
    RoombaComponent(uint8_t brcPin, SoftwareSerial *serial, Roomba::Baud baud, uint32_t updateInterval) :
        PollingComponent(updateInterval), roomba(serial, Roomba::Baud115200)
    {
        this->serial = serial;
        this->brcPin = brcPin;
        this->baud = baud;
        this->updateInterval = updateInterval;

        this->distanceSensor = new Sensor();
        this->voltageSensor = new Sensor();
        this->currentSensor = new Sensor();
        this->chargeSensor = new Sensor();
        this->capacitySensor = new Sensor();
        this->batteryPercentSensor = new Sensor();
        this->activitySensor = new TextSensor();

        this->driveSpeedSensor = new Sensor();
        this->oiModeSensor = new TextSensor();
    }
};
