#include "esphome.h"

#define ROOMBA_READ_TIMEOUT 200

class RoombaComponent : public UARTDevice, public CustomAPIDevice, public PollingComponent {
	public:
		//Sensor *distanceSensor;
		Sensor *voltageSensor;
		Sensor *currentSensor;
		Sensor *batteryChargeSensor;
		Sensor *batteryCapacitySensor;
		Sensor *batteryPercentSensor;
		Sensor *batteryTemperatureSensor;
		TextSensor *chargingSensor;
		TextSensor *activitySensor;
		Sensor *driveSpeedSensor;
		TextSensor *oiModeSensor;

		static RoombaComponent* instance(uint8_t brcPin, UARTComponent *parent, uint32_t updateInterval, bool lazy650Enabled) {
			static RoombaComponent* INSTANCE = new RoombaComponent(brcPin, parent, updateInterval, lazy650Enabled);
			return INSTANCE;
		}

		void setup() override {
			if (this->lazy650Enabled) {
				// High-impedence on the BRC_PIN
				// see https://github.com/johnboiles/esp-roomba-mqtt/commit/fa9af14376f740f366a9ecf4cb59dec2419deeb0#diff-34d21af3c614ea3cee120df276c9c4ae95053830d7f1d3deaf009a4625409ad2R140
				pinMode(this->brcPin, INPUT);
			} else {
				pinMode(this->brcPin, OUTPUT);
				digitalWrite(this->brcPin, HIGH);
			}

			register_service(&RoombaComponent::on_command, "command", {"command"});
		}

    	void update() override {
			if (this->lazy650Enabled) {
				long now = millis();
				// Wakeup the roomba at fixed intervals
				if (now - lastWakeupTime > 50000) {
					ESP_LOGD("custom", "Time to wakeup");
					lastWakeupTime = now;
					if (!wasCleaning) {
						if (wasDocked) {
							wake_on_dock();
						} else {
							brc_wakeup();
						}
					} else {
						brc_wakeup();
					}
				}
			}

			uint8_t charging;
			uint16_t voltage;
			int16_t current;
			uint16_t batteryCharge;
			uint16_t batteryCapacity;
			int16_t batteryTemperature;

			flush();

			uint8_t sensors[] = {
				SensorChargingState,
				SensorVoltage,
				SensorCurrent,
				SensorBatteryCharge,
				SensorBatteryCapacity,
				SensorBatteryTemperature,
				SensorOIMode
			};

			uint8_t values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

			bool success = getSensorsList(sensors, sizeof(sensors), values, sizeof(values));
      		if (!success) {
				ESP_LOGD("custom", "Could not get sensor values from serial");
				return;
			}

			charging = values[0];
			voltage = values[1] * 256 + values[2];
			current = values[3] * 256 + values[4];
			batteryCharge = values[5] * 256 + values[6];
      		batteryCapacity = values[7] * 256 + values[8];
			batteryTemperature = values[9];
			std::string oiMode = get_oimode(values[10]);

			std::string activity = get_activity(charging, current);
			wasCleaning = activity == "Cleaning";
			wasDocked = activity == "Docked";

			float voltageInVolts = (1.0 * voltage) / 1000.0;
			if (this->voltageSensor->state != voltageInVolts) {
				this->voltageSensor->publish_state(voltageInVolts);
			}

			float currentInAmps = (1.0 * current) / 1000.0;
			if (this->currentSensor->state != currentInAmps) {
				this->currentSensor->publish_state(currentInAmps);
			}

			float chargeInAmpHours = (1.0 * batteryCharge) / 1000.0;
			if (this->batteryChargeSensor->state != chargeInAmpHours) {
				this->batteryChargeSensor->publish_state(chargeInAmpHours);
			}

			float capacityInAmpHours = (1.0 * batteryCapacity) / 1000.0;
			if (this->batteryCapacitySensor->state != capacityInAmpHours) {
				this->batteryCapacitySensor->publish_state(capacityInAmpHours);
			}

			float battery_level = 100.0 * ((1.0 * batteryCharge) / (1.0 * batteryCapacity));
			if (this->batteryPercentSensor->state != battery_level) {
				this->batteryPercentSensor->publish_state(battery_level);
			}

			if (this->batteryTemperatureSensor->state != batteryTemperature) {
				this->batteryTemperatureSensor->publish_state(batteryTemperature);
			}

			if (this->chargingState != charging) {
				this->chargingState = charging;
				this->chargingSensor->publish_state(ToString(charging));
			}

			if (activity.compare(this->activitySensor->state) != 0) {
				this->activitySensor->publish_state(activity);
			}

			if (this->driveSpeedSensor->state != this->speed) {
				this->driveSpeedSensor->publish_state(this->speed);
			}

			if (oiMode.compare(this->oiModeSensor->state) != 0) {
				this->oiModeSensor->publish_state(oiMode);
			}
		}

	private:
		uint8_t brcPin;
		uint8_t chargingState;
		int lastWakeupTime = 0;
		bool wasCleaning = false;
		bool wasDocked = false;
		int16_t speed = 0;

		bool lazy650Enabled = false;

		RoombaComponent(uint8_t brcPin, UARTComponent *parent, uint32_t updateInterval, bool lazy650Enabled) : UARTDevice(parent), PollingComponent(updateInterval) {
			this->brcPin = brcPin;
			this->lazy650Enabled = lazy650Enabled;
			this->voltageSensor = new Sensor();
			this->currentSensor = new Sensor();
			this->batteryChargeSensor = new Sensor();
	        this->batteryCapacitySensor = new Sensor();
	        this->batteryPercentSensor = new Sensor();
			this->batteryTemperatureSensor = new Sensor();

			this->chargingSensor = new TextSensor();
			this->activitySensor = new TextSensor();

			this->driveSpeedSensor = new Sensor();
			this->oiModeSensor = new TextSensor();
		}

		typedef enum {
			Sensors7to26					= 0,  //00
			Sensors7to16					= 1,  //01
			Sensors17to20					= 2,  //02
			Sensors21to26					= 3,  //03
			Sensors27to34					= 4,  //04
			Sensors35to42					= 5,  //05
			Sensors7to42					= 6,  //06
			SensorBumpsAndWheelDrops		= 7,  //07
			SensorWall						= 8,  //08
			SensorCliffLeft					= 9,  //09
			SensorCliffFrontLeft			= 10, //0A
			SensorCliffFrontRight			= 11, //0B
			SensorCliffRight				= 12, //0C
			SensorVirtualWall				= 13, //0D
			SensorOvercurrents				= 14, //0E
			//SensorUnused1					= 15, //0F
			//SensorUnused2					= 16, //10
			SensorIRByte					= 17, //11
			SensorButtons					= 18, //12
			SensorDistance					= 19, //13
			SensorAngle						= 20, //14
			SensorChargingState				= 21, //15
			SensorVoltage					= 22, //16
			SensorCurrent					= 23, //17
			SensorBatteryTemperature		= 24, //18
			SensorBatteryCharge				= 25, //19
			SensorBatteryCapacity			= 26, //1A
			SensorWallSignal				= 27, //1B
			SensoCliffLeftSignal			= 28, //1C
			SensoCliffFrontLeftSignal		= 29, //1D
			SensoCliffFrontRightSignal		= 30, //1E
			SensoCliffRightSignal			= 31, //1F
			SensorUserDigitalInputs			= 32, //20
			SensorUserAnalogInput			= 33, //21
			SensorChargingSourcesAvailable	= 34, //22
			SensorOIMode					= 35, //23
			SensorSongNumber				= 36, //24
			SensorSongPlaying				= 37, //25
			SensorNumberOfStreamPackets		= 38, //26
			SensorVelocity					= 39, //27
			SensorRadius					= 40, //28
			SensorRightVelocity				= 41, //29
			SensorLeftVelocity				= 42, //2A
		} SensorCode;

		typedef enum {
			ChargeStateNotCharging				= 0,
			ChargeStateReconditioningCharging	= 1,
			ChargeStateFullCharging				= 2,
			ChargeStateTrickleCharging			= 3,
			ChargeStateWaiting					= 4,
			ChargeStateFault					= 5,
		} ChargeState;

		void brc_wakeup() {
			if (this->lazy650Enabled) {
				ESP_LOGD("custom", "brc_wakeup");
				pinMode(this->brcPin, OUTPUT);
				digitalWrite(this->brcPin, LOW);
				delay(200);
				pinMode(this->brcPin, OUTPUT);
				delay(200);
				start_oi(); // Start
			} else {
				digitalWrite(this->brcPin, LOW);
				delay(1000);
				digitalWrite(this->brcPin, HIGH);
				delay(100);
			}
		}

		void on_command(std::string command) {
			if (command == "turn_on" || command == "turn_off" || command == "start" || command == "stop") {
				cover();
			}
			else if (command == "dock" || command == "return_to_base") {
				dock();
			}
			else if (command == "locate") {
				locate();
			}
			else if (command == "spot" || command == "clean_spot") {
				spot();
			}
			else if (command == "wakeup") {
				brc_wakeup();
			}
			else if (command == "wake_on_dock") {
				wake_on_dock();
			}
			else if (command == "sleep") {
				sleep();
			}
			else if (command == "go_max") {
	ESP_LOGI("roomba", "go max");
				alter_speed(1000);
			} else if (command == "go_faster") {
	ESP_LOGI("roomba", "go faster");
				alter_speed(100);
			} else if (command == "go_slower") {
	ESP_LOGI("roomba", "slow it down");
				alter_speed(-100);
			} else if (command == "go_right") {
	ESP_LOGI("roomba", "easy right");
				drive(speed, -250);
			} else if (command == "go_left") {
	ESP_LOGI("roomba", "easy left");
				drive(this->speed, 250);
			} else if (command == "halt") {
	ESP_LOGI("roomba", "HALT");
				this->speed = 0;
				drive(0,0);
			} else if (command == "drive") {
	ESP_LOGI("roomba", "DRIVE mode");
				this->speed = 0;
				safeMode();
			}
ESP_LOGI("roomba", "ACK %s", command.c_str());
    	}

		void start_oi() {
			write(128);
		}

		void locate() {
			write(141);
			write(0);
		}

		void wake_on_dock() {
			ESP_LOGD("custom", "wake_on_dock");
			brc_wakeup();
			// Some black magic from @AndiTheBest to keep the Roomba awake on the dock
			// See https://github.com/johnboiles/esp-roomba-mqtt/issues/3#issuecomment-402096638
			delay(10);
			write(135); // Clean
			delay(150);
			write(143); // Dock
		}

		void alter_speed(int16_t step) {
			int16_t speed = this->speed + step;

			if (speed > 500)
				speed = 500;
			else if (speed < -500)
				speed = -500;

			this->speed = speed;
			this->driveSpeedSensor->publish_state(speed);
			this->drive(this->speed, 0);
		}

		void drive(int16_t velocity, int16_t radius) {
			write(137);
			write((velocity & 0xff00) >> 8);
			write(velocity & 0xff);
			write((radius & 0xff00) >> 8);
			write(radius & 0xff);
		}

		void safeMode() {
			write(131);
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

		void cover() {
			write(135);
		}

		void dock() {
			write(143);
		}

		void spot() {
			write(134);
		}

		void sleep() {
			write(133);
		}

		void flush() {
			while (available())
			{
				read();
			}
		}

		bool getSensorsList(uint8_t* packetIDs, uint8_t numPacketIDs, uint8_t* dest, uint8_t len){
			write(149);
			write(numPacketIDs);
			write_array(packetIDs, numPacketIDs);
			return getData(dest, len);
		}

		bool getData(uint8_t* dest, uint8_t len) {
			while (len-- > 0) {
				unsigned long startTime = millis();
				while (!available()) {
					yield();
					// Look for a timeout
					if (millis() > startTime + ROOMBA_READ_TIMEOUT)
					return false;
				}
				*dest++ = read();
			}
			return true;
		}

		std::string get_activity(uint8_t charging, int16_t current) {
			bool isCharging = charging == ChargeStateReconditioningCharging || charging == ChargeStateFullCharging || charging == ChargeStateTrickleCharging;
			
			if (current > -50)
				return "Docked";
			else if (isCharging)
				return "Charging";
			else if (current < -300)
				return "Cleaning";
			return "Lost";
		}

		inline const char* ToString(uint8_t chargeState) {
			switch (chargeState) {
				case ChargeStateNotCharging:			return "NotCharging";
				case ChargeStateReconditioningCharging:	return "ReconditioningCharging";
				case ChargeStateFullCharging:			return "FullCharging";
				case ChargeStateTrickleCharging:		return "TrickleCharging";
				case ChargeStateWaiting:				return "Waiting";
				case ChargeStateFault:					return "Fault";
				default:								return "Unknow Charging State";
			}
		}
};
