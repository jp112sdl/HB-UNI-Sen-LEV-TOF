//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2018-11-14 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>
#include <sensors/Vl53l0x.h>

// Arduino Pro mini 8 Mhz
// Arduino pin for the config button
#define CONFIG_BUTTON_PIN  8
#define LED_PIN            4

#define BATT_EN_PIN        15
#define BATT_SENS_PIN      17

// number of available peers per channel
#define PEERS_PER_CHANNEL 2

// all library classes are placed in the namespace 'as'
using namespace as;

//Korrekturfaktor der Clock-Ungenauigkeit, wenn keine RTC verwendet wird
#define SYSCLOCK_FACTOR    0.88
#define MAX_MEASURE_COUNT  5

enum ToFSensorTypes {
  sT_VL53L0X
};

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xF9, 0xD3, 0x00},          // Device ID
  "JPLEV00001",                // Device Serial
  {0xF9, 0xD3},                // Device Model
  0x10,                        // Firmware Version
  0x53,                        // Device Type
  {0x01, 0x01}                 // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AskSin<StatusLed<LED_PIN>, BatterySensorUni<BATT_SENS_PIN, BATT_EN_PIN, 0>, Radio<AvrSPI<10, 11, 12, 13>, 2>> BaseHal;
class Hal : public BaseHal {
  public:
    void init (const HMID& id) {
      BaseHal::init(id);
      battery.init(seconds2ticks(60UL * 60) * SYSCLOCK_FACTOR, sysclock); //battery measure once an hour
      battery.low(22);
      battery.critical(19);
    }

    bool runready () {
      return sysclock.runready() || BaseHal::runready();
    }
} hal;


DEFREGISTER(UReg0, MASTERID_REGS, DREG_LOWBATLIMIT, 0x20, 0x21)
class UList0 : public RegList0<UReg0> {
  public:
    UList0 (uint16_t addr) : RegList0<UReg0>(addr) {}

    bool Sendeintervall (uint16_t value) const {
      return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff);
    }
    uint16_t Sendeintervall () const {
      return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0);
    }

    void defaults () {
      clear();
      lowBatLimit(22);
      Sendeintervall(180);
    }
};

DEFREGISTER(UReg1, CREG_CASE_HIGH, CREG_CASE_WIDTH, CREG_CASE_DESIGN, CREG_CASE_LENGTH, 0x01, 0x02, 0x03)
class UList1 : public RegList1<UReg1> {
  public:
    UList1 (uint16_t addr) : RegList1<UReg1>(addr) {}

    bool distanceOffset (uint16_t value) const {
      return this->writeRegister(0x01, (value >> 8) & 0xff) && this->writeRegister(0x02, value & 0xff);
    }
    uint16_t distanceOffset () const {
      return (this->readRegister(0x01, 0) << 8) + this->readRegister(0x02, 0);
    }

    bool sensorType (uint16_t value) const {
      return this->writeRegister(0x03, value & 0xff);
    }
    uint16_t sensorType () const {
      return this->readRegister(0x03, 0);
    }

    void defaults () {
      clear();
      caseHigh(100);
      caseWidth(100);
      caseLength(100);
      caseDesign(0);
      distanceOffset(0);
      sensorType(0);
    }
};

class MeasureEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint8_t percent, uint32_t liter, uint8_t volt) {
      Message::init(0x0f, msgcnt, 0x53, (msgcnt % 20 == 1) ? BIDI : BCAST , percent & 0xff, volt & 0xff);
      pload[0] = (liter >>  24) & 0xff;
      pload[1] = (liter >>  16) & 0xff;
      pload[2] = (liter >>  8) & 0xff;
      pload[3] = liter & 0xff;
    }
};

class MeasureChannel : public Channel<Hal, UList1, EmptyList, List4, PEERS_PER_CHANNEL, UList0>, public Alarm {
    MeasureEventMsg msg;
    uint32_t fillingLiter;
    uint8_t  fillingPercent;
    uint16_t distance;
    uint8_t last_flags = 0xff;

    Vl53l0x<0x29, 500, RangeMode::LONG_RANGE>      vl53l0x;
    
  public:
    MeasureChannel () : Channel(), Alarm(0), fillingLiter(0), fillingPercent(0)  {}
    virtual ~MeasureChannel () {}

    void measure() {
      uint32_t caseHeight = this->getList1().caseHigh() * 10;
      uint32_t caseWidth = this->getList1().caseWidth() * 10;
      uint32_t caseLength = this->getList1().caseLength() * 10;
      uint32_t caseDesign  = this->getList1().caseDesign();
      uint32_t distanceOffset = this->getList1().distanceOffset() * 10;

      uint32_t m_value = 0;

      switch (this->getList1().sensorType()) {
        case sT_VL53L0X:
          if (vl53l0x.measure() == true) {
            m_value = vl53l0x.DistanceMM();
          }
          break;

        default:
          DPRINTLN(F("Invalid Sensor Type selected"));
          break;
      }

      DPRINTLN("");
      DPRINT(F("Abstand gemessen (mm)    : ")); DDECLN(m_value);
      distance = (distanceOffset > m_value) ? 0 :  m_value - distanceOffset;
      DPRINT(F("Abstand abzgl. OFFSET(mm): ")); DDECLN(distance);

      DPRINT(F("Behaelterhoehe (mm)      : ")); DDECLN(caseHeight);
      uint32_t fillingHeight = (distance > caseHeight) ? 0 : caseHeight - distance;
      DPRINT(F("Fuellhoehe (mm)          : ")); DDECLN(fillingHeight);

      uint32_t caseVolume; float r;
      #define DIVIDER 1000000L
      switch (caseDesign) {
        case 0:
          caseVolume = (PI * pow((caseWidth >> 1), 2) * caseHeight) / DIVIDER;
          fillingLiter = (PI * pow((caseWidth >> 1), 2) * fillingHeight) / DIVIDER;
          break;
        case 1:
          caseVolume = (PI * pow((caseHeight >> 1), 2) * caseWidth) / DIVIDER;
          //fillingLiter = pow(caseHeight >> 1, 2) * caseWidth * (acos((caseHeight >> 1 - fillingHeight) / caseHeight >> 1) - (caseHeight >> 1 - fillingHeight) * (sqrt((2 * caseHeight >> 1 * fillingHeight) - pow(fillingHeight, 2)) / pow(caseHeight >> 1, 2)))  ;
          r = caseHeight  / 2;
          fillingLiter = (r * r * 2 * acos(1 - fillingHeight / r) / 2 - 2 * sqrt(caseHeight * fillingHeight - fillingHeight * fillingHeight) * (r - fillingHeight) / 2) * caseWidth / DIVIDER;
          break;
        case 2:
          caseVolume = (caseHeight * caseWidth * caseLength) / DIVIDER;
          fillingLiter = (fillingHeight * caseWidth * caseLength) / DIVIDER;
          break;
        default:
          DPRINTLN(F("Invalid caseDesign")); DDECLN(caseDesign);
          break;
      }

      fillingPercent = (fillingLiter * 100) / caseVolume;
      DPRINT(F("Behaeltervolumen (gesamt): ")); DDEC(caseVolume); DPRINTLN(F("L")); 
      DPRINT(F("Inhalt                   : ")); DDEC(fillingLiter); DPRINT(F("L (")); DDEC(fillingPercent); DPRINTLN(F("%)"));
      DPRINTLN("");
    }
    virtual void trigger (__attribute__ ((unused)) AlarmClock & clock) {
      if (last_flags != flags()) {
        this->changed(true);
        last_flags = flags();
      }
      measure();
      tick = delay();
      msg.init(device().nextcount(), fillingPercent, fillingLiter, device().battery().current());
      device().sendPeerEvent(msg, *this);
      sysclock.add(*this);
    }

    uint32_t delay () {
      uint16_t _txMindelay = 20;
      _txMindelay = device().getList0().Sendeintervall();
      if (_txMindelay == 0) _txMindelay = 20;
      return seconds2ticks(_txMindelay  * SYSCLOCK_FACTOR);
    }

    void configChanged() {
      DPRINTLN(F("Config changed List1"));
      DPRINT(F("*CASE_HIGH:       "));
      DDECLN(this->getList1().caseHigh());
      DPRINT(F("*CASE_WIDTH:      "));
      DDECLN(this->getList1().caseWidth());
      DPRINT(F("*CASE_LENGTH:     "));
      DDECLN(this->getList1().caseLength());
      DPRINT(F("*CASE_DESIGN:     "));
      DDECLN(this->getList1().caseDesign());
      DPRINT(F("*DISTANCE_OFFSET: "));
      DDECLN(this->getList1().distanceOffset());
      DPRINT(F("*SENSOR_TYPE:     "));
      DDECLN(this->getList1().sensorType());
    }

    void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      vl53l0x.init();
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      uint8_t flags = this->device().battery().low() ? 0x80 : 0x00;
      return flags;
    }
};

class UType : public MultiChannelDevice<Hal, MeasureChannel, 1, UList0> {
  public:
    typedef MultiChannelDevice<Hal, MeasureChannel, 1, UList0> TSDevice;
    UType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr) {}
    virtual ~UType () {}

    virtual void configChanged () {
      TSDevice::configChanged();
      DPRINT(F("*LOW BAT Limit: "));
      DDECLN(this->getList0().lowBatLimit());
      this->battery().low(this->getList0().lowBatLimit());
      DPRINT(F("*Sendeintervall: ")); DDECLN(this->getList0().Sendeintervall());
    }
};

UType sdev(devinfo, 0x20);
ConfigButton<UType> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  DDEVINFO(sdev);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    hal.activity.savePower<Sleep<>>(hal);
  }
}



