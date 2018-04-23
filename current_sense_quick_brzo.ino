#include <brzo_i2c.h>

#define INA_SCL D2
#define INA_SDA D1

#include <INA219_brzo.h>
INA219_brzo ina(INA_SCL, INA_SDA); //INA219_brzo ina(D2,D1)

#define MAX_CURRENT 0.50 // A
unsigned long t1 = 0;
unsigned long t2 = 0;

float val_I = 0.0;
float val_Vbus = 0.0;
float val_Vshunt = 0.0;
float val_Vtotal = 0.0;
float val_Vpower = 0.0;

void setup() {
  unsigned long BAUDRATE = 115200;
  //BAUDRATE = 460800;
  //BAUDRATE = 921600;
  //BAUDRATE = 1500000;
  //BAUDRATE = 2000000;

  Serial.begin(BAUDRATE);
  Serial.println("Initialize INA219");
  Serial.println("-----------------------------------------------");
  ina.begin(0x40);
  //ina.configure(INA219_RANGE_16V, INA219_GAIN_40MV, INA219_BUS_RES_9BIT, INA219_SHUNT_RES_9BIT_1S, INA219_MODE_SHUNT_CONT);
  ina.configure(INA219_RANGE_16V, INA219_GAIN_40MV, INA219_BUS_RES_9BIT, INA219_SHUNT_RES_9BIT_1S, INA219_MODE_SHUNT_BUS_CONT);
  //ina.configure(INA219_RANGE_16V, INA219_GAIN_40MV, INA219_BUS_RES_12BIT, INA219_SHUNT_RES_12BIT_16S, INA219_MODE_SHUNT_BUS_CONT);
  ina.calibrate(0.1, MAX_CURRENT); // 0.1ohm, 50mA
  ina.checkConfig();
}
float val = 0.0;
boolean PlotTimeStep = true;
boolean PlotTime = false;

boolean PlotBusVoltage = false;
boolean PlotShuntVoltage = false;
boolean PlotCurrent = false;
boolean PlotTotalVoltage = true;
boolean PlotPower = true;
uint16_t cc = 1;
unsigned long avg_timestep = 0;
unsigned long old_time = 0;
int8_t SIG_DIGITS = 2;

boolean DataSaveMode = false;

void loop() {

  t1 = micros();
  if (PlotBusVoltage) {
    val_Vbus = ina.readBusVoltage();
  }
  if (PlotShuntVoltage) {
    val_Vshunt = ina.readShuntVoltage();
  }
  if (PlotCurrent) {
    val_I = ina.readShuntCurrent();
  }
  if (PlotTotalVoltage) {
    if (~PlotBusVoltage) {
      val_Vbus = ina.readBusVoltage();
    }
    if (~PlotShuntVoltage) {
      val_Vshunt = ina.readShuntVoltage();
    }
    val_Vtotal = val_Vshunt + val_Vbus;
  }

  if (PlotPower) {
    val_Vpower = ina.readBusPower();
  }
  t2 = micros();

  avg_timestep += (t2 - t1);
  if (cc % 100 == 0) {
    avg_timestep /= cc;
    if (PlotTimeStep) {
      Serial.print(";");
      if (avg_timestep <= 255) {
        Serial.print((uint8_t)avg_timestep);
      } else if (avg_timestep <= 65535) {
        Serial.print((uint16_t)avg_timestep);
      } else {
        Serial.print(avg_timestep);
      }


    }
    if (PlotTime) {
      Serial.print(";");
      unsigned long time_val = (t1 / 1000);
      if (time_val <= 255) {
        Serial.print((uint8_t)time_val);
      } else if (time_val <= 65535) {
        Serial.print((uint16_t)time_val);
      } else if (time_val <= 4294967295) {
        Serial.print((uint32_t)time_val);
      } else {
        Serial.print(time_val);
      }
    }
    cc = 0;
  }
  cc++;

  //  if (PlotTime) {
  //    if ((t1-old_time) > (10 * 1000)) { // alle 10ms
  //      Serial.print(t1-old_time);
  //      Serial.print(";");
  //      old_time = t1;
  //    }
  // }

  if (DataSaveMode) {
    if (PlotCurrent) {
      Serial.print(";");
      if (abs(val_I * 10000) <= 0xFF) {
        Serial.print((int16_t)(val_I * 10000));
      } else {
        Serial.print((int8_t)(val_I * 10000));
      }

    }
    if (PlotBusVoltage) {
      Serial.print(";");
      if (abs(val_Vbus * 10000) <= 0xFF) {
        Serial.print((int16_t)(val_Vbus * 10000));
      } else {
        Serial.print((int8_t)(val_Vbus * 10000));
      }

    }
    if (PlotShuntVoltage) {
      Serial.print(";");
      if (abs(val_Vshunt * 10000) <= 0xFF) {
        Serial.print((int16_t)(val_Vshunt * 10000));
      } else {
        Serial.print((int8_t)(val_Vshunt * 10000));
      }
    }
    if (PlotTotalVoltage) {
      Serial.print(";");
      if (abs(val_Vtotal * 10000) <= 0xFF) {
        Serial.print((int16_t)(val_Vtotal * 10000));
      } else {
        Serial.print((int8_t)(val_Vtotal * 10000));
      }
    }
    if (PlotPower) {
      Serial.print(";");
      if (abs(val_Vpower * 10000) <= 0xFF) {
        Serial.print((int16_t)(val_Vpower * 10000));
      } else {
        Serial.print((int8_t)(val_Vpower * 10000));
      }

    }
  } else {
    if (PlotCurrent) {
      Serial.print(";");
      Serial.print(val_I, SIG_DIGITS);
    }
    if (PlotBusVoltage) {
      Serial.print(";");
      Serial.print(val_Vbus, SIG_DIGITS);

    }
    if (PlotShuntVoltage) {
      Serial.print(";");
      Serial.print(val_Vshunt, SIG_DIGITS);

    }
    if (PlotPower) {
      Serial.print(";");
      Serial.print(val_Vpower, SIG_DIGITS);

    }
  }
  Serial.println("");
}
