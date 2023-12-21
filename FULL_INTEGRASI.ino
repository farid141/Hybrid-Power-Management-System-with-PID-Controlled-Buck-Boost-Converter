#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PZEM004Tv30.h>
#include <PWM.h>

#define ADC_SCALE 1023.0
#define VREF 5.0
#define zero 510.0
#define ACS_sensitivity 66.0  //30Ampere

#define CONV_OFF 0
#define CONV_BUCK 1
#define CONV_BOOST 2

#define BATT_CHARGE 0
#define BATT_DISCHARGE 1

PZEM004Tv30 pzem(Serial1);
LiquidCrystal_I2C lcd(0x27, 20, 4);

float tegangan_inverter, tegangan_turbin, tegangan_batt, tegangan_charging;
float arus_inverter, arus_turbin, arus_batt, arus_charging;
float tegangan_pzem, arus_pzem;

const int RL_TURBIN = 7, RL_INVERTER = 8, RL_PLN = 9, RL_BATT = 10;
const int PWM1 = 12, PWM2 = 11;

// Konstanta PID
const double KP = 0.05;  // Konstanta Proporsional
const double KI = 0.6;   // Konstanta Integral
const double KD = 0.0;   // Konstanta Derivatif

// Variabel PID
const double setpoint = 14.8;
double Lerr = 0.0;
double Derr = 0.0;
double Ierr = 0.0;
double output = 0.0;
unsigned long prev_time = 0;

int mode = CONV_OFF, batt_status, lcd_flag = 0;
unsigned long batt_time = 0, lcd_time = 0;

void setup() {
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe();

  Serial.begin(115200);

  //sets the frequency for the specified pin
  bool success1 = SetPinFrequencySafe(PWM1, 40000);
  bool success2 = SetPinFrequencySafe(PWM2, 40000);

  //if the pin frequency was set successfully, turn pin 13 on
  if (success1 & success2) {
    Serial.println("success");
  }

  pzem.readAddress();
  lcd.begin();
  lcd.backlight();

  pinMode(RL_TURBIN, OUTPUT);
  pinMode(RL_INVERTER, OUTPUT);
  pinMode(RL_PLN, OUTPUT);
  pinMode(RL_BATT, OUTPUT);

  //mati semua
  digitalWrite(RL_TURBIN, HIGH);
  digitalWrite(RL_INVERTER, HIGH);
  digitalWrite(RL_PLN, HIGH);
  digitalWrite(RL_BATT, HIGH);
  converter_off();

  // Baca batre pertama kali
  digitalWrite(RL_BATT, HIGH);  //putus relay conv-batre
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("cek batre...");
  Serial.println("baca batre.... (delay 5 detik)");
  delay(5000);
  update_battMode();  // baca batre
  batt_time = millis();
}

void loop() {
  // WAKTU INTERVAL CEK BATRE
  if (millis() - batt_time > 30000) {
    if (mode != CONV_BOOST) {
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("cek batre...");
      Serial.println("baca batre.... (delay 15 detik)");
      converter_off();
      digitalWrite(RL_BATT, HIGH);  //putus relay conv-batre
      delay(15000);
      update_battMode();  // baca batre
      batt_time = millis();
      mode = CONV_OFF;
    }
  }


  tegangan_inverter = getVoltageDivider(A0);
  arus_inverter = getCurrentACS(A1);
  tegangan_turbin = getVoltageDivider(A2);
  arus_turbin = getCurrentACS(A3);

  Serial.print("Vinverter:");
  Serial.print(tegangan_inverter);
  Serial.print(",");
  Serial.print("Iinverter:");
  Serial.print(arus_inverter);
  Serial.print(",");

  Serial.print("Vturbin:");
  Serial.print(tegangan_turbin);
  Serial.print(",");
  Serial.print("Iturbin:");
  Serial.print(arus_turbin);
  Serial.print(",");

  Serial.print("Vbatt:");
  Serial.print(tegangan_batt);
  Serial.print(",");
  Serial.print("Ibatt:");
  Serial.print(arus_batt);

  if (batt_status == BATT_DISCHARGE)
    Serial.println("  (DISCHARGE)");
  else if (batt_status == BATT_CHARGE)
    Serial.println("  (CHARGE)");
  update_lcd();


  if (tegangan_turbin >= 15) {
    Serial.print("turbin memenuhi   ");

    // JIKA BATRE KOSONG, ISI
    if (batt_status == BATT_CHARGE) {
      Serial.println("batre kosong");
      // SWITCH KE BUCK
      if (mode != CONV_BUCK) {
        Serial.println("\nmode buck baru! (delay 5 detik)");
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("switching buck...");
        delay(5000);

        // teest
        // tujuan: menentukan nilai Ierr awal agar output pid dekat dg setpoint
        // target duty = output pid awal
        // target duty = kp*err + ki*ierr
        // ierr = (target duty - kp*err)/ki
        tegangan_turbin = getVoltageDivider(A2);
        float target_duty = 14.4 / tegangan_turbin;
        Ierr = (target_duty - (KP * (14.4 - tegangan_turbin))) / KI;
        Lerr = 14.4 - tegangan_turbin;
        Derr = 0;
        Serial.print("Ierr = ");
        Serial.println(Ierr);
        Serial.println("*********************");
        delay(100);

        prev_time = millis();
      }

      mode = CONV_BUCK;
      // BATRE DICAS, PAKE PLN
      digitalWrite(RL_TURBIN, LOW);
      digitalWrite(RL_INVERTER, HIGH);
      digitalWrite(RL_PLN, HIGH);
      digitalWrite(RL_BATT, LOW);

      tegangan_charging = getVoltageDivider(A4);  //baca output buck ke batre (tegangan charging)
      arus_charging = getCurrentACS(A5);
      Serial.print("\nVcharging:");
      Serial.print(tegangan_charging);
      Serial.print(",");
      Serial.print("Icharging:");
      Serial.println(arus_charging);

      // pid
      pidBuck(tegangan_charging, 0.98);                                 // pid 
      converter_buck((int)(output * 255.0));                            // pid
      // converter_buck((int)((14.4/tegangan_turbin) * 255.0 * 1.05));  // rumus
      // converter_buck((int)(255.0 * 0.63));                           // konstan turbin 24V
    }

    // JIKA BATRE PENUH, PAKAI BATRE
    else if (batt_status == BATT_DISCHARGE) {
      Serial.println("batre penuh");
      // JIKA MODE SEBELUMNYA BUKAN BOOST
      if (mode != CONV_BOOST) {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("switching boost...");
        Serial.println("\nmode boost baru! (delay 5 detik)");
        delay(5000);
      }

      mode = CONV_BOOST;

      // Pakai batre ke inverter
      digitalWrite(RL_TURBIN, HIGH);
      digitalWrite(RL_INVERTER, LOW);
      digitalWrite(RL_BATT, LOW);
      digitalWrite(RL_PLN, LOW);

      converter_boost((int)(0.45 * 255));  //0.45 Duty (BATRE (11-13) Turbin (21-24))
      update_battMode();
      batt_time = millis();
    }
  }

  else {
    Serial.print("turbin kurang   ");
    //BATERAI PENUH, TURBIN LOW
    if (batt_status == BATT_DISCHARGE) {
      Serial.println("Batre penuh");
      if (mode != CONV_BOOST) {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("switching boost..");
        Serial.println("\nmode boost baru! (delay 5 detik)");
        delay(5000);
      }

      mode = CONV_BOOST;

      // Pakai batre ke inverter
      digitalWrite(RL_TURBIN, HIGH);
      digitalWrite(RL_INVERTER, LOW);
      digitalWrite(RL_BATT, LOW);
      digitalWrite(RL_PLN, LOW);

      converter_boost((int)(0.45 * 255));  //0.45 Duty (PATEN)
      update_battMode();
      batt_time = millis();
    }

    // BATERAI HABIS, TURBIN LOW
    else if (batt_status == BATT_CHARGE) {
      mode = CONV_OFF;
      Serial.println("Batre Habis");
      // 1. Converter mati
      converter_off();

      // 2. pake PLN
      digitalWrite(RL_TURBIN, HIGH);
      digitalWrite(RL_INVERTER, HIGH);
      digitalWrite(RL_PLN, HIGH);
      digitalWrite(RL_BATT, HIGH);
      update_battMode();
      batt_time = millis();
    }
  }
  Serial.println("=========================================");
}

void update_lcd() {
  if ((millis() - lcd_time) > 1000) {
    lcd_time = millis();
    if (lcd_flag == 0) {
      tampilkan_sensor();
      lcd_flag = 1;
    }

    else if (lcd_flag == 1) {
      bacaPZEM();
      tampilkan_pzem();
      lcd_flag = 0;
    }
  }
}

void update_battMode() {
  // 1. Baca baterai
  tegangan_batt = getVoltageDivider(A4);
  arus_batt = getCurrentACS(A5);

  // 2. Tentukan Status Batre
  if (tegangan_batt > 12.7)
    batt_status = BATT_DISCHARGE;
  else if (tegangan_batt < 12)
    batt_status = BATT_CHARGE;
}

void converter_buck(int duty) {
  // BUCK (12)
  Serial.print("ngebuck:");
  Serial.println(duty);
  pwmWrite(PWM2, 0);     // PWM2
  pwmWrite(PWM1, duty);  // PWM1
}

void converter_boost(int duty) {
  Serial.print("ngeboost:");
  Serial.println(duty);
  // BOOST (11)
  pwmWrite(PWM1, 0);     // PWM1
  pwmWrite(PWM2, duty);  // PWM2
}

void converter_off() {
  // OFF
  pwmWrite(PWM2, 0);  // PWM2
  pwmWrite(PWM1, 0);  // PWM1
}

void tampilkan_sensor() {
  // LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bid. Converter");

  lcd.setCursor(0, 1);
  lcd.print("v-1: ");
  lcd.print(tegangan_inverter);
  lcd.print(" i-1: ");
  lcd.print(arus_inverter);

  lcd.setCursor(0, 2);
  lcd.print("v-2: ");
  lcd.print(tegangan_turbin);
  lcd.print(" i-2: ");
  lcd.print(arus_turbin);

  lcd.setCursor(0, 3);
  lcd.print("v-3: ");
  lcd.print(tegangan_batt);
  lcd.print(" i-3: ");
  lcd.print(arus_batt);
}

void tampilkan_pzem() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bid. Converter");

  lcd.setCursor(0, 1);
  lcd.print("tegangan pzem: ");
  lcd.print(tegangan_pzem);
  lcd.setCursor(0, 2);
  lcd.print("arus pzem: ");
  lcd.print(arus_pzem);
}

float getVoltageDivider(int divider_pin) {
  float adcVal;
  for (int i = 0; i < 100; i++) {
    adcVal += analogRead(divider_pin);
    delayMicroseconds(50);
  }
  adcVal /= 100;
  float voltOut = (adcVal + 2.373) / 39.75;
  return voltOut;
}

float getCurrentACS(int acs_pin) {
  int adc_buff = 0;

  for (int i = 0; i < 20; i++) {
    adc_buff += (analogRead(acs_pin) - zero);
    delayMicroseconds(50);
  }
  float adc_avg = adc_buff / 20;
  float adc_volt = (adc_avg / ADC_SCALE) * VREF * 1000;  // dalam mV

  float I = fabs(adc_volt / ACS_sensitivity);
  return I;
}

void bacaPZEM() {
  tegangan_pzem = pzem.voltage();  //volt
  arus_pzem = pzem.current();      //ampere
  // float power = pzem.power();
  // float energy = pzem.energy();
  // float frequency = pzem.frequency();
  // float pf = pzem.pf();
}

void pidBuck(float input, float maxDuty) {
  unsigned long now = millis();
  double delta_time = (now - prev_time) / 1000.0;
  prev_time = now;

  double error = setpoint - input;

  //PERHITUNGAN
  Derr = (error - Lerr) / delta_time;
  Ierr += (error * delta_time);
  Lerr = error;  // Simpan error saat ini untuk  iterasi berikutnya

  output = (KP * error) + (KI * Ierr) + (KD * Derr);
  Serial.print("\nerror:");
  Serial.print(error);
  Serial.print(",");

  Serial.print("Ierr:");
  Serial.print(Ierr);
  Serial.print(",");

  // Batasan output agar tidak melampaui rentang yang diinginkan
  if (output > maxDuty) {
    output = maxDuty;
  } else if (output < 0) {
    output = 0;
  }

  Serial.print("out_pid:");
  Serial.println(output);
}