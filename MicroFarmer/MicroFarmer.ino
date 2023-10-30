#include <DS18B20.h>

#define INLET_TEMP_PIN 7
DS18B20 inlet_temp(INLET_TEMP_PIN);

float get_inlet_temp() {
  return inlet_temp.getTempC();
}

#define HIGH_SIDE_PIN 2
#define LOW_SIDE_PIN 4
#define NON_INVERTING_INPUT A0
#define OUTPUT A1
#define INVERTING_INPUT A2
#define REFERENCE_VOLTAGE 5.0
#define SAMPLES 60
#define CALIBRATION_CONSTANT 1.0

int high_side_state = LOW;
int low_side_state = LOW;

float get_inlet_ec() {
  // make sure probe is off for 100 ms to remove any current from previous reading
  digitalWrite(HIGH_SIDE_PIN, LOW);
  digitalWrite(LOW_SIDE_PIN, LOW);
  delay(100);

  float v_n_in_readings[SAMPLES];
  float v_in_readings[SAMPLES];
  float v_out_readings[SAMPLES];

  // get voltage readings
  for (int i = 0; i < SAMPLES; i++) {
    // prevent shoot through by swithing off the active side first
    if (high_side_state == HIGH || (high_side_state == LOW && low_side_state == LOW)) {
      high_side_state = LOW;
      digitalWrite(HIGH_SIDE_PIN, high_side_state);
      delay(1);
      low_side_state = HIGH;
      digitalWrite(LOW_SIDE_PIN, low_side_state);
    }else if (low_side_state == HIGH) {
      low_side_state = LOW;
      digitalWrite(LOW_SIDE_PIN, low_side_state);
      delay(1);
      high_side_state = HIGH;
      digitalWrite(HIGH_SIDE_PIN, high_side_state);
    }
    delay(1);

    float v_n_in_reading = (analogRead(NON_INVERTING_INPUT) / 1023.0 * REFERENCE_VOLTAGE);
    v_out_readings[i] = abs((analogRead(OUTPUT) / 1023.0 * REFERENCE_VOLTAGE) - v_n_in_reading);
    v_in_readings[i] = abs((analogRead(INVERTING_INPUT) / 1023.0 * REFERENCE_VOLTAGE) - v_n_in_reading);
  }
  digitalWrite(HIGH_SIDE_PIN, LOW);
  digitalWrite(LOW_SIDE_PIN, LOW);

  // calculate average
  float v_in_avg = 0.0;
  float v_out_avg = 0.0;
  for (int i = 0; i < SAMPLES; i++) {
    v_in_avg += v_in_readings[i];
    v_out_avg += v_out_readings[i];
  }
  v_in_avg /= SAMPLES;
  v_out_avg /= SAMPLES;

  // calculate average deviation
  float v_in_dev_avg = 0.0;
  float v_out_dev_avg = 0.0;
  for (int i = 0; i < SAMPLES; i++) {
    v_in_dev_avg += abs(v_in_readings[i] - v_in_avg);
    v_out_dev_avg += abs(v_out_readings[i] - v_out_avg);
  }
  v_in_dev_avg /= SAMPLES;
  v_out_dev_avg /= SAMPLES;

  // calculate filtered avg
  float v_in = 0.0;
  float v_out = 0.0;
  int clean_samples = 0;
  for (int i = 0; i < SAMPLES; i++) {
    if (abs(v_in_readings[i] - v_in_avg) > v_in_dev_avg || abs(v_out_readings[i] - v_out_avg) > v_out_dev_avg) {
      continue;
    }
    clean_samples++;
    v_in += v_in_readings[i];
    v_out += v_out_readings[i];
  }
  v_out /= clean_samples;
  v_in /= clean_samples;

  float r_in = (v_in / v_out * 220.0) - 1000.0;
  return (1 / r_in / 0.000001) * 0.3;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReference(DEFAULT);
  pinMode(HIGH_SIDE_PIN, OUTPUT);
  pinMode(LOW_SIDE_PIN, OUTPUT);
}

void loop() {
  float temp = get_inlet_temp();
  double ec = get_inlet_ec();
  double log_ec = log10(ec) / log10(7);
  double ec25 = ec / (1 + (0.02 * (temp - 25.0)));
  int ppm = ec25 * (0.01208 * ec25 + 1.503) * 0.7;

  Serial.println("Temp: " + String(temp) + ", EC: " + String(ec) + ", Log(EC): " + String(log_ec) + ", EC25: " + String(ec25) + ", PPM: " + String(ppm));
  delay(1000);
}
