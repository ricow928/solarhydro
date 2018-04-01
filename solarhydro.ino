#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

int panel_pin = 13;             // Pulls down to energize relay
float charge_upper = 28.8;      // Volts, above which turn off charging if time is satisfied
float charge_lower = 28.4;      // Volts, below which begin charging for at least time_lim cycles
float max_v_batt = 34.0;        // Volts, if battery is over this number, disable charging
bool charging = true;           // Default to enable charging
int charge_time = 0;            // Counter for number of cycles of charging
int time_lim = 60;              // Number of cycles to charge after dropping below min (estimate 1000 ms/cycle)
int batt_a_pin = A2;            // battery voltage divider measure pin
int panel_a_pin = A0;           // panel voltage divider measure pin

float panel_voc = 0.0;          // Global to hold panel Voc

// Setup-specific values
float Vcc = 5.03;               // Volts supplied by Arduino
float batt_r1 = 9.78;           // kohms, large resistor of battery bridge
float batt_r2 = 0.979;          // kohms, small resistor of battery bridge
float panel_r1 = 9.75;          // kohms, large resistor of panel bridge
float panel_r2 = 0.979;         // kohms, small resistor of panel bridge


float voltage(float a_in, float v_ref, float r1, float r2) {
  // implement standard voltage bridge equation based on 1024 sample ADC
  // calculate analog input voltage
  float v_pin = v_ref * a_in / 1024.0;
  // calculate resistor bridge multiplier
  float ratio = (r1 + r2) / r2;
  // multiply measured voltage by ratio to get actual voltage
  float v_object = v_pin * ratio;

  return v_object;
}

float pin_average(int input_name, int num_samp, int dwell) {
  // initialize local counting vars
  unsigned char sample_count = 0;
  int sum = 0;
  // Loop num_samp times, measuring pin and adding val to sum var
  while (sample_count < num_samp) {
        sum += analogRead(input_name);
        sample_count++;
        delay(dwell);
    }
  // average the measurements
  float avg_ain = (float)sum / (float)num_samp;

  return avg_ain;
}

void setup()
{
    Serial.begin(9600);
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
    // init done
    display.display();
    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("Hello Hydro!");
    display.display();

    pinMode(panel_pin, OUTPUT);

}

void loop()
{
    // MEASURE BATTERY VOLTAGE
    float batt_ain = pin_average(batt_a_pin, 10, 50);
    // calculate battery voltage
    float v_batt = voltage(batt_ain, Vcc, batt_r1, batt_r2);
    Serial.print("Battery: "); Serial.print(v_batt); Serial.println (" V");

    if (v_batt < charge_lower){
      // Apply panel to battery, start timer
      digitalWrite(panel_pin, LOW);
      charging = true;
    }

    if ((v_batt > charge_upper && charge_time > time_lim) || (v_batt > max_v_batt)){
      // Remove panel from battery, reset timer
      digitalWrite(panel_pin, HIGH);
      charging = false;
      charge_time = 0;
    }

    // For now, just measure the panel voltage even if it might be
    // connected to the battery
    // Get the analog pin measurement
    float panel_ain = pin_average(panel_a_pin, 10, 50);
    // calculate panel voltage
    float v_panel = voltage(panel_ain, Vcc, panel_r1, panel_r2);
    Serial.print("Panel: "); Serial.print(v_panel); Serial.println (" V");
    

    if (charging){
      charge_time += 1;
    }

    // Let's see about the open-circuit voltage
    // if the panel is disconnected, VOC is just the panel voltage as-is
    if (!charging){
      panel_voc = v_panel;
    }
    // if the panel is connected, don't necessarily want to interrupt charging to measure,
    // so only do once
    if (charging && charge_time == 10){
      // Tell me what you're doing
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Grabbing panel VOC");
      display.display();
      // Disconnect panel
      digitalWrite(panel_pin, HIGH);
      // Wait two seconds to let voltage recover (may need more, TBD)
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Waiting two sec");
      display.display();
      delay(2000);
      // Get the analog pin measurement
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Measuring VOC");
      display.display();
      panel_ain = pin_average(panel_a_pin, 10, 50);
      // calculate panel voltage
      panel_voc = voltage(panel_ain, Vcc, panel_r1, panel_r2);
      // Reconnect panel
      digitalWrite(panel_pin, LOW);
    }

    if (charge_time > time_lim){
      charge_time = 0;
    }

    // Tell me what you know
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print("V_batt: ");
    display.print(v_batt,1); display.println("V");

    display.print("Vpanel: "); display.print(v_panel,1); display.println("V");

    display.print("Panel VOC: "); display.print(panel_voc,1); display.println("V");

    display.print("Deadband: "); display.print(charge_lower,1);
    display.print("-"); display.print(charge_upper,1); display.println("V");

    display.print("Charge: "); display.println(charging);

    display.print("Charging: "); display.print(charge_time); display.println(" cycles");
    display.display();

    
}
