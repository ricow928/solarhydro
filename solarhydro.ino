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


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

// number of analog samples to take per reading
#define NUM_SAMPLES 10

int panel_pin = 13;             // Pulls down to energize relay
float charge_upper = 28.8;      // Volts, above which turn off charging if time is satisfied
float charge_lower = 28.4;      // Volts, below which begin charging for at least time_lim cycles
bool charging = true;           // Default to enable charging
int charge_time = 0;            // Counter for number of cycles of charging
int time_lim = 120;             // Number of cycles to charge after dropping below min (estimate 500 ms/cycle)
int batt_a_pin = A2;            // battery voltage divider measure pin
int panel_a_pin = A0;           // panel voltage divider measure pin

// Setup-specific values
float Vcc = 5.03;               // Volts supplied by Arduino
float batt_r1 = 9.80;           // kohms, large resistor of battery bridge
float batt_r2 = 0.981;          // kohms, small resistor of battery bridge
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
      charge_time = 0;
    }
    if (v_batt > charge_upper && charge_time > time_lim){
      // Remove panel from battery, reset timer
      digitalWrite(panel_pin, HIGH);
      charging = false;
      charge_time = 0;
    }

    if (charging){
      charge_time += 1;
    }

    // Tell me what you know
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print("V_batt: ");
    display.print(v_batt,1); display.println("V");

    display.print("Deadband: "); display.print(charge_lower, 1);
    display.print("-"); display.print(charge_upper, 1); display.println("V");

    display.print("Charge: "); display.println(charging);

    display.print("Charging: "); display.print(charge_time); display.println(" cycles");
    display.display();

    
}
