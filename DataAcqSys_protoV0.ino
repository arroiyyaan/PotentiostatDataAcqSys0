// =============================================== //
// POTENTIOSTAT DATA ACQUISITION SYSTEM            
// INTERFACES:                                         
//    1. GENERAL PURPOSE COMPUTER (Serial USB)     
//    2. POTENTIOSTAT (DAC-ADC)                    
//    3. GALVANOSTAT (DAC-ADC)                     
// AUTHOR: RILWANU AR ROIYYAAN - TU/e              
// TARGET DEVICE: ARDUINO PORTENTA H7 
// STATUS: FOCUSING ON EXCITATION SIGNAL                         
// =============================================== //

// --------------------------------------------------------------------------------- //
// LIBRARYING
#include <Arduino.h>
#include <math.h>

// --------------------------------------------------------------------------------- //
// CYCLIC VOLTAMMETRY VARIABLES
// CV Constants
const int SigExcCVpin = A6;        // Analog pin A6 for output
const int stepsPerSecond = 10;   // 0.1 second per step means 10 steps per second
const int stepInterval = 1000;    // in ms
// Variables for ramp logic
int stepCount = 0;
int triangleCount = 0;
unsigned long lastTime = 0;
unsigned long waveDuration = 2;  // Total duration of each triangle wave (1 second for up, 1 second for down)
float voltageCV = 0;

// --------------------------------------------------------------------------------- //
// CHRONOAMPEROMETRY VARIABLES
// DC parameters
const int SigExcCApin = A7;  // Analog output pin (PWM-based output)
const float X = 2.0;       // DC voltage to generate initially
const int T = 170000/10;      // Duration of DC signal in ms
const int t = 10000;       // Duration of sine wave in ms
// Sine wave parameters
#define DAC_PIN A6  // Use A7 for output
#define PI 3.14159265358979323846
#define RESOLUTION 12  // Assuming a 12-bit DAC
#define VREF 3.3  // Reference voltage for DAC (adjust based on your board)
float Y = X;


// ================================================================================= //
// CYCLIC VOLTAMMETRY
// ================================================================================= //
void OutputtingSigExcCV(float minVoltage, float maxVoltage, int numTriangles) {
  // Calculate the total number of steps in one triangular wave cycle
  int totalSteps = waveDuration * stepsPerSecond; 
  
  // Check if the number of triangular waves is less than the target
  if (triangleCount < numTriangles) {
    // Determine if we are in the ramp-up or ramp-down phase
    if (stepCount < totalSteps / 2) {
      // Ramp-up phase: Increase voltage linearly
      voltageCV = minVoltage + stepCount * ((maxVoltage - minVoltage) / (totalSteps / 2));
    }
    else {
      // Ramp-down phase: Decrease voltage linearly
      voltageCV = maxVoltage - (stepCount - totalSteps / 2) * ((maxVoltage - minVoltage) / (totalSteps / 2));
    }

    // Output the calculated voltage to A6 pin
    int outputValueCV = map(voltageCV * 1000, 0, 3300, 0, 255);  // Convert voltage to PWM value
    analogWrite(SigExcCVpin, outputValueCV);

    // To serial plotter
    Serial.println(voltageCV, 3);
    
    // Increment or reset the step counter for next cycle
    stepCount++;
    if (stepCount >= totalSteps) {
      stepCount = 0;  // Reset the step count after one full triangle
      triangleCount++;  // Increment the triangle count
    }
  } else {
    // Stop the output after generating the required number of triangles
    analogWrite(SigExcCVpin, 0); // Stop the signal
    Serial.println("Wave generation completed.");
    while (true); // Keep the program from continuing further
  }
}

// ================================================================================= //
// CHRONOAMPEROMETRY
// ================================================================================= //
void OutputtingSigExcCADC(float A, int durationMs) {
  int outputValue = map(A * 1000, 0, 5000, 0, 255);  // Convert voltage to PWM value
  analogWrite(SigExcCApin, outputValue);

  Serial.println("Generating DC signal...");
  unsigned long startTime = millis();
  
  while (millis() - startTime < durationMs) {
    Serial.println(A, 3); // Print DC value for serial plotter
    delay(1000);  // Update every second
  }

  // Serial.println("DC signal completed.");
}
void OutputtingSigExcCAAC(float dc_offset, float frequency, float amplitude, float duration) {
    unsigned long startTime = millis();
    int sampleRate = 10000; // 10 kHz sample rate
    int numSamples = sampleRate / frequency;
    
    while (millis() - startTime < duration * 1000) {
        for (int i = 0; i < numSamples; i++) {
            float angle = (2 * PI * i) / numSamples;
            float sineValue = amplitude * sin(angle) + dc_offset;
            int dacValue = (sineValue / VREF) * ((1 << RESOLUTION) - 1);
            dacValue = constrain(dacValue, 0, (1 << RESOLUTION) - 1);
            analogWrite(DAC_PIN, dacValue);
            Serial.println(sineValue);
            delayMicroseconds(1000000 / sampleRate);
        }
    }
}

// ================================================================================= //
// CONTROL CODE
// ================================================================================= //
void setup() {
  Serial.begin(9600); 

  // CV PINS
  pinMode(SigExcCVpin, OUTPUT);

  // CA PINS
  pinMode(SigExcCApin, OUTPUT);
  OutputtingSigExcCADC(X, T);
  OutputtingSigExcCAAC(2, 100, 0.25, 10);

}

void loop() {

  // PERFORM: CYCLIC VOLTAMMETRY GENERATION
  // unsigned long currentTime = millis();
  // // Update every stepItvl
  // if (currentTime - lastTime >= stepInterval) {
  //   lastTime = currentTime;
  // // Generating ramps with lowest V, highest V, and no. triangles
  // OutputtingSigExcCV(1.0, 1.7, 3);  
  // }

  // PERFORM: CHRONOAMPEROMETRY
  // OutputtingSigExcCADC(X, T);
  // OutputtingSigExcCAAC(2, 100, 0.25, 10);

  delay(1000);
}
