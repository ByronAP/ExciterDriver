/*
 * ExciterDriver - Calibration Helper Sketch
 * Version 1.0
 *
 * This utility sketch is used to perform the two-point voltage
 * calibration for the ExciterDriver hardware. It isolates the voltage
 * sensing circuit and reports the raw ADC (Analog-to-Digital Converter)
 * values, allowing you to accurately map them to real-world voltages.
 *
 * =========================================================================
 * == HOW TO USE
 * =========================================================================
 *
 * Required Equipment:
 * 1. Your assembled ExciterDriver hardware, connected to an Arduino Nano.
 * 2. A variable bench power supply capable of providing 10-16V.
 * 3. A trusted, accurate multimeter.
 *
 *
 * Calibration Steps:
 * ------------------
 *
 * 1.  CONNECT HARDWARE:
 *     - Connect the bench power supply's output to the ExciterDriver's main
 *       voltage input (the same input the car battery would connect to).
 *     - Connect your multimeter in parallel with the power supply to get a
 *       precise reading of the input voltage.
 *     - Connect the Arduino Nano to your computer via USB.
 *
 * 2.  UPLOAD THIS SKETCH:
 *     - Open this sketch (`CalibrationHelper.ino`) in the Arduino IDE.
 *     - Select "Arduino Nano" as the board.
 *     - Upload the sketch to your Arduino.
 *
 * 3.  OPEN SERIAL MONITOR:
 *     - Open the Arduino IDE's Serial Monitor (Tools > Serial Monitor).
 *     - Set the baud rate to 9600.
 *
 * 4.  PERFORM LOW-POINT CALIBRATION:
 *     - Turn on your power supply.
 *     - Carefully adjust the supply until your multimeter reads **exactly 12.00V**.
 *     - Observe the "Raw ADC Value" in the Serial Monitor. It may fluctuate slightly.
 *       Wait for it to stabilize and record the average value. This is your `CAL_LOW_ADC`.
 *       (Example: 742)
 *
 * 5.  PERFORM HIGH-POINT CALIBRATION:
 *     - Carefully adjust the supply until your multimeter reads **exactly 14.50V**.
 *     - Observe the "Raw ADC Value" again. Record the new stable, average value.
 *       This is your `CAL_HIGH_ADC`. (Example: 854)
 *
 * 6.  UPDATE THE MAIN SKETCH:
 *     - You now have your two calibration pairs (e.g., 12.00V = 742 and 14.50V = 854).
 *     - Open the main `ExciterDriver.ino` sketch.
 *     - Find the "VOLTAGE CALIBRATION & REGULATION" section at the top.
 *     - Replace the default values with your measured values:
 *
 *       // Before
 *       // const float CAL_LOW_VOLTS = 12.001;
 *       // const int   CAL_LOW_ADC   = 742;
 *       // const float CAL_HIGH_VOLTS = 14.499;
 *       // const int   CAL_HIGH_ADC   = 854;
 *
 *       // After (with your values)
 *       const float CAL_LOW_VOLTS = 12.00; // Your low voltage point
 *       const int   CAL_LOW_ADC   = 742;   // Your measured low ADC value
 *       const float CAL_HIGH_VOLTS = 14.50; // Your high voltage point
 *       const int   CAL_HIGH_ADC   = 854;   // Your measured high ADC value
 *
 * 7.  FINISH:
 *     - Your ExciterDriver is now calibrated! You can now upload the main sketch.
 */

// =============================================================================
// == CONFIGURATION
// =============================================================================
// The pin connected to your voltage divider circuit.
const int vSensePin = A0;

// The number of ADC readings to average for each output.
// This helps smooth out noise for a more stable reading.
const int SAMPLES_TO_AVERAGE = 16;


// =============================================================================
// == SETUP
// =============================================================================
void setup() {
  // Initialize Serial communication.
  Serial.begin(9600);
  
  // Wait for the serial port to connect. Needed for native USB ports.
  while (!Serial) {
    ; 
  }

  // Print welcome message and instructions to the user.
  Serial.println("\n\n--- ExciterDriver: Calibration Helper ---");
  Serial.println("This utility will report the raw ADC value from your voltage sensor.");
  Serial.println("\nInstructions:");
  Serial.println("1. Adjust power supply to your LOW calibration point (e.g., 12.00V).");
  Serial.println("2. Record the stable 'Raw ADC Value' shown below.");
  Serial.println("3. Adjust power supply to your HIGH calibration point (e.g., 14.50V).");
  Serial.println("4. Record the new stable 'Raw ADC Value'.");
  Serial.println("5. Update the constants in the main ExciterDriver.ino sketch.\n");
}


// =============================================================================
// == MAIN LOOP
// =============================================================================
void loop() {
  // To get a stable reading, take multiple samples and average them.
  long adcTotal = 0;
  for (int i = 0; i < SAMPLES_TO_AVERAGE; i++) {
    adcTotal += analogRead(vSensePin);
    // A small delay between readings can sometimes help with ADC stability.
    delay(2); 
  }
  
  // Calculate the average ADC value.
  int averageAdcValue = adcTotal / SAMPLES_TO_AVERAGE;

  // Print the final, averaged value to the serial monitor.
  Serial.print("Raw ADC Value: ");
  Serial.println(averageAdcValue);

  // Wait a moment before taking the next set of readings.
  delay(500);
}