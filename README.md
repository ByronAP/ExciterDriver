# ExciterDriver
<p style="display: flex; align-items: center;">
  <img src="assets/logo.svg" alt="ExciterDriver Log" style="width:60px; height:auto;">
  <span style="margin-left: 30px; font-size: 18px;">An Open-Source Alternator Regulator Field Controller</span>
</p>

[![Build Status](https://github.com/ByronAP/ExciterDriver/actions/workflows/main-pipeline.yml/badge.svg)](https://github.com/ByronAP/ExciterDriver/actions/workflows/main-pipeline.yml) [![Latest Release](https://img.shields.io/github/v/release/ByronAP/ExciterDriver)](https://github.com/ByronAP/ExciterDriver/releases) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

***

**A modern, robust, and fully-featured alternator voltage regulator field controller based on the Arduino Nano platform.**

This project replaces traditional, outdated internal or external alternator regulators with a smart, microcontroller-based system. It offers features like precise voltage control, tachometer-assisted engine state detection, soft-start for smooth engagement, and comprehensive fault detection with onboard diagnostics.

The name "ExciterDriver" comes from its core function: it is a smart **Driver** for the alternator's **Exciter** (or field) coil.

## Features

- **Precise Voltage Regulation:** Utilizes a PID control loop to maintain a stable target voltage (e.g., 14.2V), which is easily configurable.
- **Tachometer-Assisted Logic:** Uses an engine tachometer signal for fast and reliable detection of engine state (running/stopped), with an automatic fallback to voltage-only logic if no tach signal is present.
- **Soft Start:** Gently ramps up the alternator field current upon engine start, preventing sudden mechanical load on the engine and belts.
- **Robust Fault Detection:** Actively monitors for over-voltage, under-voltage, tachometer signal failure, and other fault conditions.
- **Onboard Diagnostics:** A unique "Diagnostic Mode" allows users to retrieve stored fault codes from the previous run cycle by simply using a push-button—no computer required.
- **Hardware Protection:** Designed with real-world automotive electrical systems in mind, with logic to handle signal noise, voltage sags, and other common issues.
- **Configurable & Open Source:** All key parameters (target voltage, PID gains, timings) are easily adjustable in the source code.

## Repository Structure

```
.
├── assets/                # Images and Misc
├── hardware/              # Schematics, PCB layouts, and hardware documentation
├── src/ExciterDriver      # Main Arduino source code (ExciterDriver.ino)
├── src/CalibrationHelper  # Calibration Arduino source code (CalibrationHelper.ino)
├── CODE_OF_CONDUCT.md     # Contributor Code of Conduct
├── CONTRIBUTING.md        # Contributor guidelines and style guide
├── LICENSE                # MIT License
└── README.md              # This file
```

## Hardware Requirements

A detailed schematic can be found in the `hardware/` directory. The core components are:

- **Arduino Nano:** The brain of the system.
- **Logic-Level N-Channel MOSFET:** To control the high-current alternator field coil. A recommended model is the **IRLR3110Z** or similar logic-level FET capable of handling >10A.
- **Voltage Divider:** To safely measure the battery voltage with the Arduino's analog input.
  - **R1:** 22kΩ (from Battery+ to A0)
  - **R2:** 10kΩ (from A0 to GND)
- **Protection Components:**
  - A 5.1V Zener diode and a 100Ω resistor on the analog input pin for over-voltage protection.
- **LEDs and Resistors:** For status, field, and error indicators.
  - **Field LED (D10):** Shows the activity of the alternator field.
  - **Error LED (D11):** Illuminates when a fault is detected.
- **Diagnostic PIN (A3):** A momentary push-button or jumper connected between pin `A3` and `GND` for accessing the diagnostic mode.

**Important:** Ensure all components share a common ground with the vehicle's chassis and battery.

## Software Setup

1. **Install Arduino IDE:** Download and install the latest version from the [Arduino website](https://www.arduino.cc/en/software).
2. **Open the Sketch:** Open the `src/ExciterDriver/ExciterDriver.ino` file in the Arduino IDE.
3. **Board Selection:** In the IDE, go to `Tools > Board` and select "Arduino Nano".
4. **Processor Selection:** Go to `Tools > Processor` and select "ATmega328P". Depending on your Nano clone, you may need to select "ATmega328P (Old Bootloader)".
5. **Upload:** Connect your Arduino Nano via USB and upload the sketch.

### Initial Calibration (Recommended But Not Required)

For maximum accuracy, the voltage sensing circuit should be calibrated to your specific components using 2 points.

1. **Upload the Calibration Sketch:** A helper sketch for calibration can be found in the `src/CalibrationHelper/` directory.
2. **Apply Known Voltages:** Using a variable bench power supply and a trusted multimeter, apply two known voltages (e.g., 12.0V and 14.5V) to the regulator's input.
3. **Record ADC Values:** Note the raw ADC values reported by the calibration sketch for each voltage point.
4. **Update `ExciterDriver.ino`:** Enter your measured values into the `VOLTAGE CALIBRATION & REGULATION` section at the top of the main source file.

```cpp
// Update these values with your specific bench measurements.
const float CAL_LOW_VOLTS = 12.001;
const int   CAL_LOW_ADC   = 742;
const float CAL_HIGH_VOLTS = 14.499;
const int   CAL_HIGH_ADC   = 854;
```

## Using the Onboard Diagnostics

This system can store fault codes from its last run cycle. To retrieve them:

1. **Power Down:** Ensure the device is completely powered off.
2. **Press and Hold:** Connect the diagnostic pin `A3` or button to ground.
3. **Power On:** While still connected to ground, apply power to the device.
4. **Observe the Error LED:** The device will now enter Diagnostic Mode. The `Error LED` (D11) will blink to indicate any stored fault codes.

**Fault Codes:**
- **1 Blink:** Over-Voltage Fault
- **2 Blinks:** Under-Voltage Fault (while running)
- **3 Blinks:** Alternator Probing Timeout
- **4 Blinks:** Tachometer Signal Failure

If multiple faults are stored, the system will blink the first code, pause for 2 seconds, blink the next code, and so on. If no faults are found, the Error LED will turn on solid for 2 seconds.

The device will repeat the sequence every 5 seconds until it is power-cycled.

**Normal Operation:** To run the regulator normally, simply power it on *without* grounding the diagnostic pin or button. This will clear the fault log from the previous run and begin normal operation.

## Contributing

Contributions are welcome! Please feel free to open an issue to report bugs or suggest features. If you would like to contribute code, please fork the repository and submit a pull request.

Please read our [CONTRIBUTING.md](CONTRIBUTING.md) before contributing.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.