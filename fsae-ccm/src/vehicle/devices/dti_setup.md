Abstract: Document DTI + Emrax 228 MV (enstroj). This is a copy and simplified version of the steps in the DTI Setup Guide.

Steps to perform:
- Ensure Motor, Phase leads, Encoder setup is correct
- Spin motor 1 full rotation, document start and end readings (should be +- 1 of each other)
- Perform OPEN loop sampling. This validates the current controller **without rotating the motor.** Before doing so, ensure
    - No active faults
    - No active limits
    - Motor parameters configured
    - Enable signal (if configured)
    - In Control Settings, Reference Ramp temporarily disabled (re-enable after validation)
    - In Control Settings, MFW Mode set to None
    - In General, Control Source set to CAN1

    - In tab Sampling → Active Sampling, click opneloop, the inverter injects the configured current step and records the response automatically, but the motor may not move
    - Download the sample and open it in waveform view, inspect the step response visually.
    - From DTI: "A well-tuned response shows a fast rise with minimal overshoot and no sustained oscillation. Significant overshoot or ringing indicates incorrect L_d/L_q values. A slow, sluggish response may indicate parameters that are too high."

- Encoder Offset Calibration (3A, 10A, 30A), then Encoder nonlinearity calibration (follow steps in DTI CAN Tool)
- Then enable MPTA and MFW one by one (see the manual)

Results
- Hardware Validation:
    - Inverter Precharged properly
    - Motor Spins freely (kind of )
    - TODO: Encoder readings (1 full Rotation)
    - TODO: Verify phase leads not shorting
- Calibration
    - Open Loop Testing
    - Encoder FOC
    - Encoder Nonlinearity
    - Closed Loop Testing
    - Manual Mode
    - CAN2 Mode
