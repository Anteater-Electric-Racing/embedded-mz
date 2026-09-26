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
    - DONE: Inverter Precharged properly
    - DONE: Motor Spins freely by hand (kind of )
    - DONE: Encoder readings (1 full Rotation)
    - DONE by electrical leads: Verify phase leads not shorting
- Calibration and Spin (4/30/2026)
    - FAIL Open Loop Testing
    - DONE Encoder FOC
    - DONE Encoder Nonlinearity
    - DONE Closed Loop Testing
    - DONE Manual Mode
    - DONE Run via CAN2 Mode


Calibration 2nd Round: 5/1/2026
- Redo Encoder Calibration sequence (3 times)
- Document offsets, take avg of the 3
- Offset 1: 170.35
- Offset 2: 169.91
- Offset 3: 170.35
- Average: 170.17

delayed: MTPA and MFW checks
Use passive sampling tool to ensure good data
For MFW spin to target rpm and remove target and then fall

Control via CCM
Ensure DTI CAN config is right
Test Brake Command (Regen), Test forward and reverse AC Current Command, ensure proper feedback and scaling of data.

Try Encoder Non-linearity test
Spin via external soruce is ideal, but likely spin via CCM
If not good data figure sum out

If time permits do testing of speed control/FOC current control


Document everything in a .md file for now and add to github
