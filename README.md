# stm32-vibration-and-accelerometer-public
Real-time vibration detection on STM32 (Nucleo-F401RE) with LSM6DSOX IMU at 104Hz. Tri-axis combination, 53-tap CMSIS-DSP FIR low-pass filter on 104-sample blocks, sliding-window RMS envelope extraction, threshold-driven LED actuation, UART debug output. C, bare-metal HAL/LL, deployed on real hardware.
