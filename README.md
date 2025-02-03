# momath.tech.clearcore-tcp-server
Clearcore TCP Server project, along with a key-value protocol, startup homing sequence

Motor Setup Requirements (Using ClearPath MSP 2.0 Software, plugged into the micro-USB connector of the motor, and after performing an AutoTune.

1. A ClearPath motor must be connected to Connector M-0.
2. The connected ClearPath motor must be configured through the MSP software for Step and Direction mode (In MSP select Mode>>Step and Direction).
3. The ClearPath motor must be set to use the HLFB mode "ASG-Position w/Measured Torque" with a PWM carrier frequency of 482 Hz through the MSP software (select Advanced>>High Level Feedback [Mode]... then choose "ASG-Position w/Measured Torque" from the dropdown, make sure that 482 Hz is selected in the "PWM Carrier Frequency" dropdown, and hit the OK button).
4. Set the Input Format in MSP for "Step + Direction".

Requires this library:
![image](https://github.com/user-attachments/assets/c7ef534c-c374-4e3d-96f8-d11edbecd31d)

Requires [PlatformIO](https://platformio.org/) for VSCode
