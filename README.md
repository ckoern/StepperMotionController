Stepper Motion Controller
-------

**THIS REPO IS IN EARLY DEVELOPMENT**

A simple stepper controller, allowing to perform movments via UART command. The controller outputs signals for a clasic STEP/DIR interface, 3 microstepping configuration lines and inputs for limit switches.
STEP pulses are generated using a timer module to keep resource consumption low.

UART interface is based on a 9-byte long protocol, similar to Trinamics TMCL commands.

The code is based on a STM32CubeIDE project including the autogenerated HAL. All application code is moved to the Application directory.