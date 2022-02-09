A friend found a clock like this: https://www.youtube.com/watch?v=egyOi1eJo0w

I did a thing to drive it from an ESP8266 with NTP time.

Here's a schematic:

![hardware_sketch](hardware_sketch.png =250x)

Notice how the FET can disable the stepper.. The code makes sure ground doesn't find it's way back to the converter, by making sure the two pins connected to the motor driver are set to input (high impedance) first.



