Rotary Stewart Platform
-----------------------


In this repository are source files for a Rotary Stewart Platform project. This project is for a small Stewart platform controlled by RC servos and Arduino. Stewart Platform is a parallel robot capable of positioning its moving platform in 6 degrees of freedom. In this project, platform has additional features such as an LCD to show various data and IrDA used for remote controlling of its movements. Main informations about this project and detailed description of a construction process can be found at http://www.instructables.com/id/Arduino-controlled-Rotary-Stewart-Platform/.

Source files are divided into three main folders:

- **src_arduino_code** - contains souce code files and needed libraries for Arduino:
 - for controlling LCD via I2C interface: https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
 -  for IrDA communication: https://github.com/shirriff/Arduino-IRremote/wiki
- **src_comm_lib** - contains communication library for controlling this platform, it is written in C++ and uses serial communication to communicate with arduino. This library works on Windows and on Linux, it is used for easy control of platform movements.
- **templates_for_cutting** - DXF files containing templates for laser cutting of parts from plastic and also PCB layout for board needed for external supply of power to servos. These files were made using QCad tool.

In IrDA communication library there are made tiny changes - because servo and also IrDA library need interrupt handling, there were problems when using servos and IrDA communication at the same time, these changes reduce consequence of these problems, consequences were very subtle random servo movements.
