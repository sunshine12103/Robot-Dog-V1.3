# Sport Micro

### UART

This is a definition of the types of packets that are sent out of the robot's UART.

All packets start with non-printable ASCII characters STX ``\x02`` and end with ETX, carriage return and line feed ``\x03\r\n``.

Values inside packets are delimited by tab characters `\t`. The **first** value indicates the packet type which are definied below. The **final** value is the time that the last loop finished at in microseconds.

#### RC Command Packet

RC Command packets have a code of `2`. Each value indicates the value of one of the RC channels.

    2	980	980	980	3	4	5	500	499	8	9	10	11	12	13	14	15	16	17	18	123456

#### Servo Command Packet

Servo Command packets have a code of `3`. The first 12 values indicates the value to be written to one of the 12 servos. The next value indicates the current state machine state. The next value indicates how many position targets are queued up.

    3	1	2	3	4	5	6	7	8	9	10	11	12	123456

### Simulator Virtual Environment Setup

This project features a software and hardware in the loop simulators. First set up the simulation environment. Python 3.7.8 32 bit was used for development.

    cd spot_sim
    pipenv install --deploy
    pipenv run python spot_sim.py

#### Software Simulation With RC Input Via GUI

In this mode, the RC Simulator GUI application writes RC packets to `spot_sim_data.txt`. These packets are consumed by the simulation environment to control the robot. This is the default mode of operation so no additional setup or hardware is required.

#### Software Simulation With RC Input Via Transmitter

In this mode of operation, a physical RC transmitter feeds RC commands to the RC receiver on the robot. This is accomplished fairly simply since the robot writes RC packets out to a UART. To use this mode of operation...
1. Close the RC Simulator GUI application since it will not be responsible for updating `spot_sim_data.txt`.
1. Write the content that the robot is sending out of its UART to `spot_sim_data.txt`. This may be done with a third party serial terminal software like TeraTerm via its logging feature.

### Hardware Simulation

In this mode of operation, a physical RC transmitter feeds RC commands to the RC receiver on the robot and the robot decides how to actuate the servos. This is accomplished fairly simply since the robot writes servo commands packets out to a UART. To use this mode of operation...
1. Close the RC Simulator GUI application since it will not be responsible for updating `spot_sim_data.txt`.
1. Write the content that the robot is sending out of its UART to `spot_sim_data.txt`. This may be done with a third party serial terminal software like TeraTerm via its logging feature.
