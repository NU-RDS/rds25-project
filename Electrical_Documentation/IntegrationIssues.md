

Motor Driver
    ??
    Follow on work with debugging

Encoder
    SPI Bus Impedance
    MOSI bridged to ground is wrong
    Highly recommend pre-made connectors, like QWIIC connectors from sparkfun


Daughter Board
    One bridged pin? - CAN
    No more 5V to encoders

Han's PDU
    One flipped CAN wire

CAN needs to be 5V and 12V optional


Future work on SPI bus impedance issue

    Option 1: Use SPI Buffer/Multiplexer Chips
    Use an analog multiplexer (like 74HC4067, 16:1 mux) or a digital buffer like 74LVC125 or TXB0108 on MISO, and directly control which MISO line is routed to the Teensy based on CS lines. Only one encoder's MISO is connected at a time, reducing bus loading. Requires control logic, or a simple GPIO latch to select the MISO path.

    Option 2: Use a Dedicated SPI Bus Per 4 Encoders
    Instantiate 4 SPI buses on each Teensy (i.e., use different SCK/MISO/MOSI/CS pins for each bus). Teensy 4.x supports multiple SPI instances: SPI, SPI1, SPI2 . Keep 4 encoders per SPI bus to avoid electrical issues.

    Option 3: Use SPI Isolators / Repeaters
    Use something like the ADUM3150 (SPI digital isolator with clock delay) or SN74LVC2G66 for analog switches. These reduce bus loading and allow better fanout.


We had to end up using the RDS 24 PDU for odrives... I think for CAN?






