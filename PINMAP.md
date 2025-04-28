Summary of Pin Assignments:
    CAN Interface 1 (using PIO): GP0 (RX), GP1 (TX)
    CAN Interface 2 (using PIO): GP2 (RX), GP3 (TX)
    Steering Angle Sensor (AS5048A - SPI):
        GP18 (SPI0 SCK)
        GP19 (SPI0 TX/MOSI)
        GP16 (SPI0 RX/MISO)
        GP17 (SPI0 CSn)
    Signal Light Detection (Digital Input - Low/High):
        GP4 (Signal Light 1) - Pull Down
        GP5 (Signal Light 2) - Pull Down
        Ground Connection for middle pin of vehicle connector
    Ignition Detection (Digital Input - Low/High): GP6
    Resistor Ladder Button Array (Analog Input): GP26 (ADC0)
    MOSFET Control (Digital Output): GP7
        Ability to turn off power to AS5048A?