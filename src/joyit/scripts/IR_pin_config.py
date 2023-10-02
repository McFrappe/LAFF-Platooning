class IRSensorPinConfig:
    """
    Pin configuration for the IR sensor that has one detector on it.
    The sensor connects to the host with three wires. Connect VCC to 5v, GND to
    GND and OUT to pin 3.
    """
    def __init__(self, data_pin):
        self.pin = data_pin