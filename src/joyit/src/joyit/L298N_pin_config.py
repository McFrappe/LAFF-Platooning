class L298NPinConfig:
    """
    Pin configuration for L298N motor driver.
    """

    def __init__(self, ena, in1, in2, in3, in4, enb):
        """
        Constructor.

        :param ena: GPIO pin number for ENA.
        :param in1: GPIO pin number for IN1.
        :param in2: GPIO pin number for IN2.
        :param in3: GPIO pin number for IN3.
        :param in4: GPIO pin number for IN4.
        :param enb: GPIO pin number for ENB.
        """
        self.ENA = ena
        self.IN1 = in1
        self.IN2 = in2
        self.IN3 = in3
        self.IN4 = in4
        self.ENB = enb