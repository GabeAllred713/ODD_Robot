from typing import Tuple, List

from serial import Serial


class ExampleSerial:
    def __init__(self, port: str) -> None:
        self.serial: Serial = Serial(port, 115200, timeout=0.1)
        self._motor_commands: List[int] = [0, 0]
        self._last_command: str = "0,0\n"
        self._encoders: List[float] = [0, 0]

    def __del__(self) -> None:
        self.close()

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def encoders(self) -> Tuple[float, float]:
        self.serial.write(self._last_command.encode("ascii"))
        self._read_serial()
        return self._encoders[0], self._encoders[1]

    def motors(self, motor1: int, motor2: int) -> None:
        self._motor_commands[0] = motor1
        self._motor_commands[1] = motor2
        self._write_serial()

    def _read_serial(self) -> None:
        data_string: str = self.serial.readline().decode("ascii")
        if len(data_string) == 0:
            print("No serial data received.")
            return

        data_split: List[str] = data_string.split(",")

        try:
            self._encoders = [float(data_split[0]), float(data_split[1])]
        except ValueError as e:
            print(f"Could not read '{data_string}'")

    def _write_serial(self) -> None:
        self._last_command = f"{self._motor_commands[0]},{self._motor_commands[1]}\n"
        self.serial.write(self._last_command.encode("ASCII"))
        self._read_serial()
