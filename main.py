import ubluetooth
import time
from machine import Pin

#
# --------------------------
# New bit-banged MAX6675 driver
# --------------------------
#
class MAX6675:
    MEASUREMENT_PERIOD_MS = 220

    def __init__(self, sck, cs, so):
        """
        Creates new object for controlling MAX6675 (bit-banged).
        :param sck: SCK (clock) pin, must be configured as Pin.OUT
        :param cs: CS (select) pin, must be configured as Pin.OUT
        :param so: SO (data) pin, must be configured as Pin.IN
        """
        # Thermocouple
        self._sck = sck
        self._sck.low()

        self._cs = cs
        self._cs.high()

        self._so = so
        self._so.low()

        self._last_measurement_start = 0
        self._last_read_temp = 0
        self._error = 0

    def _cycle_sck(self):
        self._sck.high()
        time.sleep_us(1)
        self._sck.low()
        time.sleep_us(1)

    def refresh(self):
        """
        Start a new measurement.
        """
        self._cs.low()
        time.sleep_us(10)
        self._cs.high()
        self._last_measurement_start = time.ticks_ms()

    def ready(self):
        """
        Signals if measurement is finished.
        :return: True if measurement is ready for reading.
        """
        return time.ticks_ms() - self._last_measurement_start > MAX6675.MEASUREMENT_PERIOD_MS

    def error(self):
        """
        Returns the error bit of the last reading. If this bit is set (=1),
        there's a problem with the thermocouple (damaged or loosely connected).
        :return: Error bit value
        """
        return self._error

    def read(self):
        """
        Reads last measurement and starts a new one. If new measurement is not ready yet,
        returns the last value. The last measurement could be old if you never call `refresh`.
        To refresh measurement, call `refresh` and wait for `ready() == True`.
        :return: Measured temperature (in Celsius)
        """
        # Check if new reading is available
        if self.ready():
            # Bring CS pin low to read the result
            self._cs.low()
            time.sleep_us(10)

            # Read bits [14..3] for temperature from MAX6675 (12 bits total)
            value = 0
            for i in range(12):
                self._cycle_sck()
                value |= self._so.value() << (11 - i)

            # The next bit is the error bit (open-circuit check)
            self._cycle_sck()
            self._error = self._so.value()

            # Discard the last two bits
            for _ in range(2):
                self._cycle_sck()

            # End reading, raise CS
            self._cs.high()
            self._last_measurement_start = time.ticks_ms()

            # Each LSB = 0.25 °C
            self._last_read_temp = value * 0.25

        return self._last_read_temp


#
# --------------------------
# BLE Peripheral Class
# --------------------------
#
class BLEPeripheral:
    def __init__(self, name="RBPThermocouple"):
        # Initialize BLE
        self.ble = ubluetooth.BLE()
        self.ble.active(True)

        self._connections = set()
        # Force notifications always on, as in your working code
        self._notification_enabled = True

        # Default notification interval (2 seconds)
        self._notify_frequency = 2000

        # RBP 128-bit UUIDs
        self.SERVICE_UUID_128 = "4ac90000-0b71-11e8-b8f5-b827ebe1d493"
        self.CHAR_UUID_128    = "4ac90001-0b71-11e8-b8f5-b827ebe1d493"
        self.NOTIFY_FREQ_UUID_128 = "4ac90002-0b71-11e8-b8f5-b827ebe1d493"
        self.PROBE_TYPE_UUID_128   = "4ac90003-0b71-11e8-b8f5-b827ebe1d493"

        # Setup GATT services: RBP + DIS
        self._setup_gatt_services()

        # Build and start advertising
        self._payload = self._advertise_payload(self.SERVICE_UUID_128)
        self._scan_resp = self._scan_response_payload(name)
        self.ble.gap_advertise(100, adv_data=self._payload, resp_data=self._scan_resp)

        print("Advertising started")
        print(f"Advertising payload: {self._payload.hex()}")
        print(f"Scan response payload: {self._scan_resp.hex()}")

    def _setup_gatt_services(self):
        """
        Registers:
          - RBP service (128-bit)
          - Device Information Service (DIS) for a Serial Number
        """
        SERVICE_UUID      = ubluetooth.UUID(self.SERVICE_UUID_128)
        CHAR_UUID         = ubluetooth.UUID(self.CHAR_UUID_128)
        NOTIFY_FREQ_UUID  = ubluetooth.UUID(self.NOTIFY_FREQ_UUID_128)
        PROBE_TYPE_UUID   = ubluetooth.UUID(self.PROBE_TYPE_UUID_128)

        self._probe_type_value = b"\x01"  # e.g. Type K
        self._notify_freq_value = (self._notify_frequency).to_bytes(2, 'little')

        rbp_characteristics = [
            (CHAR_UUID,        ubluetooth.FLAG_NOTIFY | ubluetooth.FLAG_READ),
            (NOTIFY_FREQ_UUID, ubluetooth.FLAG_READ   | ubluetooth.FLAG_WRITE),
            (PROBE_TYPE_UUID,  ubluetooth.FLAG_READ   | ubluetooth.FLAG_WRITE),
        ]
        rbp_services = (SERVICE_UUID, rbp_characteristics)

        # DIS service (UUID 0x180A), with Serial Number (UUID 0x2A25)
        DIS_UUID = ubluetooth.UUID(0x180A)
        SERIAL_NUMBER_UUID = ubluetooth.UUID(0x2A25)

        dis_characteristics = [
            (SERIAL_NUMBER_UUID, ubluetooth.FLAG_READ),
        ]
        dis_services = (DIS_UUID, dis_characteristics)

        try:
            handles = self.ble.gatts_register_services([rbp_services, dis_services])

            # RBP has 3 characteristics => handles[0]
            self._char_handle         = handles[0][0]  # Temperature notify
            self._notify_freq_handle  = handles[0][1]  # Notify freq
            self._probe_type_handle   = handles[0][2]  # Probe type

            # DIS has 1 characteristic => handles[1]
            self._dis_serial_handle   = handles[1][0]  # Serial Number

            print("Services registered:", handles)
        except Exception as e:
            print("Error registering services:", e)
            raise

        # Write a unique serial number to the DIS Serial Number String
        self.ble.gatts_write(self._dis_serial_handle, b"SN12345")

        # Register IRQ
        self.ble.irq(self._irq)

    def _advertise_payload(self, service_uuid_128):
        payload = bytearray()

        # 1) Flags
        payload.extend(bytearray([2, 0x01, 0x06]))

        # 2) Appearance
        payload.extend(bytearray([3, 0x19, 0x00, 0x00]))

        # 3) Complete List of 128-bit Service UUIDs
        uuid_bytes = bytes.fromhex(service_uuid_128.replace("-", ""))
        payload.extend(bytearray([17, 0x07]) + bytes(reversed(uuid_bytes)))

        # 4) Manufacturer data (0x0590 for Rainfrog)
        #    plus the 2-byte notification interval
        mf_data = bytearray([0x90, 0x05])
        mf_data.extend(self._notify_frequency.to_bytes(2, 'little'))
        ad_structure = bytearray([len(mf_data) + 1, 0xFF]) + mf_data
        payload.extend(ad_structure)

        print("Final adv payload length =", len(payload))
        return bytes(payload)

    def _scan_response_payload(self, name):
        """
        Builds the scan response payload with the complete local name.
        """
        payload = bytearray()
        payload.extend(bytearray([len(name) + 1, 0x09]))
        payload.extend(name.encode("utf-8"))
        return bytes(payload)

    def _irq(self, event, data):
        """
        BLE events:
         1 => central connected
         2 => central disconnected
         3 => write request (characteristic/CCCD)
        """
        if event == 1:  # Central connected
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print(f"Central connected: {conn_handle}")

            # Refresh characteristic values
            self.ble.gatts_write(self._probe_type_handle, self._probe_type_value)
            self.ble.gatts_write(self._notify_freq_handle, self._notify_freq_value)
            print("Probe type and notify frequency updated on connect")
            time.sleep(2)

        elif event == 2:  # Central disconnected
            conn_handle, _, _ = data
            self._connections.discard(conn_handle)
            self._notification_enabled = False
            print(f"Central disconnected: {conn_handle}")

        elif event == 3:  # Write to characteristic or CCCD
            conn_handle, attr_handle = data

            if attr_handle == self._char_handle + 1:  # CCCD for Temperature
                cccd_value = self.ble.gatts_read(attr_handle)
                print(f"CCCD written: {cccd_value.hex()}")
                self._notification_enabled = (cccd_value == b"\x01\x00")
                print(f"Notifications {'enabled' if self._notification_enabled else 'disabled'}")

            elif attr_handle == self._notify_freq_handle:
                new_value = self.ble.gatts_read(attr_handle)
                if len(new_value) == 2:
                    self._notify_frequency = int.from_bytes(new_value, 'little')
                    self._notify_freq_value = new_value
                    print(f"New notify frequency: {self._notify_frequency} ms")
                else:
                    print("Invalid notify frequency data length")

            elif attr_handle == self._probe_type_handle:
                new_probe_type = self.ble.gatts_read(attr_handle)
                self._probe_type_value = new_probe_type
                print(f"New probe type value: {new_probe_type.hex()}")

            else:
                print(f"Unknown write to handle: {attr_handle}")

    def send(self, temperature):
        """
        Sends temperature as a 4-byte (Int32) value in hundredths of a degree.
        E.g. 225.56 => 22556 => 0x000057DC (two's complement).
        """
        if not self._connections:
            print("No connections to notify")
            return

        if not self._notification_enabled:
            print("Notifications are not enabled by the central")
            return

        temp_int = int(temperature * 100)
        temp_int_32 = temp_int & 0xFFFFFFFF
        temp_bytes = temp_int_32.to_bytes(4, "little")

        for conn_handle in self._connections:
            try:
                self.ble.gatts_notify(conn_handle, self._char_handle, temp_bytes)
                print(f"Notification sent: raw={temp_bytes.hex()} int={temp_int}")
            except Exception as e:
                print(f"Error sending notification: {e}")


#
# --------------------------
# Main Program
# --------------------------
#
if __name__ == "__main__":
    # Instantiate the BLE device
    ble = BLEPeripheral(name="RBPThermocouple")

    # Instantiate the bit-banged MAX6675 (pins may differ on your hardware)
    sck = Pin(2, Pin.OUT)
    cs = Pin(3, Pin.OUT)
    so = Pin(4, Pin.IN)

    max_sens = MAX6675(sck, cs, so)

    while True:
        #
        # 1) Trigger a fresh conversion (refresh)
        #    Then wait until it's ready. The default MEASUREMENT_PERIOD_MS = 220 ms
        #
        max_sens.refresh()

        # Wait until the reading is finished
        while not max_sens.ready():
            time.sleep_ms(10)  # small sleep to avoid busy looping

        #
        # 2) Read the temperature (this also starts the next measurement inside read())
        #
        temp_c = max_sens.read()

        # If error bit is set, it usually means an open or disconnected thermocouple
        if max_sens.error():
            print("Thermocouple open or error.")
        else:
            print(f"Current temperature: {temp_c:.2f} °C")
            # Notify via BLE
            ble.send(temp_c)

        time.sleep(2)
