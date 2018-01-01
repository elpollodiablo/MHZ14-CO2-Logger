import serial, six

defaultPort = '/dev/ttyUSB0'

def calculate_checksum(seq):
    s = sum(seq[1:7])
    i = ((0xffff-s) + 1)
    seq[8] = i & 0xff 
    return seq

class MHZ14Reader:
    """
    Simple sensor communication class.
    """

    _requestSequence = [0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]
    _zeroSequence = [0xff, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78]
    # set to 400ppm default
    _spanSequence = calculate_checksum([0xff, 0x01, 0x88, 0x01, 0x90, 0x00, 0x00, 0x00, 0x00])
    # this could work for mh z19
    _baselineSequence = calculate_checksum([0xff, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    """
    https://www.google.com/#q=MH-Z14+datasheet+pdf
    """

    def __init__(self, port, open_connection=True):
        """
        :param string port: path to tty
        :param bool open_connection: should port be opened immediately
        """
        self.port = port
        """TTY name"""
        self.link = None
        """Connection with sensor"""
        if open_connection:
            self.connect()

    def connect(self):
        """
        Open tty connection to sensor
        """
        if self.link is not None:
            self.disconnect()
        self.link = serial.Serial(self.port, 9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE, dsrdtr=True, timeout=5, interCharTimeout=0.1)

    def disconnect(self):
        """
        Terminate sensor connection
        """
        if self.link:
            self.link.close()

    def _write_request(self, sequence):
        """
        Write request string to serial
        """
        for byte in sequence:
            self.link.write(six.int2byte(byte))

    def calibrate_zero(self):
        """
        Send zero point calibration command
        """
        self._write_request(self._zeroSequence)

    def calibrate_400ppm(self):
        """
        Send 400ppm span calibration command
        """
        self._write_request(self._spanSequence)

    def calibrate_baseline(self):
        """
        Send Automatic Baseline Calibration command
        """
        self._write_request(self._baselineSequence)

    def get_status(self):
        """
        Read data from sensor
        :return {ppa, t}|None:
        """
        self._write_request(self._requestSequence)
        response = bytearray(self.link.read(9))
        if len(response) == 9:
            return {"ppa": response[2] * 0xff + response[3], "t": response[4] - 40}
        return None


if __name__ == "__main__":
    import time
    import sys
    import argparse

    parser = argparse.ArgumentParser(description='Read data from MH-Z14 CO2 sensor.')
    parser.add_argument('tty', default=defaultPort, help='tty port to connect', type=str, nargs='?')
    parser.add_argument('--interval', default=10, help='interval between sensor reads, in seconds', type=int, nargs='?')
    parser.add_argument('--mqtt-hostname', default="127.0.0.1", help='mqtt hostname', type=str, nargs='?')
    parser.add_argument('--mqtt-port', default=1883, help='mqtt port', type=int, nargs='?')
    parser.add_argument('--mqtt-username', help='mqtt username', type=str, nargs='?')
    parser.add_argument('--mqtt-password', help='mqtt password', type=str, nargs='?')
    parser.add_argument('--mqtt-topic_prefix', default='tele/mhz14', help='mqtt topic', type=str, nargs='?')
    parser.add_argument('--mqtt-client_id', default='co2reader', help='mqtt client id', type=str, nargs='?')
    parser.add_argument('--mqtt-keepalive', default=60, help='mqtt keepalive time', type=int, nargs='?')
    parser.add_argument('--single', action='store_true', help='single measure mode')
    parser.add_argument('--mqtt', action='store_true', help='mqtt publish mode')
    parser.add_argument('--silent', action='store_true', help='no terminal output')
    parser.add_argument('--calibrate-zero', action='store_true', help='zero point calibration')
    parser.add_argument('--calibrate-400ppm', action='store_true', help='400ppm span calibration')
    parser.add_argument('--calibrate-baseline', action='store_true', help='toggle baseline calibration (mh-z19)')

    args = parser.parse_args()
    port = args.tty

    conn = MHZ14Reader(port, open_connection=True)

    if args.calibrate_zero:
        conn.calibrate_zero()
        sys.exit(0)
    if args.calibrate_400ppm:
        conn.calibrate_400ppm()
        sys.exit(0)
    if args.calibrate_baseline:
        conn.calibrate_baseline()
        sys.exit(0)

    if args.mqtt:
        try:
            import paho.mqtt.client as mqttc
        except:
            print("No paho mqtt library found. Execute `pip install paho-mqtt` to fix this.")
            sys.exit(1)
        mqtt_client = mqttc.Client(client_id = args.mqtt_client_id, clean_session=True)
        if args.mqtt_username:
            mqtt_client.username_pw_set(args.mqtt_username, args.mqtt_password)
        mqtt_client.connect(args.mqtt_hostname, args.mqtt_port, args.mqtt_keepalive)
        mqtt_client.loop_start()

    while True:
        status = conn.get_status()
        if status:
            if not args.silent:
                print("{}\t{}\t{}".format(time.strftime("%Y-%m-%d %H:%M:%S"), status["ppa"], status["t"]))
            if args.mqtt:
                mqtt_client.publish("{}/PPM".format(args.mqtt_topic_prefix), status["ppa"])
                mqtt_client.publish("{}/TEMP".format(args.mqtt_topic_prefix), status["t"])
        else:
            if not args.silent:
                print("No data received")
            sys.exit(1)
        sys.stdout.flush()
        if not args.single and args.interval > 0:
            time.sleep(args.interval)
        else:
            break
    conn.disconnect()
