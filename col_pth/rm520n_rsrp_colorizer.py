from colorizer import Colorizer
import serial

class Rm520nRsrpColorizer(Colorizer):
    def __init__(self, serial_port='/dev/ttyUSB2'):
        self.serial_port = serial_port
        self.current_color = None
        self.current_value = None

        self.rm520_serial = serial.Serial(
            port=self.serial_port,  # Replace with the actual serial port
            baudrate=115200,
            timeout=1
        )

    def send_at_command(self, command, check):
        self.rm520_serial.write(command.encode() + b'\r')
        response = ''
        import time
        time.sleep(0.1)
        cnt = 0
        # read until expected output line is found (maximum 5 lines)
        while not response.startswith(check):
            response = self.rm520_serial.readline().decode().strip()
            cnt += 1
            if cnt > 5:
                return ''
        return response

    def get_rsrp(self):
        command = 'AT+QENG="servingcell"'
        response = self.send_at_command(command, '+QENG:')
        if response.startswith('+QENG:'):
            try:
                values = response.split(':')[1].split(',')
                rsrp = int(values[12].split()[0])

                if rsrp > -90:
                    return 'excellent'
                elif rsrp > -105:
                    return 'good'
                else:
                    return 'poor'
            except:
                return 'disconnected'
        else:
            return None

    def get_color_from_strength(self, strength):
        color_map = {
            'poor': (1.0, 0.0, 0.0),            # Red
            'good': (1.0, 1.0, 0.0),            # Yellow
            'excellent': (0.0, 1.0, 0.0),       # Green
            'disconnected': (0.0, 0.0, 1.0),    # Blue
        }

        return color_map.get(strength.lower(), (0.5, 0.5, 0.5))  # Default to grey

    def query_current_color(self):
        self.current_value = self.get_rsrp()
        self.get_color_from_strength(self.current_value)
        return self.get_color()

    def get_color(self):
        return self.current_color

    def get_value(self):
        return self.current_value
    
    def is_same_as_current_color(self, prev_color):
        if not self.current_color:
            return False
        
        return ( (prev_color[0] == self.current_color[0]) and 
                 (prev_color[1] == self.current_color[1]) and 
                 (prev_color[2] == self.current_color[2]) )

