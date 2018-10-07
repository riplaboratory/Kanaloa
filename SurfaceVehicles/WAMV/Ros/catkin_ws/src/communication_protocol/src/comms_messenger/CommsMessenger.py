"""
"""

class CommsMessenger:
    """Class to send information over TCP to RobotX Technical Director's DHCP server.
    
    Usage::

    >>> import CommsMessenger
    >>> msgr = CommsMessenger('127.0.0.1', '8080', 'AVUSI')
    >>> msgr.send_heartbeat(21.31198, 'N', 157.88972, 'W', 2, 1)
    """
    def __init__(self, address, port, team_id):
        self._address = address
        self._port = port
        self._team_id = team_id

    def send_heartbeat(
        self, 
        latitude: float, 
        north_south_indicator: str, 
        longitude: float, 
        east_west_indicator: str, 
        system_mode: int, 
        auv_status: int):
        pass

    def send_entrance_and_exit(self, active_entrance_gate: int, active_exit_gate: int, light_buoy_active: str, light_pattern: str):
        pass

    def send_scan_the_code(self, light_pattern: str):
        pass

    def send_symbols_and_dock(self, shape_color: str, shape: str):
        pass
    
    def send_detect_and_deliver(self, shape_color: str, shape: str):
        pass

    def change_host(self):
        pass

    @staticmethod
    def _checksum(string):
        checksum = 0

        for char in string:
            checksum ^= ord(char)
        
        hex = "{0:02X}".format(checksum)
        
        return hex

