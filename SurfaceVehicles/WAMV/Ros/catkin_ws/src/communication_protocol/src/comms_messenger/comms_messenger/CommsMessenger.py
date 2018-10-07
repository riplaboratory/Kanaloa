"""
comms_messenger.CommsMessenger
"""

class CommsMessenger:
    """Class to send information over TCP to RobotX Technical Director's DHCP server.
    It implements the Communications Protocol specifications found at:
    https://www.robotx.org/images/RobotX-2018-Communications-Protocol_v1.0.pdf
    
    Usage::

    >>> import CommsMessenger
    >>> msgr = CommsMessenger('127.0.0.1', '8080', 'AVUSI')
    >>> msgr.send_heartbeat(21.31198, 'N', 157.88972, 'W', 2, 1)
    """

    def __init__(self, address, port, team_id):
        """
        :param address: str
        :param port: int
        :param team_id: str
        """
        if not isinstance(address, str):
            raise TypeError('address is not of type str')
        if not isinstance(port, int):
            raise TypeError('port is not of type int')
        if not isinstance(team_id, str):
            raise TypeError('team_id is not of type str')

        self._address = address
        self._port = port
        self._team_id = team_id

    def send_heartbeat(self, latitude, north_south_indicator, longitude, east_west_indicator, system_mode, auv_status):
        """Transmit a heartbeat status message over TCP.

        :param latitude: float
        :param north_south_indicator: str
        :param longitude: float 
        :param east_west_indicator: str
        :param system_mode: int
        :param auv_status: int

        Usage::
        >>> msgr_instance.send_heartbeat(21.31198, 'N', 157.88972, 'W', 2, 1)
        """
        pass

    def send_entrance_and_exit(self, active_entrance_gate, active_exit_gate, light_buoy_active, light_pattern):
        """Transmits the gates in which we detect an active beacon over TCP.

        :param active_entrance_gate: int 
        :param active_exit_gate: int 
        :param light_buoy_active: str 
        :param light_pattern: str

        Usage::
        >>> msgr_instance.send_entrance_and_exit(1, 2, 'Y', 'RGB')
        """
        pass

    def send_scan_the_code(self, light_pattern):
        """Transmits a detected light pattern over TCP.

        :param light_pattern: str 

        Usage::
        >>> msgr_instance.send_scan_the_code('RGB')
        """
        pass

    def send_symbols_and_dock(self, shape_color, shape):
        """Transmit data on which bay we plan to dock over TCP.

        :param shape_color: str 
        :param shape: str

        Usage::
        """
        pass
    
    def send_detect_and_deliver(self, shape_color, shape):
        """Transmit data on which hole a payload will be delivered to over TCP.

        :param shape_color: str 
        :param shape: str

        Usage::
        >>> msgr_instance.send_detect_and_deliver('R', 'CIRCL')
        """
        pass

    def change_host(self):
        pass

    @staticmethod
    def _checksum(string):
        """Generate a two digit, string hex number representing a simple checksum for a message.

        Usage::
        >>> CommsMessenger._checksum('RXDEL,101218,161229,AUVSI,R,CIRCL')
        """
        checksum = 0


        for char in string:
            checksum ^= ord(char)
        return checksum

