"""
comms_messenger.CommsMessenger
"""

import socket
from datetime import datetime

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

    def send_heartbeat(self, latitude, north_south_indicator, longitude, east_west_indicator, system_mode, auv_status, date=None, time=None):
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
        message = self._heartbeat_msg(latitude, north_south_indicator, longitude, east_west_indicator, system_mode, auv_status, date, time)
        sent = self._send_message(message)
        return {'message': message, 'sent': sent}
    
    def _heartbeat_msg(self, latitude, north_south_indicator, longitude, east_west_indicator, system_mode, auv_status, date, time):
        """Create and return a heartbeat message."""
        MESSAGE_ID = 'RXHRB'

        if date is None or time is None:
            date, time = self._get_date_time()

        contents = '{},{},{},{},{},{},{},{},{},{}'.format(
            MESSAGE_ID,
            date,
            time,
            latitude,
            north_south_indicator,
            longitude,
            east_west_indicator,
            self._team_id,
            system_mode,
            auv_status
        )
        checksum = self._checksum(contents)
        return self._make_message(contents, checksum)
        

    def send_entrance_and_exit(self, active_entrance_gate, active_exit_gate, light_buoy_active, light_pattern, date=None, time=None):
        """Transmits the gates in which we detect an active beacon over TCP.
        :param active_entrance_gate: int 
        :param active_exit_gate: int 
        :param light_buoy_active: str 
        :param light_pattern: str

        Usage::
        >>> msgr_instance.send_entrance_and_exit(1, 2, 'Y', 'RGB')
        """
        message = self._entrance_and_exit_msg(active_entrance_gate, active_exit_gate, light_buoy_active, light_pattern, date, time)
        sent = self._send_message(message)
        return {'message': message, 'sent': sent}

    def _entrance_and_exit_msg(self, active_entrance_gate, active_exit_gate, light_buoy_active, light_pattern, date, time):
        """Create and return an entrance_and_exit message."""
        MESSAGE_ID = 'RXGAT'

        if date is None or time is None:
            date, time = self._get_date_time()

        contents = '{},{},{},{},{},{},{},{}'.format(
            MESSAGE_ID,
            date,
            time,
            self._team_id,
            active_entrance_gate,
            active_exit_gate,
            light_buoy_active,
            light_pattern
        )
        checksum = self._checksum(contents)
        return self._make_message(contents, checksum)


    def send_scan_the_code(self, light_pattern, date=None, time=None):
        """Transmits a detected light pattern over TCP.
        :param light_pattern: str 

        Usage::
        >>> msgr_instance.send_scan_the_code('RGB')
        """
        message = self._scan_the_code_msg(light_pattern, date, time)
        sent = self._send_message(message)
        return {'message': message, 'sent': sent}

    def _scan_the_code_msg(self, light_pattern, date, time):
        """Create and return a scan_the_code message."""
        MESSAGE_ID = 'RXCOD'

        if date is None or time is None:
            date, time = self._get_date_time()

        contents = '{},{},{},{},{}'.format(
            MESSAGE_ID,
            date,
            time,
            self._team_id,
            light_pattern
        )
        checksum = self._checksum(contents)
        return self._make_message(contents, checksum)


    def send_symbols_and_dock(self, shape_color, shape, date=None, time=None):
        """Transmit data on which bay we plan to dock over TCP.
        :param shape_color: str 
        :param shape: str

        Usage::
        """
        message = self._symbols_and_dock_msg(shape_color, shape, date, time)
        sent = self._send_message(message)
        return {'message': message, 'sent': sent}

    def _symbols_and_dock_msg(self, shape_color, shape, date, time):
        """Create and return a symbols_and_dock message."""
        MESSAGE_ID = 'RXDOK'

        if date is None or time is None:
            date, time = self._get_date_time()

        contents = '{},{},{},{},{},{}'.format(
            MESSAGE_ID,
            date,
            time,
            self._team_id,
            shape_color,
            shape
        )
        checksum = self._checksum(contents)
        return self._make_message(contents, checksum)

    
    def send_detect_and_deliver(self, shape_color, shape, date=None, time=None):
        """Transmit data on which hole a payload will be delivered to over TCP.
        :param shape_color: str 
        :param shape: str

        Usage::
        >>> msgr_instance.send_detect_and_deliver('R', 'CIRCL')
        """
        message = self._detect_and_deliver_msg(shape_color, shape, date, time)
        sent = self._send_message(message)
        return {'message': message, 'sent': sent}

    def _detect_and_deliver_msg(self, shape_color, shape, date, time):
        """Create and return a detect_and_deliver message."""
        MESSAGE_ID = 'RXDEL'

        if date is None or time is None:
            date, time = self._get_date_time()

        contents = '{},{},{},{},{},{}'.format(
            MESSAGE_ID,
            date,
            time,
            self._team_id,
            shape_color,
            shape
        )
        checksum = self._checksum(contents)
        return self._make_message(contents, checksum)


    def change_host(self, address, port):
        """Update the address and port of TD server.
        :param address: str
        :param port: int
        """
        if not isinstance(address, str):
            raise TypeError('address is not of type str')
        if not isinstance(port, int):
            raise TypeError('port is not of type int')
        self._address = address
        self._port = port

    def _send_message(self, message):
        """Create a TCP connection and send the message over it. Returns number of bytes sent.
        :param message: str
        """
        totalsent = 0
        message_len = len(message)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self._address, self._port))

        while totalsent < message_len:
            sent = sock.send(message[totalsent:])
            if sent == 0:
                raise RuntimeError('socket connection broken')
            totalsent += sent
        sock.close()

        return totalsent

    @staticmethod
    def _checksum(string):
        """Generate a two digit, string hex number representing a simple checksum for a message.
        :param string: str

        Usage::
        >>> CommsMessenger._checksum('RXDEL,101218,161229,AUVSI,R,CIRCL')
        """
        checksum = 0

        for char in string:
            checksum ^= ord(char)
        
        hex_repr = '{0:02X}'.format(checksum)
        return hex_repr 

    @staticmethod
    def _make_message(contents, checksum):
        """Wrap a message content to create a complete comms protocol compliant message.
        :param contents: str
        :param checksum: str

        Usage::
        >>> msgr_instance._wrap_contents('RXHRB,101218,161229,21.31198,N,157.88972,W,AUVSI,2,1', '06')
        >>> '$RXHRB,101218,161229,21.31198,N,157.88972,W,AUVSI,2,1*06'
        """
        return '${}*{}\r\n'.format(contents, checksum)

    @staticmethod
    def _get_date_time():
        """Returns the date and time in current timezone as 'ddmmyy', 'hhmmss' 24 hour formats.
        """
        dt = datetime.now()
        return (dt.strftime('%d%m%y'), dt.strftime('%H%M%S'))