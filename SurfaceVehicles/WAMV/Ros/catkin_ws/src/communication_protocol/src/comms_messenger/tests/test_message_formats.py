import sys
import os
import pytest

sys.path.append(os.getcwd())

from comms_messenger import CommsMessenger as CM


def test_create_instance():
    _msgr = CM('127.0.0.1', 8080, 'AUVSI')
    assert(True)

def test_create_instance_fails():
    with pytest.raises(TypeError, message='Expecting TypeError'):
        _msgr = CM(127001, 8080, 'AUVSI')

def test_heartbeat_msg():
    date, time = CM._get_date_time()
    expected_contents = 'RXHRB,{},{},21.31198,N,157.88972,W,AUVSI,2,1'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)

    msgr = CM('127.0.0.1', 8080, 'AUVSI')
    fields = {
        'latitude': 21.31198,
        'north_south_indicator': 'N',
        'longitude': 157.88972,
        'east_west_indicator': 'W',
        'system_mode': 2,
        'auv_status': 1,
        'date': date,
        'time': time
    }
    message = msgr._heartbeat_msg(**fields)
    assert(message == expected_msg)

def test_entrance_and_exit_msg():
    date, time = CM._get_date_time()
    expected_contents = 'RXGAT,{},{},AUVSI,1,2,Y,RGB'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)

    msgr = CM('127.0.0.1', 8080, 'AUVSI')
    fields = {
        'active_entrance_gate': 1,
        'active_exit_gate': 2,
        'light_buoy_active': 'Y',
        'light_pattern': 'RGB',
        'date': date,
        'time': time
    }
    message = msgr._entrance_and_exit_msg(**fields)
    assert(message == expected_msg)

def test_scan_the_code_msg():
    date, time = CM._get_date_time()
    expected_contents = 'RXCOD,{},{},AUVSI,RGB'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)

    msgr = CM('127.0.0.1', 8080, 'AUVSI')
    fields = {
        'light_pattern': 'RGB',
        'date': date,
        'time': time
    }
    message = msgr._scan_the_code_msg(**fields)
    assert(message == expected_msg)

def test_symbols_and_dock_msg():
    date, time = CM._get_date_time()
    expected_contents = 'RXDOK,{},{},AUVSI,R,TRIAN'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)

    msgr = CM('127.0.0.1', 8080, 'AUVSI')
    fields = {
        'shape_color': 'R',
        'shape': 'TRIAN',
        'date': date,
        'time': time
    }
    message = msgr._symbols_and_dock_msg(**fields)
    assert(message == expected_msg)

def test_detect_and_deliver_msg():
    date, time = CM._get_date_time()
    expected_contents = 'RXDEL,{},{},AUVSI,R,CIRCL'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)

    msgr = CM('127.0.0.1', 8080, 'AUVSI')
    fields = {
        'shape_color': 'R',
        'shape': 'CIRCL',
        'date': date,
        'time': time
    }
    message = msgr._detect_and_deliver_msg(**fields)
    assert(message == expected_msg)