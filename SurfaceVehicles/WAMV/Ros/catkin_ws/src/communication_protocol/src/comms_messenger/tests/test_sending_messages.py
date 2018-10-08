import socket
import sys
import os
import pytest
import time

from functools import partial
from multiprocessing import Process

sys.path.append(os.getcwd())

import testing_server
from comms_messenger import CommsMessenger as CM

def start_server(address, port):
    p = Process(target=partial(testing_server.main, address, port))
    p.start()
    time.sleep(0.01)
    return p
    
def kill_server(address, port):
    time.sleep(0.01)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((address, port))
    s.send('kill')
    s.close()


def test_heartbeat():
    address, port = '127.0.0.1', 8081
    date, time = CM._get_date_time()
    expected_contents = 'RXHRB,{},{},21.31198,N,157.88972,W,AUVSI,2,1'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)
    expected_sent = len(expected_msg)

    server_proc = start_server(address, port)
    msgr = CM(address, port, 'AUVSI')
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
    results = msgr.send_heartbeat(**fields)
    message = results['message']
    sent = results['sent']

    assert(message == expected_msg)
    assert(sent == expected_sent)

    kill_server(address, port)
    server_proc.join()

def test_entrance_and_exit():
    address, port = '127.0.0.1', 8081
    date, time = CM._get_date_time()
    expected_contents = 'RXGAT,{},{},AUVSI,1,2,Y,RGB'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)
    expected_sent = len(expected_msg)

    server_proc = start_server(address, port)
    msgr = CM(address, port, 'AUVSI')
    fields = {
        'active_entrance_gate': 1,
        'active_exit_gate': 2,
        'light_buoy_active': 'Y',
        'light_pattern': 'RGB',
        'date': date,
        'time': time
    }
    results = msgr.send_entrance_and_exit(**fields)
    message = results['message']
    sent = results['sent']

    assert(message == expected_msg)
    assert(sent == expected_sent)

    kill_server(address, port)
    server_proc.join()

def test_scan_the_code():
    address, port = '127.0.0.1', 8081
    date, time = CM._get_date_time()
    expected_contents = 'RXCOD,{},{},AUVSI,RGB'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)
    expected_sent = len(expected_msg)

    server_proc = start_server(address, port)
    msgr = CM(address, port, 'AUVSI')
    fields = {
        'light_pattern': 'RGB',
        'date': date,
        'time': time
    }
    results = msgr.send_scan_the_code(**fields)
    message = results['message']
    sent = results['sent']

    assert(message == expected_msg)
    assert(sent == expected_sent)

    kill_server(address, port)
    server_proc.join()

def test_symbols_and_dock():
    address, port = '127.0.0.1', 8081
    date, time = CM._get_date_time()
    expected_contents = 'RXDOK,{},{},AUVSI,R,TRIAN'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)
    expected_sent = len(expected_msg)

    server_proc = start_server(address, port)
    msgr = CM(address, port, 'AUVSI')
    fields = {
        'shape_color': 'R',
        'shape': 'TRIAN',
        'date': date,
        'time': time
    }
    results = msgr.send_symbols_and_dock(**fields)
    message = results['message']
    sent = results['sent']

    assert(message == expected_msg)
    assert(sent == expected_sent)

    kill_server(address, port)
    server_proc.join()

def test_detect_and_deliver():
    address, port = '127.0.0.1', 8081
    date, time = CM._get_date_time()
    expected_contents = 'RXDEL,{},{},AUVSI,R,CIRCL'.format(date, time)
    expected_checksum = CM._checksum(expected_contents)
    expected_msg = '${}*{}\r\n'.format(expected_contents, expected_checksum)
    expected_sent = len(expected_msg)

    server_proc = start_server(address, port)
    msgr = CM(address, port, 'AUVSI')
    fields = {
        'shape_color': 'R',
        'shape': 'CIRCL',
        'date': date,
        'time': time
    }
    results = msgr.send_detect_and_deliver(**fields)
    message = results['message']
    sent = results['sent']

    assert(message == expected_msg)
    assert(sent == expected_sent)

    kill_server(address, port)
    server_proc.join()