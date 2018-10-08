"""testing_server.py

This script runs a server that echos data sent over TCP connections.
Run it before running tests to act as a testing server.
"""
from __future__ import print_function
import argparse
import socket
import sys


def main(address, port):
    print('starting server bound to {}:{}'.format(address, port))
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((address, port))
    server.listen(3)

    while True:
        conn, addr = server.accept()

        data = conn.recv(100)
        print('received:', repr(data), 'from', addr)
        if data == 'kill':
            sys.exit()

        conn.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Start a testing development server.')
    parser.add_argument('address', type=str, nargs='?', default='127.0.0.1')
    parser.add_argument('port', type=int, nargs='?', default=8081)
    args = parser.parse_args()

    main(args.address, args.port)
