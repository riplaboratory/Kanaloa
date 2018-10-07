import sys
import os
import pytest

sys.path.append(os.getcwd())

import comms_messenger.CommsMessenger as cm

def test_checksum_1():
    assert(cm._checksum('RXDEL,101218,161229,AUVSI,R,CIRCL') == '32')
def test_checksum_2():
    assert(cm._checksum('RXCOD,101218,161229,AUVSI,RBG') == '49')
def test_checksum_3():
    assert(cm._checksum('RXGAT,101218,161229,AUVSI,1,2,Y,RBG') == '25')
    