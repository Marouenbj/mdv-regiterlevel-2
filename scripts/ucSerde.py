import argparse
import json
from pickletools import bytes8
from re import L
from struct import error
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import zmq
import scipy.io as io
import scipy.fft as fft
import struct
import warnings
import seaborn as sns

from collections import deque

sns.set_style('whitegrid')

FIR_LENGTH = 128

class ucSerde(object):

    def __init__(self, port: str, baudrate: int) -> None:

        """ucSerde

        Initializes the serial communication with the
        microcontroller and the Zeromq communication channel
        for asynchronous message passing for live-mode plotting.

        For command-line interface usage: python ucSerde.py -h

        Args:
            port (str): serial interface, '/dev/ttyACMx' for Unix-like systems,
            'COMx' for Windows systems

            baudrate (int): communication speed over the serial channel,
            [bits/s]
        """

        self.port = port
        self.baudrate = baudrate

        self.serialComm = serial.Serial(port=port,
                                        baudrate=baudrate,
                                        timeout=None)

        self._fs = 0

    def daq_config(self,
                   fclk: int,
                   psc: int,
                   arr: int,
                   scount: int,
                   sfilter: int,
                   sdecim: int):
        """Configures the command to be sent to the microcontroller.
        Args:
            fclk (int): FCLK [Hz], (system || timer) clock frequency
            psc (int): TIM2 Prescaler
            arr (int): AutoReloadRegister (CounterPeriod)
            scount (int, optional): Requested number of samples,\
            sfilter (int, optional): Toggle filter on/off. Defaults to 0.
            sdecim (int, optional): Decimate each n-th sample.\
            NOT IMPLEMENTED. Defaults to 0.
        """

        self._fs = ((fclk/(psc+1))/(arr+1))
        self._ts = 1/self._fs

        scount = scount // sdecim

        self.scount = scount
        self.sfilter = sfilter
        self.sdecim = sdecim

        self.command = "ANA {} {} {} {} {}".format(psc,
                                                   arr,
                                                   scount,
                                                   sfilter,
                                                   sdecim)

        print("Sampling frequency [Hz]: {}".format(self._fs))

        print(self.command.encode("ascii", "ignore"))

    def send_command(self):
        self.serialComm.write(self.command.encode())
        self.serialComm.reset_output_buffer()
        self.serialComm.reset_input_buffer()

    def receive_data(self):
        """Transmits the command to the microcontroller, initializing the
        periphery and starting the analog-digital converter. The converted
        signals are transmitted over UART.
        """
        print('Standby')
        tstart = time.time()
        print('Receiving data')
        data = self.serialComm.read_until(b'/n', size=self.scount * 2)
        tend = time.time()
        print("Time elapsed: {}".format(tend-tstart))
        iter_q15 = struct.iter_unpack('<H', data)
        data = np.array(list(iter_q15))
        self.serialComm.close()
        return data

    @property
    def fs(self):
        return self._fs

    @property
    def ts(self):
        return self._ts


def main():
    """Command-line interface for ucSerde, additionally
    plots the acquired data for testing purposes.
    """

    parser = argparse.ArgumentParser(
        description='ucSerde command line utility')
    parser.add_argument('port',
                        action='store',
                        default='/dev/ttyACM0',
                        type=str,
                        help='serial interface, \'/dev/ttyACMx\' for\
                            Unix-like systems, \'COMx\' for Windows systems')
    parser.add_argument('baud',
                        action='store',
                        default=115200,
                        type=int,
                        help='Baudrate [uint32]')
    parser.add_argument('fclk',
                        action='store',
                        default=72000000,
                        type=int,
                        help='FCLK [Hz] [int]')
    parser.add_argument('psc',
                        action='store',
                        default=32-1,
                        type=int,
                        help='TIM2 Prescaler [int]')
    parser.add_argument('arr',
                        action='store',
                        default=1000-1,
                        type=int,
                        help='Counter Period (AutoReloadRegister) [int]')
    parser.add_argument('N',
                        action='store',
                        default=1024,
                        type=int,
                        help='Requested number of samples [uint32]')
    parser.add_argument('toggle_filter',
                        action='store',
                        default=1,
                        type=int,
                        help='Toggle filter on/off [bool]')
    parser.add_argument('dec',
                        action='store',
                        default=0,
                        type=int,
                        help='Decimate each n-th sample [uint8]')
    parser.add_argument('filename',
                        action='store',
                        default='Hz',
                        type=str,
                        help='Decimate each n-th sample [uint8]')
    args = parser.parse_args()

    serde = ucSerde(args.port, args.baud)

    serde.daq_config(fclk=args.fclk,
                     psc=args.psc,
                     arr=args.arr,
                     scount=args.N,
                     sfilter=args.toggle_filter,
                     sdecim=args.dec)

    serde.send_command()
    data = serde.receive_data()

    print(data)

if __name__ == '__main__':
    main()
