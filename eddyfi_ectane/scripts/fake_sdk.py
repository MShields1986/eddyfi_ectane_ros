#!/usr/bin/env python3
"""Simulated C SDK, for running the driver without an Ectane.

Stands in for hostdemo.c from eddyfi_ectane_sdk_ros: connects to the driver's command server on TCP 24010, and while
it is commanded to acquire data, streams a data cycle to UDP 24011 at a fixed rate.

The stream contains the channels enabled in Configurations.h for the configuration, ordered by time slot, input, then
frequency, as the C SDK sends them. The values are not eddy current signals: each word holds its position in the
stream, real = position + 1 and imaginary = 1000 + position, the same every cycle, so the points in each cloud can be
traced back to the channel they came from.

The command strings the driver sends are printed, or written to --log.
"""
import argparse
import os
import re
import socket
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_STRUCTURES_H = os.path.join(HERE, '..', 'include', 'c_sdk_data_structures.h')
# Where eddyfi_ectane_sdk_ros is when cloned alongside this repository
DEFAULT_CONFIGURATIONS_H = os.path.join(HERE, '..', '..', '..', 'eddyfi_ectane_sdk_ros', 'src', 'Configurations.h')

COMMAND_PORT = 24010
DATA_PORT = 24011
DATA_WORDS = 32 * 8 * 5   # Time slots * inputs * frequencies, as in the C SDK's data buffer

ACQUIRE_DATA = 5
STOP_DATA = 6
DISCONNECT_DEVICE = 7


def configuration_names():
    """The Configurations enum, whose order gives each configuration's case in Configurations.h."""
    enum = re.search(r'enum\s+Configurations\s*\{(.*?)\}', open(DATA_STRUCTURES_H).read(), re.S).group(1)
    return [name.strip() for name in re.sub(r'//.*', '', enum).split(',') if name.strip()]


def enabled_channels(configurations_h, case):
    """(time slot, input, frequency) of each enabled channel for the case, zero based, in stream order."""
    body = re.split(r'\n\s*case (\d+):', open(configurations_h).read())
    cases = {int(body[i]): body[i + 1] for i in range(1, len(body), 2)}
    if case not in cases:
        sys.exit('No case %d in %s' % (case, configurations_h))
    found = re.findall(r'astInput\[(\d+)\]\.abFreqChannelEnabled\[(\d+)\]\[(\d+)\]\s*=\s*TRUE', cases[case])
    return sorted({(int(ts), int(inp), int(fr)) for inp, ts, fr in found})


def main():
    names = configuration_names()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('configuration', choices=names, metavar='configuration',
                        help='the configuration the driver is launched with: ' + ', '.join(names))
    parser.add_argument('--configurations-h', default=DEFAULT_CONFIGURATIONS_H,
                        help="the C SDK's Configurations.h (default: %(default)s)")
    parser.add_argument('--rate', type=float, default=50.0, help='data cycles per second (default: %(default)s)')
    parser.add_argument('--log', help='write the stream layout and commands here instead of printing them')
    args = parser.parse_args()

    channels = enabled_channels(os.path.normpath(args.configurations_h), names.index(args.configuration))
    words = [((1000 + pos) << 16) | (pos + 1) for pos in range(len(channels))]
    payload = struct.pack('<%di' % DATA_WORDS, *(words + [0] * (DATA_WORDS - len(words))))

    log = open(args.log, 'w', buffering=1) if args.log else sys.stdout
    def write(line):
        log.write(line + '\n')
        log.flush()

    # Time slot, input and frequency, one based
    write('stream %s' % ';'.join('%d,%d,%d' % (t + 1, i + 1, f + 1) for t, i, f in channels))

    streaming = threading.Event()

    def stream():
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while True:
            streaming.wait()
            udp.sendto(payload, ('127.0.0.1', DATA_PORT))
            time.sleep(1.0 / args.rate)

    threading.Thread(target=stream, daemon=True).start()

    # The driver's command server may not be up yet
    while True:
        try:
            tcp = socket.create_connection(('127.0.0.1', COMMAND_PORT))
            break
        except OSError:
            time.sleep(0.2)

    while True:
        data = tcp.recv(2000)
        if not data:
            break
        command = data.decode(errors='replace')
        write('command %s' % command)
        state = int(command.split(',')[0])
        if state == ACQUIRE_DATA:
            streaming.set()
        elif state in (STOP_DATA, DISCONNECT_DEVICE):
            streaming.clear()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
