#!/usr/bin/env python3
"""Checks the driver's output against the stream fake_sdk.py sends, which it writes to its log.

Starts streaming, then checks that every channel of an observation reads the stream word for its time slot, input and
frequency, and that every point cloud holds just the channels of its frequency and coil orientation.

usage: check_pipeline.py <fake sdk log> <number of frequencies> <settings yaml> [stream-first]
"""
import re
import struct
import sys
import time

import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from eddyfi_ectane.msg import Observation
from eddyfi_ectane.srv import StreamData

LOG, NUM_FREQS, SETTINGS = sys.argv[1], int(sys.argv[2]), sys.argv[3]
STREAM_FIRST = len(sys.argv) > 4 and sys.argv[4] == 'stream-first'   # Start streaming before the reconstruction

settings = yaml.safe_load(open(SETTINGS))['/**']['ros__parameters']
FREQS, VOLTS = settings['frequencies'], settings['voltages']

stream = None
for line in open(LOG):
    if line.startswith('stream '):
        stream = [tuple(int(v) for v in item.split(',')) for item in line.split()[1].split(';')]
position_of = {key: pos for pos, key in enumerate(stream)}

failures = []


def check(condition, message):
    if not condition:
        failures.append(message)


def parse_name(name):
    m = re.match(r'^F(\d+)(.+)$', name)
    return (m.group(2), int(m.group(1))) if m else (name, 1)


def points(cloud):
    return [struct.unpack_from('<fffff', cloud.data, i * cloud.point_step) for i in range(cloud.width)]


rclpy.init()
node = rclpy.create_node('check_pipeline')
data_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
recon_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
prefix = lambda f: '/ectane/' + ('frequency_%d/' % f if NUM_FREQS > 1 else '')

latest = {}
def keep(topic):
    return lambda msg: latest.__setitem__(topic, msg)

topics = ['/ectane/eddy_current_data']
node.create_subscription(Observation, '/ectane/eddy_current_data', keep('/ectane/eddy_current_data'), data_qos)
for f in range(1, NUM_FREQS + 1):
    for kind in ('axial', 'transversal'):
        at_probe = prefix(f) + kind + '_data_at_probe'
        recon = prefix(f) + 'distributed_' + kind + '_reconstruction'
        topics += [at_probe, recon]
        node.create_subscription(PointCloud2, at_probe, keep(at_probe), data_qos)
        node.create_subscription(PointCloud2, recon, keep(recon), recon_qos)

client = node.create_client(StreamData, '/ectane/stream_data')
deadline = time.time() + 60
while not client.wait_for_service(timeout_sec=0.5):
    if time.time() > deadline:
        print('FAILED: no /ectane/stream_data service')
        sys.exit(1)


def spin_for(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def stream_data(value):
    future = client.call_async(StreamData.Request(stream_data=value))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    return future.result() is not None and future.result().success


# Start streaming only once the reconstruction is up and connected, as on the robot
recon_topics = [t for t in topics if 'reconstruction' in t]
while not STREAM_FIRST and not all(node.count_publishers(t) > 0 for t in recon_topics):
    spin_for(0.2)
spin_for(0 if STREAM_FIRST else 3)

check(stream_data(True), 'stream_data(True) did not succeed')
print('streaming started')
sys.stdout.flush()

def ready():
    if not all(t in latest for t in topics):
        return False
    for t in recon_topics:
        at_probe = re.sub(r'distributed_(axial|transversal)_reconstruction$', r'\1_data_at_probe', t)
        if latest[at_probe].width > 0 and latest[t].width == 0:
            return False
    return True


deadline = time.time() + (60 if STREAM_FIRST else 20)
while time.time() < deadline and not ready():
    spin_for(0.2)
spin_for(3)

missing = [t for t in topics if t not in latest]
check(not missing, 'no messages on %s' % missing)

# Observation: every channel reads the stream word for its (time slot, input, frequency)
obs = latest.get('/ectane/eddy_current_data')
name_at = {}
if obs:
    check(len(obs.channels) == len(stream), 'observation has %d channels, SDK streams %d' % (len(obs.channels), len(stream)))
    for ch in obs.channels:
        coil, f = parse_name(ch.name)
        expected = position_of.get((ch.time_slot, ch.input, f))
        got = round(ch.imaginary_reading) - 1
        check(expected is not None, '%s (ts %d, input %d, f %d) not streamed by SDK' % (ch.name, ch.time_slot, ch.input, f))
        check(got == expected and round(-ch.real_reading) - 1000 == expected,
              '%s reads stream word %s, expected %s' % (ch.name, got, expected))
        check(ch.frequency == FREQS[f - 1] and abs(ch.dc_voltage - VOLTS[f - 1]) < 1e-6,
              '%s has frequency %d / voltage %s' % (ch.name, ch.frequency, ch.dc_voltage))
        name_at[got] = ch.name
    print('observation: %d channels checked' % len(obs.channels))

# Clouds: every point belongs to the cloud's frequency and orientation
for f in range(1, NUM_FREQS + 1):
    for kind in ('axial', 'transversal'):
        expected_names = {n for n in name_at.values() if parse_name(n)[1] == f and parse_name(n)[0][0] == kind[0].upper()}
        for t in (prefix(f) + kind + '_data_at_probe', prefix(f) + 'distributed_' + kind + '_reconstruction'):
            cloud = latest.get(t)
            if cloud is None:
                continue
            names = [name_at.get(round(p[3]) - 1) for p in points(cloud)]
            wrong = sorted({str(n) for n in names if n not in expected_names})
            check(not wrong, '%s contains %s' % (t, wrong[:5]))
            if t.endswith('_at_probe'):
                check(sorted(names) == sorted(expected_names), '%s has %d points, expected %d' % (t, len(names), len(expected_names)))
            elif expected_names:
                check(cloud.width > 0, '%s is empty' % t)
            types = {field.name: field.datatype for field in cloud.fields}
            check(types.get('Imaginary') == 7 and types.get('Real') == 7,
                  '%s Imaginary/Real datatypes are %s, expected FLOAT32 (7)' % (t, (types.get('Imaginary'), types.get('Real'))))
            print('%-55s %5d points (%d coils)' % (t, cloud.width, len(expected_names)))

check(stream_data(False), 'stream_data(False) did not succeed')

if failures:
    print('FAILED (%d)' % len(failures))
    for m in failures[:30]:
        print('  ' + m)
    sys.exit(1)
print('PASSED')
