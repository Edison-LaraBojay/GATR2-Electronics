"""capture_fixture.py - writes gatr2.capture/1 bundles for viewer tests.

A store-only ZIP of per-stream CSVs plus metadata.json, with the column names
of docs/capture.md (the headers W3's capture_bundle.cpp writes). Used two ways:

- synthetic(): a deterministic recording (a circle in the odometry frame, a
  placement, an odometry epoch change that starts a new segment, telemetry,
  one path, events) whose robot pose at any time is known in closed form, so
  a replay test can check what the page shows.
- write_bundle(): the stub server's recorder writes what it actually sent.

This is test data. It is not produced by the robot and proves nothing about
the Pi's capture code; W3's host tests cover that.

    python capture_fixture.py out.zip      # writes the synthetic bundle
"""

import io
import json
import math
import sys
import zipfile

ROBOT_HEADER = ('pi_host_us,pi_session,reset_count,source,publication,segment,valid,placed,source_clock,'
                'source_ms,measured_pi_host_ms,source_age_ms,odometry_epoch,anchor_revision,'
                'placement_session,placement_sequence,odom_x_m,odom_y_m,odom_heading_deg,'
                'odom_heading_unwrapped_deg,field_x_m,field_y_m,field_heading_deg,'
                'field_heading_unwrapped_deg,odom_vx_m_s,odom_vy_m_s,yaw_rate_deg_s,confidence,'
                'attitude_status,roll_deg,pitch_deg,stationary').split(',')
EVENTS_HEADER = 'pi_host_us,pi_session,reset_count,kind,source,text'.split(',')
TELEMETRY_HEADER = ('pi_host_us,pi_session,reset_count,source,brain_session,decoded,source_clock,source_ms,'
                    'flags,attitude_present,roll_deg,pitch_deg,motion_present,command_id,motion_state,'
                    'motion_reason,plan_mode,path_segment,path_segment_count,target_field_x_m,'
                    'target_field_y_m,target_field_heading_deg,cmd_body_vx_m_s,cmd_body_vy_m_s,'
                    'cmd_omega_deg_s,cross_track_m,distance_error_m,heading_error_deg,drive_fault,'
                    'wheels_present,wheel_count').split(',') + [f'wheel{i}_rpm' for i in range(4)] + ['payload_hex']
PATHS_HEADER = ('pi_host_us,pi_session,reset_count,source,brain_session,command_id,mode,mode_name,'
                'point_count,point_index,field_x_m,field_y_m').split(',')
PICO_SENSOR_HEADER = ('pi_host_us,pi_session,reset_count,source,source_clock,source_ms,frame_version,boot_id,'
                      'acq_epoch,imu_epoch,seq,mask,enc0_counts,enc1_counts,enc2_counts,enc0_delta_counts,'
                      'enc1_delta_counts,enc2_delta_counts,gyro_z_deg_s').split(',')

SESSION = 'fixture0session01'
T0_US = 5_000_000          # first row, Pi host clock
DURATION_S = 12.0
RATE_HZ = 50.0
PLACE_S = 2.0              # placed from here on
EPOCH_S = 7.0              # a new odometry epoch (segment) from here on
RADIUS_M = 0.5
OMEGA = 2 * math.pi / 6.0  # rad/s
ANCHOR = (1.0, 1.2, 30.0)  # field_from_odom while placed (x, y, heading deg)
# synthetic(variant): a span with no robot_state rows ('hole'), or rows that
# repeat the pose of its first row with a growing source_age_ms ('frozen',
# the runtime's rows between measurements); both must replay as stale
STALE_FROM_S = 4.0
STALE_TO_S = 6.9


def wrap180(d):
    x = ((d + 180.0) % 360.0) - 180.0
    return 180.0 if x == -180.0 else x


def compose(a, b):
    h = math.radians(a[2])
    c, s = math.cos(h), math.sin(h)
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap180(a[2] + b[2]))


def odom_pose(t_s):
    """Odometry pose at t (s from the first row): a circle, restarted at EPOCH_S."""
    tt = t_s - EPOCH_S if t_s >= EPOCH_S else t_s
    a = OMEGA * tt
    return (RADIUS_M * math.sin(a), RADIUS_M * (1.0 - math.cos(a)), wrap180(math.degrees(a)))


def robot_at(t_s):
    """What robot_state says at t: (x, y, heading, placed, epoch) with x/y the
    field pose once placed, else the odometry pose (the viewer's rule)."""
    o = odom_pose(t_s)
    placed = t_s >= PLACE_S
    epoch = 2 if t_s >= EPOCH_S else 1
    if placed:
        f = compose(ANCHOR, o)
        return f[0], f[1], f[2], True, epoch
    return o[0], o[1], o[2], False, epoch


def num(v, digits=6):
    if v is None:
        return ''
    if isinstance(v, bool):
        return '1' if v else '0'
    if isinstance(v, int):
        return str(v)
    return f'{v:.{digits}f}'


def csv_text(header, rows):
    out = [','.join(header)]
    for r in rows:
        cells = []
        for h in header:
            v = r.get(h)
            if isinstance(v, str):
                cells.append('"' + v.replace('"', '""') + '"' if any(ch in v for ch in ',"\n') else v)
            else:
                cells.append(num(v))
        out.append(','.join(cells))
    return '\n'.join(out) + '\n'


def write_bundle(metadata, tables, readme='gatr2.capture/1 test bundle (not produced by the robot)\n'):
    """tables: {file name: (header, rows as dicts)} -> ZIP bytes (stored)."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', compression=zipfile.ZIP_STORED) as z:
        z.writestr('metadata.json', json.dumps(metadata, indent=1))
        z.writestr('README.txt', readme)
        for name, (header, rows) in tables.items():
            z.writestr(name, csv_text(header, rows))
    return buf.getvalue()


def synthetic(variant=None):
    n = int(DURATION_S * RATE_HZ)
    robot = []
    pico = []
    frozen = None
    unwrapped = 0.0
    prev_h = None
    for k in range(n):
        t = k / RATE_HZ
        us = T0_US + int(round(t * 1e6))
        o = odom_pose(t)
        x, y, h, placed, epoch = robot_at(t)
        segment = (1 if placed else 0) + (1 if epoch == 2 else 0)
        if prev_h is None or (k and robot and robot[-1]['odometry_epoch'] != epoch):
            unwrapped = o[2]
        else:
            unwrapped += wrap180(o[2] - prev_h)
        prev_h = o[2]
        f = compose(ANCHOR, o) if placed else None
        robot.append({
            'pi_host_us': us, 'pi_session': SESSION, 'reset_count': 0, 'source': 'localization',
            'publication': k + 1, 'segment': segment, 'valid': True, 'placed': placed,
            'source_clock': 'pico', 'source_ms': 1000 + int(t * 1000), 'measured_pi_host_ms': us / 1000.0 - 12.0,
            'source_age_ms': 12.0, 'odometry_epoch': epoch, 'anchor_revision': 1 if placed else 0,
            'placement_session': 0, 'placement_sequence': 1 if placed else 0,
            'odom_x_m': o[0], 'odom_y_m': o[1], 'odom_heading_deg': o[2], 'odom_heading_unwrapped_deg': unwrapped,
            'field_x_m': f[0] if f else None, 'field_y_m': f[1] if f else None, 'field_heading_deg': f[2] if f else None,
            'field_heading_unwrapped_deg': (unwrapped + ANCHOR[2]) if f else None,
            'odom_vx_m_s': RADIUS_M * OMEGA * math.cos(OMEGA * t), 'odom_vy_m_s': RADIUS_M * OMEGA * math.sin(OMEGA * t),
            'yaw_rate_deg_s': math.degrees(OMEGA), 'confidence': 0.9,
            'attitude_status': 'measured', 'roll_deg': 2.0 * math.sin(t), 'pitch_deg': -1.0, 'stationary': False,
        })
        if variant and STALE_FROM_S <= t < STALE_TO_S:
            if variant == 'hole':
                robot.pop()
            elif variant == 'frozen':
                if frozen is None:
                    frozen = dict(robot[-1])
                row = dict(frozen, pi_host_us=us, publication=k + 1)
                row['source_age_ms'] = us / 1000.0 - frozen['measured_pi_host_ms']
                robot[-1] = row
        pico.append({
            'pi_host_us': us - 3000, 'pi_session': SESSION, 'reset_count': 0, 'source': 'pico',
            'source_clock': 'pico', 'source_ms': 1000 + int(t * 1000), 'frame_version': 2, 'boot_id': 17,
            'acq_epoch': 1, 'imu_epoch': 1, 'seq': k % 256, 'mask': 7,
            'enc0_counts': int(2000 * t), 'enc1_counts': int(-300 * t), 'enc2_counts': None,
            'enc0_delta_counts': 40 if k else None, 'enc1_delta_counts': -6 if k else None, 'enc2_delta_counts': None,
            'gyro_z_deg_s': math.degrees(OMEGA) + 0.3,
        })
    telemetry = []
    for k in range(int(DURATION_S * 10)):
        t = k / 10.0
        us = T0_US + int(round(t * 1e6)) + 1500
        seg = min(3, int(t / 3))
        telemetry.append({
            'pi_host_us': us, 'pi_session': SESSION, 'reset_count': 0, 'source': 'brain', 'brain_session': 0x1234,
            'decoded': True, 'source_clock': 'brain', 'source_ms': 50000 + int(t * 1000), 'flags': 3,
            'attitude_present': True, 'roll_deg': 1.5, 'pitch_deg': -0.5, 'motion_present': True, 'command_id': 7,
            'motion_state': 2, 'motion_reason': 0, 'plan_mode': 1, 'path_segment': seg, 'path_segment_count': 4,
            'target_field_x_m': 2.5, 'target_field_y_m': 2.0, 'target_field_heading_deg': 90.0,
            'cmd_body_vx_m_s': 0.3, 'cmd_body_vy_m_s': 0.0, 'cmd_omega_deg_s': 10.0 * math.sin(t),
            'cross_track_m': 0.02 * math.sin(3 * t), 'distance_error_m': max(0.0, 2.0 - 0.15 * t),
            'heading_error_deg': wrap180(90.0 - math.degrees(OMEGA * t)), 'drive_fault': 0,
            'wheels_present': True, 'wheel_count': 2, 'wheel0_rpm': 120.0, 'wheel1_rpm': 118.0,
            'wheel2_rpm': None, 'wheel3_rpm': None, 'payload_hex': '',
        })
    pts = [(1.0, 1.2), (1.5, 1.4), (2.0, 1.6), (2.3, 1.8), (2.5, 2.0)]
    paths = [{
        'pi_host_us': T0_US + 1_000_000, 'pi_session': SESSION, 'reset_count': 0, 'source': 'brain',
        'brain_session': 0x1234, 'command_id': 7, 'mode': 1, 'mode_name': 'direct', 'point_count': len(pts),
        'point_index': i, 'field_x_m': p[0], 'field_y_m': p[1],
    } for i, p in enumerate(pts)]
    events = [
        {'pi_host_us': T0_US + 500_000, 'pi_session': SESSION, 'reset_count': 0, 'kind': 'runtime', 'source': 'system',
         'text': 'tracking_motion: IMU bias calibration done'},
        {'pi_host_us': T0_US + int(PLACE_S * 1e6), 'pi_session': SESSION, 'reset_count': 0, 'kind': 'runtime',
         'source': 'system', 'text': 'placed by Brain, command 3'},
        {'pi_host_us': T0_US + 4_000_000, 'pi_session': SESSION, 'reset_count': 0, 'kind': 'capture', 'source': 'capture',
         'text': 'trigger manual (requester "test, with comma")'},
        {'pi_host_us': T0_US + int(EPOCH_S * 1e6), 'pi_session': SESSION, 'reset_count': 0, 'kind': 'runtime',
         'source': 'system', 'text': 'odometry epoch 2: continuity lost (motion sensor)'},
    ]
    seg_rows = [(0, 0, PLACE_S), (1, PLACE_S, EPOCH_S), (2, EPOCH_S, DURATION_S)]
    metadata = {
        'schema': 'gatr2.capture/1', 'id': 'fixture-synthetic',
        'created': {'pi_host_us': T0_US + 4_000_000, 'unix_ms': None},
        'pi': {'session': SESSION, 'resets': [{'reset_count': 0, 'from_pi_host_us': 0}]},
        'configuration': {'id': 'viewer_test_fixture', 'name': 'viewer test fixture (synthetic, not a robot)'},
        'profile': None,
        'trigger': {'reason': 'manual', 'requester': 'test', 'pre_s': 4.0, 'post_s': 8.0, 'truncated': False},
        'streams': {name: {'selected': True, 'rows': cnt} for name, cnt in
                    (('robot_state', len(robot)), ('pico_sensor', len(pico)), ('brain_telemetry', len(telemetry)),
                     ('path', len(paths)), ('event', len(events)))},
        'drops': {'robot_state': {'hub_ring_full': 0, 'capture_limit': 0}},
        'segments': [{'domain': 'robot', 'index': i, 'reset_count': 0, 'odometry_epoch': 2 if a >= EPOCH_S else 1,
                      'anchor_revision': 1 if a >= PLACE_S else 0,
                      'first_pi_host_us': T0_US + int(a * 1e6), 'last_pi_host_us': T0_US + int(b * 1e6) - 20000,
                      'rows': int((b - a) * RATE_HZ)} for i, a, b in seg_rows],
        'conventions': {'frames': 'field and odometry; +x forward, +y left, CCW headings', 'missing': 'empty field'},
        'markers': [{'pi_host_us': T0_US + 4_000_000, 'text': 'trigger'}],
    }
    tables = {
        'robot_state.csv': (ROBOT_HEADER, robot),
        'pico_sensor.csv': (PICO_SENSOR_HEADER, pico),
        'brain_telemetry.csv': (TELEMETRY_HEADER, telemetry),
        'paths.csv': (PATHS_HEADER, paths),
        'events.csv': (EVENTS_HEADER, events),
    }
    return write_bundle(metadata, tables)


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'fixture_capture.zip'
    data = synthetic()
    with open(out, 'wb') as f:
        f.write(data)
    print(f'wrote {out}: {len(data)} bytes')
