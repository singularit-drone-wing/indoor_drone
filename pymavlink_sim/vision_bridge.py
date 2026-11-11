"""SITL-only simulated external-navigation bridge. NOT for real aircraft.

Read SIM_STATE (MAVLink message 108, simulator ground truth), convert global
coordinates to local NED, inject VISION_POSITION_ESTIMATE at 15 Hz.

Prerequisites: pip install pymavlink
ArduCopter SITL must expose a SEPARATE MAVLink TCP endpoint for this process.
Run --help for options. Never feed EKF-fused GLOBAL_POSITION_INT back into EKF.
"""
import argparse
import math
import time

from pymavlink import mavutil

EARTH_RADIUS = 6378137.0


def local_ned(lat, lon, alt, lat0, lon0, alt0):
    north = math.radians(lat - lat0) * EARTH_RADIUS
    east = math.radians(lon - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
    down = alt0 - alt
    return north, east, down


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--connect', default='tcp:127.0.0.1:5762', help='SITL MAVLink endpoint; use a FREE dedicated port')
    p.add_argument('--rate', type=float, default=15)
    p.add_argument('--truth-timeout', type=float, default=2.0)
    p.add_argument('--dropout-after', type=float, default=-1, help='Seconds after start of injection; -1 disables')
    p.add_argument('--dropout-duration', type=float, default=3)
    p.add_argument('--set-origin', action='store_true', help='Set EKF origin from initial simulator truth (SITL ONLY)')
    args = p.parse_args()
    if not 1 <= args.rate <= 50:
        p.error('--rate must be between 1 and 50')
    if args.dropout_after >= 0 and args.dropout_duration < 0:
        p.error('dropout duration must be nonnegative')

    print('Connecting to', args.connect)
    vehicle = mavutil.mavlink_connection(args.connect, source_system=245, source_component=190)
    hb = vehicle.wait_heartbeat(timeout=20)
    if hb is None:
        raise RuntimeError('No heartbeat. Check dedicated TCP port and SITL status.')
    print('Heartbeat received from system', vehicle.target_system)

    # Ask SITL to emit SIM_STATE ground-truth messages; support depends on build.
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 108,
        int(1e6 / max(args.rate, 15)), 0, 0, 0, 0, 0)

    first = None
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        m = vehicle.recv_match(type='SIM_STATE', blocking=True, timeout=1)
        if m and math.isfinite(float(m.lat)) and math.isfinite(float(m.lon)):
            first = m
            break
    if first is None:
        raise RuntimeError('No SIM_STATE ground truth detected. This SITL build/port may not stream message 108. Do NOT replace with GLOBAL_POSITION_INT.')

    def truth_latlon(m):
        lat_i = getattr(m, 'lat_int', 0)
        lon_i = getattr(m, 'lon_int', 0)
        # High-resolution integer coordinates avoid float32 loss at geographic degrees.
        lat = lat_i / 1e7 if lat_i else float(m.lat)
        lon = lon_i / 1e7 if lon_i else float(m.lon)
        return lat, lon

    lat0, lon0 = truth_latlon(first)
    alt0 = float(first.alt)
    print(f'Vision local origin: {lat0:.7f}, {lon0:.7f}, alt {alt0:.2f}m')
    print('Start SITL ON THE GROUND before starting bridge; this first pose is local (0,0,0).')
    if args.set_origin:
        vehicle.mav.set_gps_global_origin_send(vehicle.target_system,
            round(lat0 * 1e7), round(lon0 * 1e7), round(alt0 * 1000), 0)
        print('Requested EKF origin from SITL truth; verify origin/position in Mission Planner.')
    else:
        print('Set EKF origin manually in Mission Planner if not already established.')

    # MAVLink VISION_POSITION_ESTIMATE covariance is upper triangle (21 elements).
    # Explicit finite diagonal variances, all other covariances zero.
    covariance = [0.0] * 21
    for index, variance in [(0, 0.03**2), (6, 0.03**2), (11, 0.05**2),
                            (15, 0.03**2), (18, 0.03**2), (20, 0.05**2)]:
        covariance[index] = variance

    start = time.monotonic()
    last_truth = first
    last_truth_time = start
    next_send = start
    sent = 0
    last_status = 0
    try:
        while True:
            # Read all available messages without blocking to retain freshest truth.
            while True:
                m = vehicle.recv_match(blocking=False)
                if m is None:
                    break
                if m.get_type() == 'SIM_STATE':
                    last_truth, last_truth_time = m, time.monotonic()
            now = time.monotonic()
            if now < next_send:
                time.sleep(min(next_send - now, 0.008))
                continue
            next_send += 1.0 / args.rate
            if next_send < now - (1 / args.rate):
                next_send = now + 1 / args.rate
            if now - last_truth_time > args.truth_timeout:
                if now - last_status > 2:
                    print('WARNING stale SIM_STATE; stopping vision injection until fresh truth resumes')
                    last_status = now
                continue
            t = now - start
            in_dropout = (args.dropout_after >= 0 and
                          args.dropout_after <= t < args.dropout_after + args.dropout_duration)
            if in_dropout:
                if now - last_status > 2:
                    print(f'DROPOUT test active t={t:.1f}s: vision messages withheld')
                    last_status = now
                continue
            lat, lon = truth_latlon(last_truth)
            x, y, z = local_ned(lat, lon, float(last_truth.alt), lat0, lon0, alt0)
            vehicle.mav.vision_position_estimate_send(int(now * 1e6),
                x, y, z, float(last_truth.roll), float(last_truth.pitch),
                float(last_truth.yaw), covariance, 0)
            sent += 1
            if now - last_status > 2:
                print(f't={t:.1f}s sent={sent} NED=({x:+.2f},{y:+.2f},{z:+.2f}) yaw={last_truth.yaw:+.2f}')
                last_status = now
    except KeyboardInterrupt:
        print('\nStopped. Vision data no longer available. LAND SITL vehicle if airborne.')
    finally:
        vehicle.close()

if __name__ == '__main__':
    main()
