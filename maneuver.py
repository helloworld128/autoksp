import math
import time

import krpc
from util import *


def executeManeuver(conn, node):
    vessel = conn.space_center.active_vessel
    ut = conn.add_stream(getattr, conn.space_center, 'ut')

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(node.delta_v / Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    # Orientate ship
    display(conn, 'orientating ship for maneuver')
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = node.burn_vector(node.reference_frame)
    vessel.auto_pilot.engage()
    vessel.auto_pilot.wait()

    # Wait until burn
    display(conn, 'waiting until burn')
    burn_ut = node.ut - (burn_time / 2)
    warpto_ut = burn_ut - 5
    conn.space_center.warp_to(warpto_ut)

    # Execute burn
    display(conn, 'ready to execute burn')
    while ut() < burn_ut:
        time.sleep(0.01)
    display(conn, 'executing burn')
    vessel.control.throttle = 1.0
    while ut() < burn_ut + burn_time - 0.1:
        time.sleep(0.01)

    remaining_burn = conn.add_stream(
        node.remaining_burn_vector, node.reference_frame)
    init_remain = remaining_burn()[1]
    display(conn, f'fine tuning, {init_remain} dv remaining')
    prev_remain = init_remain
    ref_thrust = 0.5 * init_remain / (vessel.available_thrust / vessel.mass)
    while 1:
        remain = remaining_burn()[1]
        if remain > prev_remain or remain < 0.3:
            break
        vessel.control.throttle = max(0.001, ref_thrust / init_remain * remain)
        prev_remain = remain
    vessel.control.throttle = 0.0
    time.sleep(2)
    node.remove()
    display(conn, 'maneuver complete')
