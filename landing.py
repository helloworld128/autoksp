'''
vessel: Kerbal X
2022-04-04
'''

import math
import time

import krpc
from util import *


def getTimeToImpact(conn):
    vessel = conn.space_center.active_vessel
    orbit = vessel.orbit
    body = orbit.body
    ref = body.reference_frame
    flight = vessel.flight(ref)
    ut = conn.space_center.ut

    low, up = 0, flight.surface_altitude / abs(flight.vertical_speed)
    while low + 0.01 < up:
        mid = 0.5 * (low + up)
        pos = orbit.position_at(ut + mid, ref)
        alt = body.altitude_at_position(pos, ref)
        surfaceAlt = body.surface_height(body.latitude_at_position(
            pos, ref), body.longitude_at_position(pos, ref))
        if alt > surfaceAlt:
            low = mid
        else:
            up = mid
    return low


conn = krpc.connect(name='landing')
vessel = conn.space_center.active_vessel
orbit = vessel.orbit
body = orbit.body
ref = body.reference_frame
flight = vessel.flight(ref)

alt = conn.add_stream(getattr, flight, 'surface_altitude')
spd = conn.add_stream(getattr, flight, 'speed')
hspd = conn.add_stream(getattr, flight, 'horizontal_speed')
vspd = conn.add_stream(getattr, flight, 'vertical_speed')
ut = conn.add_stream(getattr, conn.space_center, 'ut')


maxDecel = vessel.available_thrust / vessel.mass - body.surface_gravity
lo, hi = 0, getTimeToImpact(conn)
while lo + 0.1 < hi:
    mid = 0.5 * (lo + hi)
    pos = orbit.position_at(ut() + mid, ref)
    alt1 = body.altitude_at_position(pos, ref)
    spd1 = math.sqrt(body.gravitational_parameter * (2 /
                     orbit.radius_at(ut() + mid) - 1 / orbit.semi_major_axis))
    if maxDecel > 0.5 * spd1 ** 2 / alt1:
        lo = mid
    else:
        hi = mid
display(conn, f'calculated burn start {alt1=:.2f}m, {spd1=:.2f}m/s')
conn.space_center.warp_to(ut() + lo - 5)

vessel.control.sas = True
time.sleep(0.1)
vessel.control.sas_mode = conn.space_center.SASMode.retrograde
display(conn, 'orienting vessel')
while vessel.direction(vessel.reference_frame)[1] < 0.95:
    time.sleep(0.1)

display(conn, 'wait for deceleration burn')
while alt() > 500 + 0.5 * spd() ** 2 / maxDecel:
    time.sleep(0.1)

legsDeployed = False
display(conn, 'start deceleration burn')
while alt() > 50:
    targetDecel = 0.5 * spd() ** 2 / (alt() - 30)
    thrust = (targetDecel + body.surface_gravity) * vessel.mass
    throttle = thrust / vessel.available_thrust
    if throttle > 1:
        print(f'calculated throttle {throttle} exceeds maximum!')
    vessel.control.throttle = min(1, throttle)

    if alt() < 1000 and not legsDeployed:
        display(conn, 'deploying landing legs')
        for leg in vessel.parts.legs:
            leg.deployed = True
        legsDeployed = True

    time.sleep(0.1)

display(conn, 'preparing for touchdown, limiting vs to 3m/s')
while alt() > 5 and abs(vspd()) > 0.1:
    throttle = (body.surface_gravity - vspd() - 3) * \
        vessel.mass / vessel.available_thrust
    vessel.control.throttle = throttle
    time.sleep(0.1)

display(conn, 'touchdown')
vessel.control.throttle = 0
