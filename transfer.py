'''
vessel: Kerbal X
2022-04-03
'''

import math
import time

import krpc
from maneuver import executeManeuver
from util import *

conn = krpc.connect(name='transfer')
vessel = conn.space_center.active_vessel
kerbin = conn.space_center.bodies['Kerbin']
mun = conn.space_center.bodies['Mun']

ut = conn.add_stream(getattr, conn.space_center, 'ut')

r1 = vessel.orbit.semi_major_axis
r2 = mun.orbit.semi_major_axis
targetPhase = 180 * (1 - ((r1 + r2) / (2 * r2)) ** 1.5)

theta1 = (vessel.flight().longitude + 360) % 360
munPos = mun.position(kerbin.reference_frame)
theta2 = (math.atan2(munPos[2], munPos[0]) / math.pi * 180 + 360) % 360
currentPhase = (theta2 - theta1 + 360) % 360

display(conn, f'{targetPhase=:.2f}, {currentPhase=:.2f}')

w1 = 360 / vessel.orbit.period
w2 = 360 / mun.rotational_period
waitTime = ((currentPhase - targetPhase + 360) % 360) / (w1 - w2)

mu = vessel.orbit.body.gravitational_parameter
deltav = math.sqrt(mu / r1) * (math.sqrt(2 * r2 / (r1 + r2)) - 1)
display(conn, f'{deltav=:.2f}')
node = vessel.control.add_node(ut() + waitTime, prograde=deltav)
executeManeuver(conn, node)
