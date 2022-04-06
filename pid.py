'''
vessel: PID
2022-04-01
'''


import time
from collections import deque

import krpc

conn = krpc.connect(name='test')
conn.ui.message('connected')
sc = conn.space_center
vessel = sc.active_vessel
flight = vessel.flight(vessel.orbit.body.reference_frame)
control = vessel.control
g = vessel.orbit.body.surface_gravity

vessel.auto_pilot.target_pitch = 90
vessel.auto_pilot.engage()
control.activate_next_stage()
time.sleep(1)

max_thrust = vessel.max_thrust
while flight.vertical_speed < 5:
    throttle = vessel.mass * (g + 1) / max_thrust
    if throttle > 1:
        raise RuntimeWarning('not enough thrust')
    control.throttle = throttle
    time.sleep(0.1)
conn.ui.message('acceleration finished')
while flight.surface_altitude < 45:
    control.throttle = vessel.mass * g / max_thrust
    time.sleep(0.1)
conn.ui.message('altitude reached')

conn.ui.message('pid start')
Kp, Ki, Kd = -1, 0, -0.1
dt = 0.05
prevErr = deque([flight.surface_altitude - 50])
lastCtrl = deque()
while 1:
    err = flight.surface_altitude - 50
    ctrlP = Kp * err
    ctrlI = Ki * 0
    ctrlD = Kd * (err - prevErr[-1]) / dt
    prevErr.append(err)
    while len(prevErr) > 100:
        prevErr.popleft()
    if max(map(abs, prevErr)) < 0.05:
        conn.ui.message('reached desired altitude')
        break
    ctrl = ctrlP + ctrlI + ctrlD
    lastCtrl.append(abs(ctrl))
    while len(lastCtrl) > 20:
        lastCtrl.popleft()
    ctrl *= 3 / max(lastCtrl)
    conn.ui.message(f'{ctrl=:.2f}, alt={flight.surface_altitude:.2f}m', dt)
    control.throttle = vessel.mass * (g + ctrl) / max_thrust
    time.sleep(dt)

conn.ui.message('landing')
while flight.vertical_speed > -3:
    throttle = vessel.mass * (g - 1) / max_thrust
    if throttle > 1:
        raise RuntimeWarning('not enough thrust')
    control.throttle = throttle
    time.sleep(0.1)
conn.ui.message('deceleration finished')
while abs(flight.vertical_speed) > 0.1:
    control.throttle = vessel.mass * g / max_thrust
    time.sleep(0.1)

control.throttle = 0
conn.ui.message('landed!')

