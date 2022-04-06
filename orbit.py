'''
vessel: Kerbal X
2022-04-03
'''

import math
import time
import multiprocessing as mp
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import krpc
from maneuver import executeManeuver
from util import *


def runPlot(q1, q2):
    alt = [], []
    angle = [], [], [], []
    figure, ax = plt.subplots(2, 1)
    ax[0].plot(*alt)
    ax[0].set_title('altitude')
    ax[0].set_xlabel('time/s')
    ax[0].set_ylabel('alt/m')
    ax[1].plot(angle[0], angle[1], label='pitch')
    ax[1].plot(angle[2], angle[3], label='velocity')
    ax[1].set_title('pitch/velocity angle')
    ax[1].set_xlabel('time/s')
    ax[1].set_ylabel('angle')

    def update(frame):
        while not q1.empty():
            x, y = q1.get()
            alt[0].append(x)
            alt[1].append(y)
        ax[0].get_lines()[0].set_data(*alt)
        ax[0].relim()
        ax[0].autoscale_view()
        while not q2.empty():
            x0, x1, x2, x3 = q2.get()
            angle[0].append(x0)
            angle[1].append(x1)
            angle[2].append(x2)
            angle[3].append(x3)
        ax[1].get_lines()[0].set_data(angle[0], angle[1])
        ax[1].get_lines()[1].set_data(angle[2], angle[3])
        ax[1].relim()
        ax[1].autoscale_view()
        return ax[0].get_lines() + ax[1].get_lines()

    animation = FuncAnimation(figure, update, interval=200)
    plt.show()


def main():
    q1, q2 = mp.Queue(), mp.Queue()
    p = mp.Process(target=runPlot, args=(q1, q2))
    p.start()

    turn_start_altitude = 2500
    turn_end_altitude = 40000
    target_altitude = 100000

    conn = krpc.connect(name='orbit')
    vessel = conn.space_center.active_vessel

    # Set up streams for telemetry
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
    pitch = conn.add_stream(getattr, vessel.flight(), 'pitch')
    vs = conn.add_stream(getattr, vessel.flight(
        vessel.orbit.body.reference_frame), 'vertical_speed')
    hs = conn.add_stream(getattr, vessel.flight(
        vessel.orbit.body.reference_frame), 'horizontal_speed')
    booster_stages = [4, 5, 6]
    boosters = vessel.resources_in_decouple_stage(
        stage=booster_stages[-1], cumulative=False)
    boosters_fuel = conn.add_stream(boosters.amount, 'LiquidFuel')

    # Pre-launch setup
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 1.0

    # Countdown...
    display(conn, '3...')
    time.sleep(1)
    display(conn, '2...')
    time.sleep(1)
    display(conn, '1...')
    time.sleep(1)
    display(conn, 'launch!')
    t0 = ut()

    # Activate the first stage
    vessel.control.activate_next_stage()
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)

    # Main ascent loop
    turn_angle = 0
    last = time.time()
    while True:
        if time.time() > last + 0.5:
            last = time.time()
            log = True
            t = ut() - t0
            q1.put((t, altitude()))
            q2.put((t, pitch(), t, math.atan(vs() / hs()) / math.pi * 180))
        else:
            log = False

        # Gravity turn
        if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
            frac = ((altitude() - turn_start_altitude) /
                    (turn_end_altitude - turn_start_altitude))
            turn_angle = frac * 90
            vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

        # Separate boosters when finished
        if booster_stages:
            if boosters_fuel() < 0.1:
                vessel.control.activate_next_stage()
                display(conn, f'booster stage {booster_stages[-1]} separated')
                booster_stages.pop()
                if booster_stages:
                    boosters = vessel.resources_in_decouple_stage(
                        stage=booster_stages[-1], cumulative=False)
                    boosters_fuel = conn.add_stream(
                        boosters.amount, 'LiquidFuel')

        # Decrease throttle when approaching target apoapsis
        if target_altitude * 0.9 < apoapsis() < target_altitude:
            vessel.control.throttle = max(
                0.2, 1 - (apoapsis() - target_altitude * 0.9) / (target_altitude * 0.1))
        elif target_altitude <= apoapsis():
            display(conn, 'target apoapsis reached')
            vessel.control.throttle = 0.0
            break

        time.sleep(0.1)

    # Wait until out of atmosphere
    conn.space_center.physics_warp_factor = 3
    while altitude() < 70000:
        time.sleep(0.1)
    conn.space_center.physics_warp_factor = 0

    # apoapsis correction
    display(conn, 'apoapsis correction')
    while apoapsis() < target_altitude:
        vessel.control.throttle = max(
            0.04, 1 - (apoapsis() - target_altitude * 0.9) / (target_altitude * 0.1))
        time.sleep(0.1)
    vessel.control.throttle = 0.0

    # Plan circularization burn (using vis-viva equation)
    display(conn, 'planning circularization burn')
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    node = vessel.control.add_node(
        ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

    executeManeuver(conn, node)
    display(conn, 'launch complete')

    vessel.control.activate_next_stage()
    time.sleep(0.1)
    vessel.control.activate_next_stage()


if __name__ == '__main__':
    main()
