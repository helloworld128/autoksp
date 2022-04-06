import time
import krpc

from maneuver import executeManeuver

conn = krpc.connect(name='test')
vessel = conn.space_center.active_vessel
ut = conn.add_stream(getattr, conn.space_center, 'ut')

node = vessel.control.add_node(ut() + 60, prograde=100)
executeManeuver(conn, node)
