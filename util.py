import krpc


def display(conn, text, duration=5.0):
    conn.ui.message(str(text), duration=duration, size=32.0)
