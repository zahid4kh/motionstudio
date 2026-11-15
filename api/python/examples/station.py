"""Runs a station from signals: start it, wait for a sensor, count parts.

Use with the Part flow cell template (File › Open Template…), whose
signals are named in MotionStudio's Automation › Signals tab.
"""
import motionstudio as ms

with ms.connect() as studio:
    for s in studio.signals.list():
        print(f"{s['name']:20} {s['kind']:9} {s['type']:5} = {s['value']:g}")
    studio.play()
    studio.sleep(10)
    print("time", studio.time)
