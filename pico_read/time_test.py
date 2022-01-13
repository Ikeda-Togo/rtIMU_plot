import time

predate =0
while True:
    date_s = time.ticks_ms()- predate
    print(date_s)
    predate = time.ticks_ms()
    time.sleep(0.02)
    