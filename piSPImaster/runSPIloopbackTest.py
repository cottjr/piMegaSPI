# runSPIloopbackTest.py

import time
import piSPImaster

testLoop = piSPImaster.SPIloopbackTest()

while True:
    testLoop.run()

    # Pause so we can see results
    time.sleep(.05)
