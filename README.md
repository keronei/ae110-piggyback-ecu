## DIY Piggyback ECU

The current build utilizes 3 inputs
- Single IGT signal from stock ECU
- Cam sensor signal from A3144 hall sensor mounted in the distributor
- Battery level input based on divider resistors on the buck converter

## Basic Flow
Arduino listens to atleast 2 consistent signals from the cam sensor, each time the signal is received, it means the rotor is headed to cylender 2 and therefore the next IGT signal will be for cylender 2.

Once the initial 2 have been seen, firing starts on cylender 2 and follows the firing order for 5A for the next 3 signals, then repeats - from here, it nolonger takes into consideration the signal because it is considered *SYNCED*.

The battery voltage is used to compute dwell for compensation purpose instead of a fixed dwell value considering charge time differs with varying voltage level.

## Design descisions
- IGT uses analog pin for interrupt because it helps in silencing excess noise on the line.
- TC4424 is used as the coild driver bacause of noise immunity, currently two channels of a single IC are used to drive one coil for prolonged perfomance.

