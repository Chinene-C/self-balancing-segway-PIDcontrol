# self-balancing-segway-PIDcontrol
PID tuned, two-wheeled, self-balancing segway.

## Signal Processing
Using an integrated microphone, 
a beat detection algorithm was coded for the segway to complete predeterined moves to the beat of a song.
Pointers were used to add all instantaneous energies, comparing each value to the average energy. 
If the energy ratio was sufficient, then a beat was detected and the robot performed a “dance move".

## Complementary Filter
Due to the high frequency noise from the gyroscope and the drift from integrating, a complementary filter was used. The code was initially written in Matlab then adapted to Python.

## PID Feedback Control
A PID controller was coded to work in real time to calculate the required motor speed to balance the segway.
Tuning the PID coefficients was the longest part of the project.

