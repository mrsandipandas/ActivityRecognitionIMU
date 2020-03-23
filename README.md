Homepage: http://sensorfusion.se/
Study material: https://www.control.isy.liu.se/student/tsrt14/file/orientation.pdf

Rotation can descrbed in many ways:
- Rotation matrix: Uses 9 values to represent 3DOF. Not efficient.
- Euler angles: Not unique. Suffers from gimble lock.
- Quartenions: More stable. Uses 4 variables to represent rotation in 3D.

Find out Quartenion delta and average it over few iterations:
- Q = [a_1*q_1 a_2*q_2 ... a_n*q_n]
- a_1 + a_2 + ..... + a_n = 1

Majority voting for output on the activity. 
ML(LDA) can be applied along with gyro and quartenion del to remove false positives due to fast rotation.