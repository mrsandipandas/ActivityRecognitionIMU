# Activity recognition using IMU

### 1. Course materials

- [Homepage](http://sensorfusion.se/)
- [Study material](https://www.control.isy.liu.se/student/tsrt14/file/orientation.pdf)

### 2. Why use quartenions

#### Rotation can descrbed in many ways
- Rotation matrix: Uses 9 values to represent 3DOF. Not efficient
- Euler angles: Not unique. Suffers from gimble lock
- Quartenions: More stable. Uses 4 variables to represent rotation in 3D

#### Quartenion operations
- Average
```Q = [a_1*q_1 a_2*q_2 ... a_n*q_n]
- a_1 + a_2 + ..... + a_n = 1
```
- Difference between two quartenions
![equation](https://latex.codecogs.com/gif.latex?\theta&space;=&space;2&space;\arccos&space;\left(|&space;\langle&space;p,&space;q&space;\rangle&space;|\right))
where, ![equation](https://latex.codecogs.com/gif.latex?\langle&space;p,q&space;\rangle&space;=&space;p_1q_1&space;&plus;&space;p_2q_2&space;&plus;&space;p_3q_3&space;&plus;&space;p_4q_4)

#### Activity recognizition
- Used thersholding
- Majority voting for output on the activity over a window length
- ML(LDA) can be applied along with gyro and quartenion derivative to remove false positives due to fast rotation

### 3. Run
- `filterTemplate`
- Change thersholding in filter settings

### 4. To do
- Calculate and develop own quartenion algo
