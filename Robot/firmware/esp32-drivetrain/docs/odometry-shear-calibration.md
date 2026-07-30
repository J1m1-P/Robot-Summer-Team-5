# Angular Odometry Shear Calibration

This procedure calibrates cross-axis skew in the fused (x/y) odometry output. It assumes the odometry scale is already correct, so the correction only shears the coordinate system and does not change the diagonal scale terms.

## Principle

Use a correction matrix with unit diagonal:

```text
|x_corrected|   |1     k_xy| |x_raw|
|y_corrected| = |k_yx  1   | |y_raw|
```

The matrix removes the sideways displacement observed while following known straight lines.

## Calibration procedure

1. Mark a straight tape path as long as practical. About 2 m is a good target if the robot can follow it reliably.
2. Reset the fused pose before each run, or record the displacement from the starting pose.
3. Follow the tape in the `+x` direction at least three times. Record the final raw displacement ((x, y)) for every run and average the results.
4. Repeat in the `-x` direction at least three times.
5. Follow a tape path in the `+y` direction at least three times and average the results.
6. If possible, also repeat in the `-y` direction. This provides a symmetric estimate and a useful validation check.

Keep every run in the same coordinate frame. If the robot starts at a different heading, rotate the measured displacement back into the calibration frame before averaging.

## Calculating the shear

Let the averaged displacement for the positive (x) run be:

```text
v_x = (x_x, y_x)
```

The corrected (y) displacement should be zero, so calculate:

```text
k_yx = -y_x / x_x
```

For the positive (y) run, let the averaged displacement be:

```text
v_y = (x_y, y_y)
```

The corrected (x) displacement should be zero, so calculate:

```text
k_xy = -x_y / y_y
```

If both reverse-direction runs are available, use the signed, symmetric averages first:

```text
v_x = (average(+x) - average(-x)) / 2
v_y = (average(+y) - average(-y)) / 2
```

Then calculate the shear coefficients from these two vectors.

## Runtime application

Apply the correction to every raw position displacement:

```cpp
corrected_x = raw_x + k_xy * raw_y;
corrected_y = k_yx * raw_x + raw_y;
```

Apply it to displacement relative to the pose origin, not independently to unrelated absolute coordinates:

```cpp
corrected_pose.x = origin.x + k_xy * delta_y + delta_x;
corrected_pose.y = origin.y + k_yx * delta_x + delta_y;
```

Do not modify heading with this matrix.

## Distance and validation

A longer calibration path makes the cross-axis error easier to measure. A 2 m calibration is valid for small movements such as 5 cm if the skew is linear and stable; the correction should be independent of travel distance.

After calculating the coefficients:

- Repeat the long `+x` and `+y` runs and check that cross-axis error is near zero.
- Test several short movements, such as 5 cm.
- Check that remaining error is roughly proportional to distance.

If short-distance behavior is substantially different, investigate encoder quantization, backlash, wheel slip, or line-following transients. Those effects cannot be corrected reliably by a single shear matrix.

px
# ===== ODOMETRY CALIBRATION RESULT =====
# Initial: x=-0.000245 y=-0.000763
# Final:   x=1.970076 y=0.309947
# Delta:   dx=1.970321 dy=0.310710
# =========================================
# ===== ODOMETRY CALIBRATION RESULT =====
# Initial: x=0.000031 y=-0.000068
# Final:   x=1.969262 y=0.307109
# Delta:   dx=1.969231 dy=0.307176
mx
# ===== ODOMETRY CALIBRATION RESULT =====
# Initial: x=-0.000058 y=-0.000948
# Final:   x=-1.946900 y=0.414963
# Delta:   dx=-1.946842 dy=0.415912
# =========================================
# ===== ODOMETRY CALIBRATION RESULT =====
# Initial: x=0.000047 y=-0.000909
# Final:   x=-1.946291 y=0.376790
# Delta:   dx=-1.946338 dy=0.377698
# =========================================
py
# ===== ODOMETRY CALIBRATION RESULT =====
# Initial: x=0.000020 y=0.000000
# Final:   x=0.557843 y=1.884120
# Delta:   dx=0.557823 dy=1.884120
# =========================================
# ===== ODOMETRY CALIBRATION RESULT =====
# Initial: x=0.000004 y=-0.000000
# Final:   x=0.493962 y=1.912980
# Delta:   dx=0.493958 dy=1.912980
# =========================================