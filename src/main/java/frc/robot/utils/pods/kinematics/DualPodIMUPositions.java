// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pods.kinematics;

import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.units.measure.Distance;

/**
 * Represents the positions of the odometry pod and the gyro in a dual pod
 * (forward and lateral) + IMU (gyro) odometry pod system.
 */
public class DualPodIMUPositions implements Interpolatable<DualPodIMUPositions> {
    /** Distance measured by the forward pod. */
    public double forwardMeters;

    /** Distance measured by the lateral pod. */
    public double lateralMeters;

    /** Rotation reported by the gyro. */
    public double thetaRadians;

    /** Constructs a DualPodIMUPositions with zeros for all member fields. */
    public DualPodIMUPositions() {
    }

    /**
     * Constructs a DualPodIMUPositions.
     *
     * @param forwardMeters Distance measured by the forward pod.
     * @param lateralMeters Distance measured by the lateral pod.
     * @param thetaRadians  Rotation reported by the gyro.
     */
    public DualPodIMUPositions(double forwardMeters, double lateralMeters, double thetaRadians) {
        this.forwardMeters = forwardMeters;
        this.lateralMeters = lateralMeters;
        this.thetaRadians = thetaRadians;
    }

    /**
     * Constructs a DualPodIMUPositions.
     *
     * @param forward Distance measured by the forward pod.
     * @param lateral Distance measured by the lateral pod.
     */
    public DualPodIMUPositions(
            Distance forward, Distance lateral, Rotation2d theta) {
        this(forward.in(Meters), lateral.in(Meters), theta.getRadians());
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof DualPodIMUPositions other
                && Math.abs(other.forwardMeters - forwardMeters) < 1E-9
                && Math.abs(other.lateralMeters - lateralMeters) < 1E-9;
    }

    @Override
    public int hashCode() {
        return Objects.hash(forwardMeters, lateralMeters);
    }

    @Override
    public String toString() {
        return String.format(
                "PodWheelPositions(Forward: %.2f m, Lateral: %.2f m, Rotation: %.2f rad)",
                forwardMeters,
                lateralMeters,
                thetaRadians);
    }

    @Override
    public DualPodIMUPositions interpolate(DualPodIMUPositions endValue, double t) {
        return new DualPodIMUPositions(
                MathUtil.interpolate(this.forwardMeters, endValue.forwardMeters, t),
                MathUtil.interpolate(this.lateralMeters, endValue.lateralMeters, t),
                MathUtil.interpolate(this.thetaRadians, endValue.thetaRadians, t));
    }
}
