// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pods.kinematics;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Represents the speeds of the odometry pods and the gyro in a dual pod
 * (forward and lateral) + IMU (gyro) odometry pod system.
 */
public class DualPodIMUSpeeds {
    /** Speed of the forward pod. */
    public double forwardMetersPerSecond;

    /** Speed of the lateral pod. */
    public double lateralMetersPerSecond;

    /** Rotational speed reported by the gyro. */
    public double omegaRadiansPerSecond;

    /** Constructs a DualPodIMUSpeeds with zeros for all member fields. */
    public DualPodIMUSpeeds() {
    }

    /**
     * Constructs a DualPodIMUSpeeds.
     *
     * @param forwardMetersPerSecond     Speed of the forward pod.
     * @param lateralMetersPerSecond     Speed of the lateral pod.
     * @param omegaRadiansPerSecondSpeed Rotational speed reported by the gyro.
     */
    public DualPodIMUSpeeds(
            double forwardMetersPerSecond,
            double lateralMetersPerSecond,
            double omegaRadiansPerSecond) {
        this.forwardMetersPerSecond = forwardMetersPerSecond;
        this.lateralMetersPerSecond = lateralMetersPerSecond;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    /**
     * Constructs a DualPodIMUSpeeds.
     *
     * @param forward    Speed of the forward pod.
     * @param lateral    Speed of the lateral pod.
     * @param rotational Rotational speed of the robot reported by the gyro.
     */
    public DualPodIMUSpeeds(
            LinearVelocity forward,
            LinearVelocity lateral,
            Rotation2d rotation) {
        this(
                forward.in(MetersPerSecond),
                lateral.in(MetersPerSecond),
                rotation.getRadians());
    }

    /**
     * Adds two DualPodIMUSpeeds and returns the sum.
     *
     * <p>
     * For example, DualPodIMUSpeeds{1.0, 0.5, 2.0} +
     * DualPodIMUSpeeds{2.0, 1.5, 0.5} = DualPodIMUSpeeds{3.0, 2.0, 2.5}
     *
     * @param other The DualPodIMUSpeeds to add.
     * @return The sum of the DualPodIMUSpeeds.
     */
    public DualPodIMUSpeeds plus(DualPodIMUSpeeds other) {
        return new DualPodIMUSpeeds(
                forwardMetersPerSecond + other.forwardMetersPerSecond,
                lateralMetersPerSecond + other.lateralMetersPerSecond,
                omegaRadiansPerSecond + other.omegaRadiansPerSecond);
    }

    /**
     * Subtracts the other DualPodIMUSpeeds from the current
     * DualPodIMUSpeeds and returns the difference.
     * 
     * <p>
     * For example, DualPodIMUSpeeds{5.0, 4.0, 6.0} -
     * DualPodIMUSpeeds{1.0, 2.0, 3.0,} = DualPodIMUSpeeds{4.0, 2.0, 3.0}
     *
     * @param other The DualPodIMUSpeeds to subtract.
     * @return The difference between the two DualPodIMUSpeeds.
     */
    public DualPodIMUSpeeds minus(DualPodIMUSpeeds other) {
        return new DualPodIMUSpeeds(
                forwardMetersPerSecond - other.forwardMetersPerSecond,
                lateralMetersPerSecond - other.lateralMetersPerSecond,
                omegaRadiansPerSecond - other.omegaRadiansPerSecond);
    }

    /**
     * Returns the inverse of the current DualPodIMUSpeeds. This is equivalent
     * to negating all components of the DualPodIMUSpeeds.
     *
     * @return The inverse of the current DualPodIMUSpeeds.
     */
    public DualPodIMUSpeeds unaryMinus() {
        return new DualPodIMUSpeeds(
                -forwardMetersPerSecond,
                -lateralMetersPerSecond,
                -omegaRadiansPerSecond);
    }

    /**
     * Multiplies the DualPodIMUSpeeds by a scalar and returns the new
     * DualPodIMUSpeeds.
     *
     * <p>
     * For example, DualPodIMUSpeeds{2.0, 2.5, 3.0} * 2 =
     * DualPodIMUSpeeds{4.0, 5.0, 6.0}
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled DualPodIMUSpeeds.
     */
    public DualPodIMUSpeeds times(double scalar) {
        return new DualPodIMUSpeeds(
                forwardMetersPerSecond * scalar,
                lateralMetersPerSecond * scalar,
                omegaRadiansPerSecond * scalar);
    }

    /**
     * Divides the DualPodIMUSpeeds by a scalar and returns the new
     * DualPodIMUSpeeds.
     *
     * <p>
     * For example, DualPodIMUSpeeds{2.0, 2.5, 1.5, 1.0} / 2 =
     * DualPodIMUSpeeds{1.0,
     * 1.25, 0.75, 0.5}
     *
     * @param scalar The scalar to divide by.
     * @return The scaled DualPodIMUSpeeds.
     */
    public DualPodIMUSpeeds div(double scalar) {
        return new DualPodIMUSpeeds(
                forwardMetersPerSecond / scalar,
                lateralMetersPerSecond / scalar,
                omegaRadiansPerSecond / scalar);
    }

    @Override
    public String toString() {
        return String.format(
                "DualPodIMUWheelSpeeds(Forward: %.2f m/s, Lateral: %.2f m/s, Rotational: %.2f rad/s)",
                forwardMetersPerSecond,
                lateralMetersPerSecond,
                omegaRadiansPerSecond);
    }
}
