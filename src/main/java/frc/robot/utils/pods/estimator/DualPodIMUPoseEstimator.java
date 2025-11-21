// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pods.estimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.pods.kinematics.DualPodIMUKinematics;
import frc.robot.utils.pods.kinematics.DualPodIMUOdometry;
import frc.robot.utils.pods.kinematics.DualPodIMUPositions;

/**
 * This class wraps {@link DualPodIMUOdometry Dual Pod IMU Odometry} to fuse
 * latency-compensated vision measurements with mecanum drive encoder distance
 * measurements. It will correct for noisy measurements and encoder drift. It is
 * intended to be a drop-in replacement for {@link DualPodIMUOdometry}.
 *
 * <p>
 * {@link DualPodIMUPoseEstimator#update} should be called every robot loop.
 *
 * <p>
 * {@link DualPodIMUPoseEstimator#addVisionMeasurement} can be called as
 * infrequently as you want; if you never call it, then this class will behave
 * mostly like regular encoder odometry.
 */
public class DualPodIMUPoseEstimator extends PoseEstimator<DualPodIMUPositions> {
    /**
     * Constructs a DualPodIMUPoseEstimator with default standard deviations for the
     * model and vision measurements.
     *
     * <p>
     * The default standard deviations of the model states are 0.1 meters for x, 0.1
     * meters for y, and 0.1 radians for heading. The default standard deviations of
     * the vision measurements are 0.45 meters for x, 0.45 meters for y, and 0.45
     * radians for heading.
     *
     * @param kinematics        A correctly-configured kinematics object for your
     *                          odometry pod system.
     * @param gyroAngle         The current gyro angle.
     * @param wheelPositions    The distances driven by each wheel.
     * @param initialPoseMeters The starting pose estimate.
     */
    public DualPodIMUPoseEstimator(DualPodIMUKinematics kinematics, Rotation2d gyroAngle,
            DualPodIMUPositions wheelPositions, Pose2d initialPoseMeters) {
        this(kinematics, gyroAngle, wheelPositions, initialPoseMeters, VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.45, 0.45, 0.45));
    }

    /**
     * Constructs a DualPodIMUPoseEstimator.
     *
     * @param kinematics               A correctly-configured kinematics object for
     *                                 your odometry pod system.
     * @param gyroAngle                The current gyro angle.
     * @param wheelPositions           The distance measured by each wheel.
     * @param initialPoseMeters        The starting pose estimate.
     * @param stateStdDevs             Standard deviations of the pose estimate (x
     *                                 position in meters, y position in meters, and
     *                                 heading in radians). Increase these numbers
     *                                 to trust your state estimate less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position in meters, y position
     *                                 in meters, and heading in radians). Increase
     *                                 these numbers to trust the vision pose
     *                                 measurement less.
     */
    public DualPodIMUPoseEstimator(DualPodIMUKinematics kinematics, Rotation2d gyroAngle,
            DualPodIMUPositions wheelPositions, Pose2d initialPoseMeters, Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super(kinematics, new DualPodIMUOdometry(kinematics, gyroAngle, wheelPositions, initialPoseMeters),
                stateStdDevs, visionMeasurementStdDevs);
    }
}
