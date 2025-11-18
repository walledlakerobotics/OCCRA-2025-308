// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pods.estimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.robot.utils.pods.kinematics.DualPodIMUKinematics;
import frc.robot.utils.pods.kinematics.DualPodIMUOdometry3d;
import frc.robot.utils.pods.kinematics.DualPodIMUPositions;

/**
 * This class wraps {@link DualPodIMUOdometry3d Dual Pod IMU Odometry} to fuse
 * latency-compensated vision measurements with mecanum drive encoder distance
 * measurements. It will correct for noisy measurements and encoder drift. It is
 * intended to be a drop-in replacement for {@link DualPodIMUOdometry3d}. It is
 * also intended to be an easy replacement for {@link DualPodIMUPoseEstimator},
 * only requiring the addition of a standard deviation for Z and appropriate
 * conversions between 2D and 3D versions of geometry classes. (See
 * {@link Pose3d#Pose3d(Pose2d)}, {@link Rotation3d#Rotation3d(Rotation2d)},
 * {@link Translation3d#Translation3d(Translation2d)}, and
 * {@link Pose3d#toPose2d()}.)
 *
 * <p>
 * {@link DualPodIMUPoseEstimator3d#update} should be called every robot loop.
 *
 * <p>
 * {@link DualPodIMUPoseEstimator3d#addVisionMeasurement} can be called as
 * infrequently as you want; if you never call it, then this class will behave
 * mostly like regular encoder odometry.
 */
public class DualPodIMUPoseEstimator3d extends PoseEstimator3d<DualPodIMUPositions> {
    /**
     * Constructs a DualPodIMUPoseEstimator3d with default standard deviations for
     * the model and vision measurements.
     *
     * <p>
     * The default standard deviations of the model states are 0.1 meters for x, 0.1
     * meters for y, 0.1 meters for z, and 0.1 radians for angle. The default
     * standard deviations of the vision measurements are 0.45 meters for x, 0.45
     * meters for y, 0.45 meters for z, and 0.45 radians for angle.
     *
     * @param kinematics        A correctly-configured kinematics object for your
     *                          odometry pod system.
     * @param gyroAngle         The current gyro angle.
     * @param wheelPositions    The distances driven by each wheel.
     * @param initialPoseMeters The starting pose estimate.
     */
    public DualPodIMUPoseEstimator3d(
            DualPodIMUKinematics kinematics,
            Rotation3d gyroAngle,
            DualPodIMUPositions wheelPositions,
            Pose3d initialPoseMeters) {
        this(
                kinematics,
                gyroAngle,
                wheelPositions,
                initialPoseMeters,
                VecBuilder.fill(0.1, 0.1, 0.1, 0.1),
                VecBuilder.fill(0.45, 0.45, 0.45, 0.45));
    }

    /**
     * Constructs a DualPodIMUPoseEstimator3d.
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
    public DualPodIMUPoseEstimator3d(
            DualPodIMUKinematics kinematics,
            Rotation3d gyroAngle,
            DualPodIMUPositions wheelPositions,
            Pose3d initialPoseMeters,
            Matrix<N4, N1> stateStdDevs,
            Matrix<N4, N1> visionMeasurementStdDevs) {
        super(
                kinematics,
                new DualPodIMUOdometry3d(kinematics, gyroAngle, wheelPositions, initialPoseMeters),
                stateStdDevs,
                visionMeasurementStdDevs);
    }
}
