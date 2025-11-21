// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pods.kinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.Odometry3d;

/**
 * Class for a dual pod (forward and lateral) + IMU odometry system. Odometry
 * allows you to track the robot's position on the field over a course of a
 * match using readings from your odometry pod encoders.
 *
 * <p>
 * This class is meant to be an easy replacement for {@link DualPodIMUOdometry},
 * only requiring the addition of appropriate conversions between 2D and 3D
 * versions of geometry classes. (See {@link Pose3d#Pose3d(Pose2d)},
 * {@link Rotation3d#Rotation3d(Rotation2d)},
 * {@link Translation3d#Translation3d(Translation2d)}, and
 * {@link Pose3d#toPose2d()}.)
 *
 * <p>
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
public class DualPodIMUOdometry3d extends Odometry3d<DualPodIMUPositions> {
    /**
     * Constructs a DualPodIMUOdometry3d object.
     *
     * @param kinematics        The kinematics of the odometry pod system.
     * @param gyroAngle         The angle reported by the gyroscope.
     * @param wheelPositions    The current encoder readings.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public DualPodIMUOdometry3d(DualPodIMUKinematics kinematics, Rotation3d gyroAngle,
            DualPodIMUPositions wheelPositions, Pose3d initialPoseMeters) {
        super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
    }

    /**
     * Constructs a DualPodIMUOdometry3d object with the default pose at the origin.
     *
     * @param kinematics     The kinematics of the odometry pod system.
     * @param gyroAngle      The angle reported by the gyroscope.
     * @param wheelPositions The current encoder readings.
     */
    public DualPodIMUOdometry3d(DualPodIMUKinematics kinematics, Rotation3d gyroAngle,
            DualPodIMUPositions wheelPositions) {
        super(kinematics, gyroAngle, wheelPositions, Pose3d.kZero);
    }
}
