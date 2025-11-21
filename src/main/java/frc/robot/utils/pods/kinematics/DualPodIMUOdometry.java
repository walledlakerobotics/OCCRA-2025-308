package frc.robot.utils.pods.kinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;

/**
 * Class for a dual pod (forward and lateral) + IMU odometry system. Odometry
 * allows you to track the robot's position on the field over a course of a
 * match using readings from your odometry pod encoders.
 * 
 * <p>
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
public class DualPodIMUOdometry extends Odometry<DualPodIMUPositions> {
    /**
     * Constructs a DualPodIMUOdometry object.
     * 
     * @param kinematics        The kinematics of the odometry pod system.
     * @param gyroAngle         The angle reported by the gyroscope.
     * @param wheelPositions    The current encoder readings.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public DualPodIMUOdometry(DualPodIMUKinematics kinematics, Rotation2d gyroAngle, DualPodIMUPositions wheelPositions,
            Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
    }

    /**
     * Constructs a DualPodIMUOdometry object with the default pose at the origin.
     * 
     * @param kinematics     The kinematics of the odometry pod system.
     * @param gyroAngle      The angle reported by the gyroscope.
     * @param wheelPositions The current encoder readings.
     */
    public DualPodIMUOdometry(DualPodIMUKinematics kinematics, Rotation2d gyroAngle,
            DualPodIMUPositions wheelPositions) {
        super(kinematics, gyroAngle, wheelPositions, Pose2d.kZero);
    }
}
