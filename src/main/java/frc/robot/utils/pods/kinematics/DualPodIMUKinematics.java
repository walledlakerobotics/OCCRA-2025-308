package frc.robot.utils.pods.kinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;

/**
 * Helper class that converts speeds from a dual pod (forward and lateral) +
 * IMU (gyro) system into chassis speeds (dx, dy, dtheta).
 */
public class DualPodIMUKinematics implements Kinematics<DualPodIMUSpeeds, DualPodIMUPositions> {
    private final Translation2d m_forwardPodMeters;
    private final Translation2d m_lateralPodMeters;

    /**
     * Constructs a dual pod IMU kinematics object.
     *
     * @param forwardPodMeters The location of the forward tracking (x-axis)
     *                         odometry pod relative to the physical center of the
     *                         robot.
     * @param lateralPodMeters The location of the lateral tracking (y-axis)
     *                         odometry pod relative to the physical center of the
     *                         robot.
     */
    public DualPodIMUKinematics(Translation2d forwardPodMeters, Translation2d lateralPodMeters) {
        m_forwardPodMeters = forwardPodMeters;
        m_lateralPodMeters = lateralPodMeters;
    }

    public ChassisSpeeds toChassisSpeeds(DualPodIMUSpeeds wheelSpeeds) {
        double forward = wheelSpeeds.forwardMetersPerSecond;
        double lateral = wheelSpeeds.lateralMetersPerSecond;

        forward -= wheelSpeeds.omegaRadiansPerSecond * m_forwardPodMeters.getY();
        lateral -= wheelSpeeds.omegaRadiansPerSecond * m_lateralPodMeters.getX();

        return new ChassisSpeeds(forward, lateral, wheelSpeeds.omegaRadiansPerSecond);
    }

    public DualPodIMUSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        double forward = chassisSpeeds.vxMetersPerSecond;
        double lateral = chassisSpeeds.vyMetersPerSecond;

        forward += chassisSpeeds.omegaRadiansPerSecond * m_forwardPodMeters.getY();
        lateral += chassisSpeeds.omegaRadiansPerSecond * m_lateralPodMeters.getX();

        return new DualPodIMUSpeeds(forward, lateral, chassisSpeeds.omegaRadiansPerSecond);
    }

    public Twist2d toTwist2d(DualPodIMUPositions start, DualPodIMUPositions end) {
        double dx = end.forwardMeters - start.forwardMeters;
        double dy = end.lateralMeters - start.lateralMeters;
        double dtheta = end.thetaRadians - start.thetaRadians;

        dx -= dtheta * m_forwardPodMeters.getY();
        dy -= dtheta * m_lateralPodMeters.getX();

        return new Twist2d(dx, dy, dtheta);
    }

    @Override
    public DualPodIMUPositions interpolate(DualPodIMUPositions startValue, DualPodIMUPositions endValue,
            double t) {
        return startValue.interpolate(endValue, t);
    }

    public DualPodIMUPositions copy(DualPodIMUPositions positions) {
        return new DualPodIMUPositions(
                positions.forwardMeters,
                positions.lateralMeters,
                positions.thetaRadians);
    }

    public void copyInto(DualPodIMUPositions positions, DualPodIMUPositions output) {
        output.forwardMeters = positions.forwardMeters;
        output.lateralMeters = positions.lateralMeters;
        output.thetaRadians = positions.thetaRadians;
    }

    /**
     * Returns the forward (x-axis) pod translation.
     *
     * @return The forward pod translation.
     */
    public Translation2d getForward() {
        return m_forwardPodMeters;
    }

    /**
     * Returns the lateral (y-axis) pod translation.
     *
     * @return The lateral pod translation.
     */
    public Translation2d getLateral() {
        return m_lateralPodMeters;
    }
}
