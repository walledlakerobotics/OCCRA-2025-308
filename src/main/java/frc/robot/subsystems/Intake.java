package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/*
 *  A subsystem that controls the intake motor's
 */
public class Intake extends SubsystemBase {


    private SparkMax m_leaderMotor, m_followerMotor;

    public Intake() {
        m_leaderMotor = new SparkMax(IntakeConstants.kIntakeLeaderMotorId, MotorType.kBrushless);
        m_followerMotor = new SparkMax(IntakeConstants.kIntakeFollowerMotorId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        
        config
                .inverted(IntakeConstants.kLeaderMotorInverted)
                .idleMode(IntakeConstants.kIdleMode)
                .smartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

        m_leaderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config
                .inverted(IntakeConstants.kFollowerMotorInverted)
                .follow(m_leaderMotor);
        
        m_followerMotor.configure(config, null, null);
    }

    public void setSpeed(double speed) {
        m_leaderMotor.set(speed);
    }

    public Command goToVelocity(double speed) {
        return runOnce(() -> setSpeed(speed));
    }
}
