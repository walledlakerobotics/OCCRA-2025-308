package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicBoolean;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/*
 *  A subsystem that controls the robot's cube intake
 */
public class Intake extends SubsystemBase {
    private final SparkMax m_leftMotor = new SparkMax(IntakeConstants.kIntakeLeftMotorId, MotorType.kBrushless);
    private final SparkMax m_rightMotor = new SparkMax(IntakeConstants.kIntakeRightMotorId, MotorType.kBrushless);

    public Intake() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
                .idleMode(IntakeConstants.kIdleMode)
                .smartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

        config.inverted(IntakeConstants.kLeftMotorInverted);
        m_leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.inverted(IntakeConstants.kRightMotorInverted);
        m_rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command intake() {
        AtomicBoolean gotCube = new AtomicBoolean(false);

        return runOnce(() -> {
            gotCube.set(false);

            m_leftMotor.set(IntakeConstants.kIntakeSpeed);
            m_rightMotor.set(IntakeConstants.kIntakeSpeed);
        })
                .andThen(Commands.idle(this))
                .finallyDo(() -> {
                    m_leftMotor.set(0);
                    m_rightMotor.set(0);
                });
    }

    public Command shootRight() {
        return runOnce(() -> {
            m_leftMotor.set(0);
            m_rightMotor.set(-IntakeConstants.kShootSpeed);
        })
                .andThen(Commands.idle(this))
                .finallyDo(() -> {
                    m_leftMotor.set(0);
                    m_rightMotor.set(0);
                });
    }

    public Command shootLeft() {
        return runOnce(() -> {
            m_leftMotor.set(-IntakeConstants.kShootSpeed);
            m_rightMotor.set(0);
        })
                .andThen(Commands.idle(this))
                .finallyDo(() -> {
                    m_leftMotor.set(0);
                    m_rightMotor.set(0);
                });
    }

    public Command shootStraight() {
        return runOnce(() -> {
            m_leftMotor.set(-IntakeConstants.kShootSpeed);
            m_rightMotor.set(-IntakeConstants.kShootSpeed);
        })
                .andThen(Commands.idle(this))
                .finallyDo(() -> {
                    m_leftMotor.set(0);
                    m_rightMotor.set(0);
                });
    }
}
