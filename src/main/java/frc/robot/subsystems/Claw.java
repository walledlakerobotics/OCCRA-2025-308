package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ClawConstants;

/*
 *  A subsystem that controls the intake motor's
 */
public class Claw extends SubsystemBase {

    private SparkMax m_clawMotor = new SparkMax(ClawConstants.kClawMotorId, MotorType.kBrushed);
    private SparkLimitSwitch m_forwardSwitch, m_reverseSwitch;

    /*
     * Constructs intake
     */
    public Claw() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
                .smartCurrentLimit(ClawConstants.kSmartCurrentLimit)
                .idleMode(ClawConstants.kIdleMode)
                .inverted(ClawConstants.kLeaderMotorInverted);

        m_clawMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_forwardSwitch = m_clawMotor.getForwardLimitSwitch();
        m_reverseSwitch = m_clawMotor.getReverseLimitSwitch();

        // put drive motors into coast mode when disabled
        RobotModeTriggers.disabled().and(() -> !DriverStation.isFMSAttached())
                .onTrue(setIdleMode(IdleMode.kCoast))
                .onFalse(setIdleMode(IdleMode.kBrake));
    }

    /**
     * Sets the speed of the two motors.
     * 
     * @param speed The speed to set.
     */
    public void setSpeed(double speed) {
        m_clawMotor.set(speed);
    }

    /**
     * Creates a {@link Command} that sets the {@link IdleMode} for the claw motor.
     * This will not persist through power cycles.
     * 
     * @param mode The idle mode to set.
     * @return The command.
     */
    private Command setIdleMode(IdleMode mode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(mode);

        return Commands.runOnce(() -> {
            m_clawMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }).ignoringDisable(true);
    }

    /**
     * Returns a command to open.
     * 
     * @return
     */
    public Command open() {
        return runOnce(() -> setSpeed(ClawConstants.kClawSpeed))
                .andThen(Commands.waitUntil(this::isClawOpen))
                .finallyDo(() -> setSpeed(0));
    }

    /**
     * Returns a command to close.
     * 
     * @return
     */
    public Command close() {
        return runOnce(() -> setSpeed(-ClawConstants.kClawSpeed))
                .andThen(Commands.waitUntil(this::isClawClosed))
                .finallyDo(() -> setSpeed(0));
    }

    public Command goToVelocity(double velocity) {
        return runOnce(() -> setSpeed(velocity));
    }

    public boolean isClawClosed() {
        return m_reverseSwitch.isPressed();
    }

    public boolean isClawOpen() {
        return m_forwardSwitch.isPressed();
    }
}
