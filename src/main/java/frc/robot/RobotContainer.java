// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystem defined here
    private final DriveTrain m_driveTrain = new DriveTrain();
    private final Elevator m_elevator = new Elevator();
    private final Arm m_arm = new Arm();
    private final Intake m_intake = new Intake();

    // controllers
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_coDriverController = new CommandXboxController(
            OperatorConstants.kCoDriverControllerPort);

    private SendableChooser<Command> m_autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        configureNamedCommands();

        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.addOption("Right", new PathPlannerAuto("Left", true));

        Shuffleboard.getTab("Autonomous").add("Auto", m_autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        m_driveTrain.setDefaultCommand(m_driveTrain.drive(m_driverController::getLeftY, m_driverController::getLeftX,
                m_driverController::getRightX, () -> !m_driverController.getHID().getLeftBumperButton()));

        m_driverController.a().onTrue(m_driveTrain.resetFieldRelative());

        m_coDriverController.povUp().onTrue(m_elevator.goToVelocity(ElevatorConstants.kElevatorManualSpeed))
                .onFalse(m_elevator.goToVelocity(0));
        m_coDriverController.povDown().onTrue(m_elevator.goToVelocity(-ElevatorConstants.kElevatorManualSpeed))
                .onFalse(m_elevator.goToVelocity(0));

        m_coDriverController.leftBumper().whileTrue(m_intake.shootStraight());

        m_coDriverController.rightBumper().whileTrue(m_intake.intake());

        m_coDriverController.leftTrigger().whileTrue(m_intake.shootLeft());

        m_coDriverController.rightTrigger().whileTrue(m_intake.shootRight());
    }

    /**
     * Configures PathPlanner named commands
     */
    private void configureNamedCommands() {
        NamedCommands.registerCommand("Zero Arm", m_arm.goToAngle(Rotation2d.kZero, true));
        NamedCommands.registerCommand("Elevator L1", m_elevator.goToHeight(35, true));
        NamedCommands.registerCommand("Elevator Zero", m_elevator.goToHeight(0, true));
        NamedCommands.registerCommand("Reset Elevator", m_elevator.zeroEncoder());

        NamedCommands.registerCommand("Shoot", m_intake.shootStraight().withTimeout(1.0));

        NamedCommands.registerCommand("Intake", m_intake.intakeHold().withTimeout(1.0));
    }

    /**
     * Gets the {@link Command} to run in autonomous.
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
