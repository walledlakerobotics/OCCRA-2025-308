// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * General utilities used throughout the code.
 */
public class Utils {
    private Utils() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Configures SysID for a <code>Subsystem</code> on a
     * <code>ShuffleboardLayout</code>.
     * 
     * @param layout         The layout to add the <code>Command</code> buttons to.
     * @param config         Configuration object for SysID
     * @param driveAtVoltage Drive the motors being tested at the given voltage.
     * @param subsystem      The <code>Subsystem</code> that controls the motors.
     * @see Subsystem
     * @see ShuffleboardLayout
     * @see Command
     */
    public static void configureSysID(ShuffleboardLayout layout, SysIdRoutine.Config config,
            Consumer<Voltage> driveAtVoltage, Subsystem subsystem, SparkMax... motors) {
        RelativeEncoder[] encoders = new RelativeEncoder[motors.length];

        MutVoltage[] voltages = new MutVoltage[motors.length];
        MutLinearVelocity[] velocities = new MutLinearVelocity[motors.length];
        MutDistance[] distances = new MutDistance[motors.length];

        for (int i = 0; i < motors.length; i++) {
            encoders[i] = motors[i].getEncoder();

            voltages[i] = Volts.mutable(0);
            velocities[i] = MetersPerSecond.mutable(0);
            distances[i] = Meters.mutable(0);
        }

        SysIdRoutine sysIdRoutine = new SysIdRoutine(config, new SysIdRoutine.Mechanism(driveAtVoltage, log -> {
            for (int i = 0; i < encoders.length; i++) {
                SparkMax motor = motors[i];
                RelativeEncoder encoder = encoders[i];

                log.motor("drive-motor-" + String.valueOf(i))
                        .voltage(voltages[i].mut_replace(motor.get() * RobotController.getBatteryVoltage(), Volts))
                        .linearVelocity(velocities[i].mut_replace(encoder.getVelocity(), MetersPerSecond))
                        .linearPosition(distances[i].mut_replace(encoder.getPosition(), Meters));

            }
        }, subsystem));

        Command quasistaticForward = sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
        Command quasistaticBackward = sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
        Command dynamicForward = sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
        Command dynamicBackward = sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);

        Command all = quasistaticForward.andThen(quasistaticBackward).andThen(dynamicForward).andThen(dynamicBackward)
                .withName("Run All");

        layout.add("Quasistatic Forward", sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        layout.add("Quasistatic Backward", sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        layout.add("Dynamic Forward", sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        layout.add("Dynamic Backward", sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        layout.add("Run All", all);
    }

    public static double roundToNearest(double value, int place) {
        double power = Math.pow(10, place);
        return Math.round(power * value) / power;
    }
}