package org.team5557.commands.Swerve;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.pilot.PilotConfig;
import frc.robot.swerve.commands.SwerveDrive;
import frc.robot.trajectories.TrajectoriesCommands;
import java.util.function.DoubleSupplier;

import org.team5557.RobotContainer;
import org.team5557.subsystems.Swerve;

/** Should contain commands used only by the pilot controller and to rumble pilot command */
public class PilotCommands {
    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
    }

    /** Field Oriented Drive */
    public static Command pilotSwerve() {
        return new TeleopDrive(
                        () -> Robot.pilotGamepad.getDriveFwdPositive(),
                        () -> Robot.pilotGamepad.getDriveLeftPositive(),
                        () -> Robot.pilotGamepad.getDriveCCWPositive())
                .withName("PilotSwerve");
    }

    /** Robot Oriented Drive */
    public static Command fpvPilotSwerve() {
        return new SwerveDrive(
                        () -> Robot.pilotGamepad.getDriveFwdPositive(),
                        () -> Robot.pilotGamepad.getDriveLeftPositive(),
                        () -> Robot.pilotGamepad.getDriveCCWPositive(),
                        false)
                .withName("fpvPilotSwerve");
    }

    public static Command snakeDrive() {
        return TrajectoriesCommands.resetThetaController()
                .andThen(
                        new SwerveDrive(
                                () -> Robot.pilotGamepad.getDriveFwdPositive(),
                                () -> Robot.pilotGamepad.getDriveLeftPositive(),
                                Robot.trajectories.calculateThetaSupplier(
                                        () -> Robot.pilotGamepad.getDriveAngle()),
                                true,
                                false,
                                PilotConfig.intakeCoRmeters))
                .withName("SnakeDrive");
    }

    /**
     * Drive the robot and control orientation using the right stick
     *
     * @return
     */
    public static Command stickSteer() {
        return aimPilotDrive(() -> Robot.pilotGamepad.getRightStickAngle()).withName("StickSteer");
    }

    /** Drive while aiming to a specific angle, uses theta controller from Trajectories */
    public static Command aimPilotDrive(double goalAngleRadians) {
        return aimPilotDrive(() -> goalAngleRadians);
    }

    /** Reset the Theata Controller and then run the SwerveDrive command and pass a goal Supplier */
    public static Command aimPilotDrive(DoubleSupplier goalAngleSupplierRadians) {
        return TrajectoriesCommands.resetThetaController()
                .andThen(
                        new SwerveDrive(
                                () -> Robot.pilotGamepad.getDriveFwdPositive(),
                                () -> Robot.pilotGamepad.getDriveLeftPositive(),
                                Robot.trajectories.calculateThetaSupplier(goalAngleSupplierRadians),
                                true,
                                false))
                .withName("AimPilotDrive");
    }
}