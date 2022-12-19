package org.team5557.auto;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team5557.RobotContainer;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    //private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
    private final LoggedDashboardChooser<AutonomousMode> autonomousModeChooser = new LoggedDashboardChooser<>("Auto/Chooser");

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.addDefaultOption("Five Ball (Orange)", AutonomousMode.CIRCLE_AUTO);
        autonomousModeChooser.addOption("Test Auto", AutonomousMode.TEST_AUTO);
    }

    public LoggedDashboardChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getTestAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTestAutoPartOne());

        command.addCommands(follow(container, trajectories.getTestAutoPartOne()));

        return command;
    }

    private Command follow(RobotContainer container, PathPlannerTrajectory trajectory) {
        //return new FollowTrajectoryCommand(container.getSwerve(), trajectory);
        return new PPSwerveControllerCommand(trajectory, null, null, null, null, null, null, null);
    }

    public void resetRobotPose(SequentialCommandGroup command, RobotContainer container, PathPlannerTrajectory trajectory) {
        Pose2d start = trajectory.getInitialPose();
        command.addCommands(new InstantCommand(() -> container.getSwerve().setPose(new Pose2d(start.getX(),
                start.getY(), start.getRotation()))));
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.get()) {
            case TEST_AUTO :
                return getTestAuto(container);
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        RANDOM_THING,
        TEST_AUTO,
        CIRCLE_AUTO
    }
}