package org.team5557.vision;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team5557.Constants;
import org.team5557.DriverReadout;
import org.team5557.Robot;
import org.team5557.RobotContainer;
import org.team5557.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class GoalPlanner {
    private final VisionManager vm;
    private final Swerve swerve;
    private final LoggedDashboardChooser<Hotspot> hotspotChooser = new LoggedDashboardChooser<>("GoalPlanner/Chooser");

    //private final Hotspot ZeroZero = new Hotspot();
    private final Hotspot ZeroZero = new Hotspot(
        VisionManager.zero_zero.pose.toPose2d().transformBy(new Transform2d(new Translation2d(0.75, 0), new Rotation2d(Math.PI))), 
        VisionManager.zero_zero, 
        new PathConstraints(2,2), 
        true);

    private final Hotspot FiveZero = new Hotspot(
        VisionManager.five_zero.pose.toPose2d().transformBy(new Transform2d(new Translation2d(0.75, 0), new Rotation2d(Math.PI))), 
        VisionManager.five_zero, 
        new PathConstraints(2,2), 
        true);
    
    private final Hotspot FiveFive = new Hotspot(
        VisionManager.five_five.pose.toPose2d().transformBy(new Transform2d(new Translation2d(0.75, 0), new Rotation2d(Math.PI))), 
        VisionManager.five_five, 
        new PathConstraints(2,2), 
        false);

    public GoalPlanner() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.driver_readout_key);
        this.swerve = RobotContainer.swerve;
        this.vm = RobotContainer.vision_manager;

        hotspotChooser.addOption("Zero Zero Zero", ZeroZero);
        hotspotChooser.addOption("Five Zero", FiveZero);
        hotspotChooser.addOption("Five Five", FiveFive);

        tab.add((Sendable) hotspotChooser);
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public Hotspot getHotspot() {
        return hotspotChooser.get();
    }
}
