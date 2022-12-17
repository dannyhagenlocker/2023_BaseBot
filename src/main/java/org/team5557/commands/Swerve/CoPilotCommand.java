package org.team5557.commands.Swerve;

import org.littletonrobotics.junction.Logger;
import org.team5557.Constants;
import org.team5557.Vision.GoalPlanner;
import org.team5557.Vision.Hotspot;
import org.team5557.Vision.VisionManager;
import org.team5557.subsystems.LEDs;
import org.team5557.subsystems.Swerve;
import org.team5557.subsystems.LEDs.State;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CoPilotCommand extends CommandBase{
    private final PPHolonomicDriveController controller;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    //Follow with Events
    /*
    private final List<PathPlannerTrajectory.EventMarker> pathMarkers;
    private final HashMap<String, Command> eventMap;
    private final HashMap<Command, Boolean> currentCommands = new HashMap<>();
    private final List<PathPlannerTrajectory.EventMarker> unpassedMarkers = new ArrayList<>();
    */

    private final Swerve swerve;
    private final GoalPlanner goalPlanner;
    private final VisionManager visionManager;
    private final PeriodicIO m_periodicIO;
    private LEDs leds;

    public CoPilotCommand(
        Swerve swerve,
        GoalPlanner goalPlanner,
        VisionManager visionManager,
        LEDs leds,
        DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier,
        //List<PathPlannerTrajectory.EventMarker> pathMarkers,
        //HashMap<String, Command> eventMap,
        Subsystem... requirements) {
      this.swerve = swerve;
      this.goalPlanner = goalPlanner;
      this.visionManager = visionManager;
      this.leds = leds;
      this.controller = swerve.getFollower();

      //this.pathMarkers = pathMarkers;
      //this.eventMap = eventMap;

      this.m_translationXSupplier = translationXSupplier;
      this.m_translationYSupplier = translationYSupplier;
      this.m_rotationSupplier = rotationSupplier;

      m_periodicIO = new PeriodicIO();

      m_periodicIO.active_hotspot = goalPlanner.getHotspot();
      controller.setTolerance(m_periodicIO.active_hotspot.getErrorRadius());

      addRequirements(requirements);
    }

    @Override
    public void execute() {
        readPeriodicInputs();
        writePeriodicOutputs();
        logData();
    }

    public synchronized void readPeriodicInputs() {
        //MAKE POSE SUPPLIER ADAPTABLE BASED ON WHETHER THE ASSOCIATED TARGET IS VISIBLE
        m_periodicIO.timestamp = Timer.getFPGATimestamp();
        m_periodicIO.current_pose = goalPlanner.getPose();
        m_periodicIO.hotspot_has_changed = m_periodicIO.active_hotspot != goalPlanner.getHotspot();
        if (m_periodicIO.hotspot_has_changed) {
            m_periodicIO.active_hotspot = goalPlanner.getHotspot();
            controller.setTolerance(m_periodicIO.active_hotspot.getErrorRadius());
        }
        m_periodicIO.robot_to_target = m_periodicIO.active_hotspot.getGoalPosition().getTranslation().minus(m_periodicIO.current_pose.getTranslation());

        //These really only need to be updated when regenerating a trajectory
        //Potentially put them in an if statement?
        m_periodicIO.field_relative_initial_velocity = new Translation2d(swerve.getCurrentVelocity().vxMetersPerSecond, swerve.getCurrentVelocity().vyMetersPerSecond);
        m_periodicIO.field_relative_initial_direction = m_periodicIO.field_relative_initial_velocity.getAngle();
        m_periodicIO.initial_speed = Math.max(0, m_periodicIO.field_relative_initial_velocity.getNorm());
        m_periodicIO.initial_direction_offset = m_periodicIO.robot_to_target.getAngle().minus(m_periodicIO.field_relative_initial_direction); //initial direction offset is the difference between the instantaneous direction of travel and the desired direction of travel
        m_periodicIO.initial_speed = m_periodicIO.initial_speed * m_periodicIO.initial_direction_offset.getCos();

        if(m_periodicIO.timestamp - m_periodicIO.regeneration_timestamp >= Constants.copilot.path_regeneration_time && 
            (m_periodicIO.robot_to_target.getNorm() > Constants.copilot.min_path_length || m_periodicIO.hotspot_has_changed || visionManager.associatedTargetVisible())) {

            m_periodicIO.active_trajectory = 
                PathPlanner.generatePath(
                    m_periodicIO.active_hotspot.getPathConstraints(), 
                    new PathPoint(
                        m_periodicIO.current_pose.getTranslation(),
                        m_periodicIO.robot_to_target.getAngle(),
                        m_periodicIO.current_pose.getRotation(),
                        m_periodicIO.initial_speed),
                    new PathPoint(
                        m_periodicIO.active_hotspot.getGoalPosition().getTranslation(),
                        m_periodicIO.robot_to_target.getAngle(),
                        m_periodicIO.active_hotspot.getGoalPosition().getRotation()) // position, heading
                );
            m_periodicIO.regeneration_timestamp = m_periodicIO.timestamp;
            swerve.getField().getObject("traj").setTrajectory(m_periodicIO.active_trajectory);
            PathPlannerServer.sendActivePath(m_periodicIO.active_trajectory.getStates());
        }

        if(controller.atReference()) {
            m_periodicIO.current_mode = CoPilotMode.AT_TARGET;
            m_periodicIO.target_chassis_speeds = new ChassisSpeeds();
            if(m_periodicIO.active_hotspot.getShouldX())
                m_periodicIO.should_X = true;
        } else if (m_periodicIO.robot_to_target.getNorm() <= m_periodicIO.active_hotspot.getMaxPathLength()) {
            m_periodicIO.current_mode = CoPilotMode.PATH_FOLLOWING;
            m_periodicIO.desired_state = (PathPlannerState) m_periodicIO.active_trajectory.sample(m_periodicIO.timestamp - m_periodicIO.regeneration_timestamp);
            m_periodicIO.target_chassis_speeds = controller.calculate(m_periodicIO.current_pose, m_periodicIO.desired_state);
            m_periodicIO.should_X = false;
        } else {
            m_periodicIO.current_mode = CoPilotMode.FREE_DRIVE;
            m_periodicIO.target_chassis_speeds = 
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_translationXSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                    m_translationYSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                    m_rotationSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                    swerve.getPose().getRotation());

            m_periodicIO.should_X = false;
        }
    }

    public synchronized void writePeriodicOutputs() {
        swerve.drive(m_periodicIO.target_chassis_speeds);
        swerve.setXED(m_periodicIO.should_X);

        switch (m_periodicIO.current_mode) {
            case FREE_DRIVE:
                leds.requestState(State.COPILOT_FREE_DRIVE);
                break;
            case AT_TARGET:
                leds.requestState(State.COPILOT_AT_TARGET);
                break;
            case PATH_FOLLOWING:
                PathPlannerServer.sendPathFollowingData(new Pose2d(m_periodicIO.desired_state.poseMeters.getTranslation(), m_periodicIO.desired_state.holonomicRotation), m_periodicIO.current_pose);
                leds.requestState(State.COPILOT_FOLLOWING);
                break;
        }
    }

    public void logData() {
        Logger.getInstance().recordOutput("Swerve/PP/DesiredState", m_periodicIO.desired_state.poseMeters);
        Logger.getInstance().recordOutput("Swerve/PP/CurrentMode", m_periodicIO.current_mode.toString());
        Logger.getInstance().recordOutput("Swerve/PP/RobotToDesiredState", m_periodicIO.robot_to_target);
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double regeneration_timestamp;
        public Pose2d current_pose;
        public Translation2d robot_to_target;
        public Hotspot active_hotspot;
        public boolean hotspot_has_changed;
        public PathPlannerState desired_state;
        public CoPilotMode current_mode;
  
        public Translation2d field_relative_initial_velocity;
        public Rotation2d field_relative_initial_direction;
        public double initial_speed;
        public Rotation2d initial_direction_offset;
  
        // OUTPUTS
        public PathPlannerTrajectory active_trajectory;
        public ChassisSpeeds target_chassis_speeds;
        public boolean should_X;
  
    }
    
    private enum CoPilotMode {
        PATH_FOLLOWING,
        AT_TARGET,
        FREE_DRIVE
    }
}
