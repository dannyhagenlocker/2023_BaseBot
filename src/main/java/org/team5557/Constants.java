// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;

public final class Constants {

    private static final Mode robot_mode = Mode.REAL;
    public static final double kloop_period = 0.02;
    public static final int klong_CAN_TimeoutMs = 100;
    public static final int kCAN_TimeoutMs = 10;
    public static final boolean tuning_mode = false;

    public static final SwerveModuleConstants fl_module = new SwerveModuleConstants(); 
    static {
        fl_module.drive_motor = 1;
        fl_module.steer_motor = 2;
        fl_module.steer_encoder = 3;
        fl_module.encoder_offset = 0.0;
    }

    public static final SwerveModuleConstants fr_module = new SwerveModuleConstants(); 
    static {
        fr_module.drive_motor = 4;
        fr_module.steer_motor = 5;
        fr_module.steer_encoder = 6;
        fr_module.encoder_offset = 0.0;
    }

    public static final SwerveModuleConstants bl_module = new SwerveModuleConstants(); 
    static {
        bl_module.drive_motor = 7;
        bl_module.steer_motor = 8;
        bl_module.steer_encoder = 9;
        bl_module.encoder_offset = 0.0;
    }

    public static final SwerveModuleConstants br_module = new SwerveModuleConstants(); 
    static {
        br_module.drive_motor = 10;
        br_module.steer_motor = 11;
        br_module.steer_encoder = 12;
    }

    /**
     * This is where the constants for zeroing swerve are held
     */
    static {
        fl_module.encoder_offset = 0.0;
        fr_module.encoder_offset = 0.0;
        bl_module.encoder_offset = 0.0;
        br_module.encoder_offset = 0.0;
    }

    public static final CoPilotConstants copilot = new CoPilotConstants();
    static {
        copilot.path_regeneration_time = 1.0;
        copilot.max_path_length = Units.inchesToMeters(100.0);
        copilot.min_path_length = Units.inchesToMeters(12.0);
        copilot.error_radius = new Pose2d(Units.inchesToMeters(4), Units.inchesToMeters(4), Rotation2d.fromDegrees(3));
    }

    public static final SuperstructureConstants superstructure = new SuperstructureConstants();
    static {
        superstructure.trackwidth = 24.0;
        superstructure.drivebase = 24.0;
    }

    public static final PortConstants ports = new PortConstants();
    static {
        ports.primary_controller = 0;
        ports.candle = 13;
        ports.pigeon = 14;

        ports.underglow_start_index = 0;
        ports.underglow_end_index = 67;
        ports.underglow_num_leds = ports.underglow_end_index - ports.underglow_start_index;
    }

    public static final PathPlannerConstants pathplanner = new PathPlannerConstants();
    static {
        pathplanner.fast_constraints = new PathConstraints(4.4, 1.25);
        pathplanner.medium_constraints = new PathConstraints(3.0, 1);
        pathplanner.slow_constraints = new PathConstraints(2.0, 1.0);
        pathplanner.hellaslow_constraints = new PathConstraints(1.0, 1.0);
    }

    public static final ShuffleboardConstants shuffleboard = new ShuffleboardConstants();
    static {
        shuffleboard.driver_readout_key = "Driver";
        shuffleboard.swerve_readout_key = "Swerve";
        shuffleboard.tunable_readout_key = "Tunable";
        shuffleboard.vision_readout_key = "Vision";
    }

    public static final PoseEstimatorConstants estimator = new PoseEstimatorConstants();
    static {
        estimator.max_high_accuracy_distance = Units.inchesToMeters(36.0);

        estimator.highAccuracyVisionStdDevs = VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1));
        estimator.normalVisionStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
        estimator.gyroStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
        estimator.stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);
    }

    public static class CoPilotConstants {
        public double path_regeneration_time;
        public double max_path_length;
        public double min_path_length;
        public Pose2d error_radius;
    }

    public static class SuperstructureConstants {
        public double trackwidth;
        public double drivebase;
    }

    public static class PathPlannerConstants {
        public PathConstraints fast_constraints;
        public PathConstraints medium_constraints;
        public PathConstraints slow_constraints;
        public PathConstraints hellaslow_constraints;
    }

    public static class PortConstants {
        public int primary_controller;
        public int pigeon;
        public int candle;

        public int underglow_start_index;
        public int underglow_end_index;
        public int underglow_num_leds;
    }

    public static class PoseEstimatorConstants {
        public Vector<N7> stateStdDevs;
        public Vector<N5> gyroStdDevs;
        public Vector<N3> normalVisionStdDevs;
        public Vector<N3> highAccuracyVisionStdDevs;

        public double max_high_accuracy_distance;
    }

    public static class ShuffleboardConstants {
        public String driver_readout_key;
        public String swerve_readout_key;
        public String tunable_readout_key;
        public String vision_readout_key;
    }

    public static class SwerveModuleConstants {
        public int drive_motor;
        public int steer_motor;
        public int steer_encoder;
        public double encoder_offset;
    }

    public static Mode getRobotMode() {
        return robot_mode;
    }

    public static enum Mode {
        REAL, REPLAY
      }

}