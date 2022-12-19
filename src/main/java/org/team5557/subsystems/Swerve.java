package org.team5557.subsystems;

import java.util.Map;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;
import org.team5557.Constants;

public class Swerve extends SubsystemBase {
    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5670.0 / 60.0
            * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
            * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(Constants.superstructure.trackwidth / 2.0, Constants.superstructure.drivebase / 2.0);

    public static final double ROTATIONAL_STATIC_CONSTANT = 0.3;

    public static final double DRIVETRAIN_CURRENT_LIMIT = 50.0;

    public final PIDController xController = new PIDController(5.0, 0.0, 0.0);
    public final PIDController yController = new PIDController(5.0, 0.0, 0.0);
    public final PIDController rotationController = new PIDController(5.0, 0.0, 0.0);
    private final PPHolonomicDriveController follower = new PPHolonomicDriveController(xController, yController, rotationController);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.superstructure.trackwidth / 2.0, Constants.superstructure.drivebase / 2.0),
            // Front right
            new Translation2d(Constants.superstructure.trackwidth / 2.0, -Constants.superstructure.drivebase / 2.0),
            // Back left
            new Translation2d(-Constants.superstructure.trackwidth / 2.0, Constants.superstructure.drivebase / 2.0),
            // Back right
            new Translation2d(-Constants.superstructure.trackwidth / 2.0, -Constants.superstructure.drivebase / 2.0));

    private final SwerveDrivePoseEstimator<N7, N7, N5> estimator;

    private final Pigeon2 pigeon = new Pigeon2(Constants.ports.pigeon);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds currentVelocity = new ChassisSpeeds();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private Translation2d centerOfRotation = new Translation2d();

    private final GenericEntry motorOutputPercentageLimiterEntry;
    private final Field2d field = new Field2d();

    private double motorOutputLimiter;
    private boolean isXED;

    public Swerve() {
        Mk4ModuleConfiguration mk4ModuleConfiguration = new Mk4ModuleConfiguration();
        mk4ModuleConfiguration.setDriveCurrentLimit(DRIVETRAIN_CURRENT_LIMIT);
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.swerve_readout_key);
        
        frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                mk4ModuleConfiguration, 
                Mk4iSwerveModuleHelper.GearRatio.L2, 
                Constants.fl_module.drive_motor,
                Constants.fl_module.steer_motor, 
                Constants.fl_module.steer_encoder, 
                Constants.fl_module.encoder_offset
        );

        frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                mk4ModuleConfiguration, 
                Mk4iSwerveModuleHelper.GearRatio.L2, 
                Constants.fr_module.drive_motor,
                Constants.fr_module.steer_motor, 
                Constants.fr_module.steer_encoder, 
                Constants.fr_module.encoder_offset
        );

        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                mk4ModuleConfiguration, 
                Mk4iSwerveModuleHelper.GearRatio.L2, 
                Constants.bl_module.drive_motor,
                Constants.bl_module.steer_motor, 
                Constants.bl_module.steer_encoder, 
                Constants.bl_module.encoder_offset
        );

        backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                mk4ModuleConfiguration, 
                Mk4iSwerveModuleHelper.GearRatio.L2, 
                Constants.br_module.drive_motor,
                Constants.br_module.steer_motor, 
                Constants.br_module.steer_encoder, 
                Constants.br_module.encoder_offset
        );

        estimator = new SwerveDrivePoseEstimator<N7, N7, N5>(Nat.N7(), Nat.N7(), Nat.N5(),
                getGyroscopeRotation(),
                getModulePositions(),
                new Pose2d(),
                kinematics,
                Constants.estimator.stateStdDevs, // estimator values (x, y, rotation) std-devs
                Constants.estimator.gyroStdDevs, // Gyroscope rotation std-dev
                Constants.estimator.normalVisionStdDevs); // Vision (x, y, rotation) std-devs*/

        motorOutputPercentageLimiterEntry = tab.add("Motor Percentage", 100.0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 100.0, "Block increment", 10.0)).withPosition(0, 3)
                .getEntry();

        tab.addNumber("Odometry X", () -> Units.metersToFeet(getPose().getX()));
        tab.addNumber("Odometry Y", () -> Units.metersToFeet(getPose().getY()));
        tab.addNumber("Odometry Angle", () -> getPose().getRotation().getDegrees());
        tab.addNumber("Velocity X", () -> Units.metersToFeet(getCurrentVelocity().vxMetersPerSecond));
        tab.addNumber("Velocity Y", () -> Units.metersToFeet(getCurrentVelocity().vyMetersPerSecond));
        //SmartDashboard.putData("PPSwerveControllerCommand_field", field);
        tab.add("Field", field);
        /*tab.addNumber("Trajectory Position X", () -> {
            var lastState = follower.getLastState();
            if (lastState == null)
                return 0;

            return Units.metersToFeet(lastState.getPathState().getPosition().x);
        });
        tab.addNumber("Trajectory Velocity X", () -> {
            var lastState = follower.getLastState();
            if (lastState == null)
                return 0;

            return Units.metersToFeet(lastState.getVelocity());
        });
        */
        tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
        pigeon.configFactoryDefault();

        new Trigger(RobotState::isEnabled).onTrue(new StartEndCommand(() -> {
                ((CANSparkMax)frontLeftModule.getSteerMotor()).setIdleMode(IdleMode.kBrake);
                ((CANSparkMax)frontRightModule.getSteerMotor()).setIdleMode(IdleMode.kBrake);
                ((CANSparkMax)backLeftModule.getSteerMotor()).setIdleMode(IdleMode.kBrake);
                ((CANSparkMax)backRightModule.getSteerMotor()).setIdleMode(IdleMode.kBrake);

                ((CANSparkMax)frontLeftModule.getDriveMotor()).setIdleMode(IdleMode.kBrake);
                ((CANSparkMax)frontRightModule.getDriveMotor()).setIdleMode(IdleMode.kBrake);
                ((CANSparkMax)backLeftModule.getDriveMotor()).setIdleMode(IdleMode.kBrake);
                ((CANSparkMax)backRightModule.getDriveMotor()).setIdleMode(IdleMode.kBrake);
              }, () -> {
                ((CANSparkMax)frontLeftModule.getSteerMotor()).setIdleMode(IdleMode.kCoast);
                ((CANSparkMax)frontRightModule.getSteerMotor()).setIdleMode(IdleMode.kCoast);
                ((CANSparkMax)backLeftModule.getSteerMotor()).setIdleMode(IdleMode.kCoast);
                ((CANSparkMax)backRightModule.getSteerMotor()).setIdleMode(IdleMode.kCoast);

                ((CANSparkMax)frontLeftModule.getDriveMotor()).setIdleMode(IdleMode.kCoast);
                ((CANSparkMax)frontRightModule.getDriveMotor()).setIdleMode(IdleMode.kCoast);
                ((CANSparkMax)backLeftModule.getDriveMotor()).setIdleMode(IdleMode.kCoast);
                ((CANSparkMax)backRightModule.getDriveMotor()).setIdleMode(IdleMode.kCoast);
                }));
        }

    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    /**
     * Resets the rotation of the drivetrain to zero.
     */
    public void zeroRotation() {
        estimator.resetPosition(
                getGyroscopeRotation(), 
                getModulePositions(), 
                new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()));
    }

    /**
     * Returns the position of the robot
     */
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public double getMotorOutputLimiter() {
        return motorOutputLimiter;
    }

    public PPHolonomicDriveController getFollower() {
        return follower;
    }
    
    public SwerveDrivePoseEstimator<N7, N7, N5> getEstimator() {
        return estimator;
    }

    public ChassisSpeeds getCurrentVelocity() {
        return currentVelocity;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swerveModulePositions = {frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()};
        return swerveModulePositions;
    }

    /**
     * Returns whether the drivetrain is in X-OUT stance
     * @return boolean - isXED
     */
    public boolean isXED() {
        return isXED;
    }

    /**
     * Sets the X-OUT state of the the swerve
     * @param shouldX
     */
    public void setXED(boolean shouldX) {
        isXED = shouldX;
    }

    public Field2d getField() {
        return field;
    }

    /**
     * Sets the position of the robot to the position passed in with the current
     * gyroscope rotation.
     */
    public void setPose(Pose2d pose) {
        estimator.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
    }

    /**
     * Sets the desired chassis speed of the drivetrain.
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
        if (fieldRelative) {
            this.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, getPose().getRotation());
        } else {
            this.chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
        }
    }

    public void periodic() {
        SwerveModuleState currentFrontLeftModuleState = new SwerveModuleState(frontLeftModule.getDriveVelocity(),
                new Rotation2d(frontLeftModule.getSteerAngle()));
        SwerveModuleState currentFrontRightModuleState = new SwerveModuleState(frontRightModule.getDriveVelocity(),
                new Rotation2d(frontRightModule.getSteerAngle()));
        SwerveModuleState currentBackLeftModuleState = new SwerveModuleState(backLeftModule.getDriveVelocity(),
                new Rotation2d(backLeftModule.getSteerAngle()));
        SwerveModuleState currentBackRightModuleState = new SwerveModuleState(backRightModule.getDriveVelocity(),
                new Rotation2d(backRightModule.getSteerAngle()));
        SwerveModuleState[] swerveModuleStates = {currentFrontLeftModuleState, currentFrontRightModuleState, currentBackLeftModuleState, currentBackRightModuleState};

        currentVelocity = kinematics.toChassisSpeeds(currentFrontLeftModuleState, currentFrontRightModuleState,
                currentBackLeftModuleState, currentBackRightModuleState);

        estimator.update(getGyroscopeRotation(), swerveModuleStates, getModulePositions());

        motorOutputLimiter = motorOutputPercentageLimiterEntry.getDouble(0.0) / 100;

        if (!isXED) {
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[0].angle.getRadians());
                frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[1].angle.getRadians());
                backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[2].angle.getRadians());
                backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[3].angle.getRadians());
        } else {
                frontLeftModule.set(0.0, Units.degreesToRadians(-45.0));
                frontRightModule.set(0.0, Units.degreesToRadians(45.0));
                backLeftModule.set(0.0, Units.degreesToRadians(45.0));
                backRightModule.set(0.0, Units.degreesToRadians(-45.0));
        }

        Logger.getInstance().recordOutput("Swerve/Estimator/Rotation", estimator.getEstimatedPosition().getRotation().getDegrees());
        Logger.getInstance().recordOutput("Swerve/Estimator/XPos", estimator.getEstimatedPosition().getX());
        Logger.getInstance().recordOutput("Swerve/Estimator/YPos", estimator.getEstimatedPosition().getY());
        Logger.getInstance().recordOutput("Swerve/Follower/AtRefference", follower.atReference());
        Logger.getInstance().recordOutput("Swerve/X-Out", isXED());

        isXED = false;
    }
}