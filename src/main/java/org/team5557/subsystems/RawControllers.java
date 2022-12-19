package org.team5557.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import org.team5557.Constants;
import org.team5557.RobotContainer;

public class RawControllers extends SubsystemBase {
    public ProfiledPIDController thetaController;
    public PIDController xController;
    public PIDController yController;
    public Rotation2d startAngle;
    private final Swerve swerve;

    /** Creates a new Trajectory. */
    public RawControllers() {
        thetaController = new ProfiledPIDController(
            Constants.follower.theta_kP, 
            Constants.follower.theta_kI, 
            Constants.follower.theta_kD, 
            new Constraints(Constants.follower.theta_kV, Constants.follower.theta_kA)
        );
        // Setup thetaController used for auton and automatic turns
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        xController = new PIDController(Constants.follower.translation_kP, Constants.follower.translation_kI, Constants.follower.translation_kD);
        yController = new PIDController(Constants.follower.translation_kP, Constants.follower.translation_kI, Constants.follower.translation_kD);

        this.swerve = RobotContainer.swerve;
    }

    public void resetTheta() {
        startAngle = swerve.getPose().getRotation();
        thetaController.reset(startAngle.getRadians(), swerve.getCurrentVelocity().omegaRadiansPerSecond);
    }

    public Rotation2d getStartAngle() {
        return startAngle;
    }

    public double calculateTheta(double goalAngleRadians) {
        return thetaController.calculate(swerve.getPose().getRotation().getRadians(), goalAngleRadians);
    }

    public DoubleSupplier calculateThetaSupplier(DoubleSupplier goalAngleSupplierRadians) {
        return () -> calculateTheta(goalAngleSupplierRadians.getAsDouble());
    }

    public DoubleSupplier calculateThetaSupplier(double goalAngle) {
        return calculateThetaSupplier(() -> goalAngle);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}