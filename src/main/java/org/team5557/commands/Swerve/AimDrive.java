package org.team5557.commands.Swerve;

import java.util.function.DoubleSupplier;

import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.subsystems.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimDrive extends CommandBase {

    private final Swerve swerve;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier goalAngleSupplierRadians;

    public AimDrive(DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier goalAngleSupplierRadians) {
        this.swerve = RobotContainer.swerve;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.goalAngleSupplierRadians = goalAngleSupplierRadians;
        addRequirements(swerve);
    }

    public AimDrive(DoubleSupplier translationXSupplier, 
            DoubleSupplier translationYSupplier, double goalAngleRadians) {
        this(translationXSupplier, translationYSupplier, () -> goalAngleRadians);
    }

    @Override
    public void initialize() {
        RobotContainer.raw_controllers.resetTheta();
    }

    @Override
    public void execute() {
        double rotationalVelocity = RobotContainer.raw_controllers.calculateTheta(goalAngleSupplierRadians.getAsDouble()) * swerve.getMotorOutputLimiter();
        rotationalVelocity += Math.copySign(Swerve.ROTATIONAL_STATIC_CONSTANT / Swerve.MAX_VOLTAGE
                    * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, rotationalVelocity);

        swerve.drive(new ChassisSpeeds(
                m_translationXSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                m_translationYSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                rotationalVelocity), true, Constants.superstructure.center_of_rotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0), false, Constants.superstructure.center_of_rotation);
    }
}
