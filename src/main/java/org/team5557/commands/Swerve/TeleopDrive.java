package org.team5557.commands.Swerve;

import java.util.function.DoubleSupplier;

import org.library.team254.LatchedBoolean;
import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
    private final Swerve swerve;
    private Rotation2d sustain_heading;
    private LatchedBoolean input_checker;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public TeleopDrive(DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.swerve = RobotContainer.swerve;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        input_checker = new LatchedBoolean();

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (input_checker.update(m_rotationSupplier.getAsDouble() == 0.0)) {
            sustain_heading = swerve.getPose().getRotation();
            RobotContainer.raw_controllers.resetTheta();
        }

        double rotationalVelocity; 
        if(m_rotationSupplier.getAsDouble() == 0.0) {
            rotationalVelocity = RobotContainer.raw_controllers.calculateTheta(sustain_heading.getRadians()) * swerve.getMotorOutputLimiter();
            rotationalVelocity += Math.copySign(Swerve.ROTATIONAL_STATIC_CONSTANT / Swerve.MAX_VOLTAGE
                * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, rotationalVelocity);
        } else {
            rotationalVelocity = m_rotationSupplier.getAsDouble() * swerve.getMotorOutputLimiter();
        }

        swerve.drive(
            new ChassisSpeeds(
                m_translationXSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                m_translationYSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                rotationalVelocity),
            true,
            Constants.superstructure.center_of_rotation
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0), true, Constants.superstructure.center_of_rotation);
    }
}