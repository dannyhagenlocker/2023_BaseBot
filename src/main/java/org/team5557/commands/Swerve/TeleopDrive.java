package org.team5557.commands.Swerve;

import java.util.function.DoubleSupplier;

import org.team5557.subsystems.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
    private final Swerve swerve;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public TeleopDrive(Swerve drivetrainSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.swerve = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if (m_rotationSupplier.getAsDouble() == 0.0) {
            rotationSignal = 
        }
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement

        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                m_translationXSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                m_translationYSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                m_rotationSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                swerve.getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}