package org.team5557.commands.Swerve;

import java.util.function.DoubleSupplier;

import org.team5557.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimDrive extends CommandBase {

    private final Swerve swerve;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier goalAngleSupplierRadians;
    private final PIDController rotationController;

    public AimDrive(Swerve swerve, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier goalAngleSupplierRadians) {
        this.swerve = swerve;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.goalAngleSupplierRadians = goalAngleSupplierRadians;
        this.rotationController = swerve.rotationController;

        addRequirements(swerve);
    }

    public AimDrive(Swerve swerve, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, double goalAngleRadians) {
        return AimDrive(swerve, translationXSupplier, translationYSupplier, ()-> ));
    }

    @Override
    public void initialize() {
        rotationController.reset();
    }

    @Override
    public void execute() {
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                m_translationXSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                m_translationYSupplier.getAsDouble() * swerve.getMotorOutputLimiter(),
                rotationController.calculate(swerve.getPose().getRotation().getRadians(), goalAngleSupplierRadians.getAsDouble()) * swerve.getMotorOutputLimiter(),
                swerve.getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    
}
