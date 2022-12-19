// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import org.library.team6328.Alert;
import org.library.team6328.Alert.AlertType;
import org.littletonrobotics.junction.Logger;
import org.team5557.auto.AutonomousChooser;
import org.team5557.auto.AutonomousTrajectories;
import org.team5557.commands.Swerve.AimDrive;
import org.team5557.commands.Swerve.CoPilotCommand;
import org.team5557.commands.Swerve.TeleopDrive;
import org.team5557.subsystems.LEDs;
import org.team5557.subsystems.RawControllers;
import org.team5557.subsystems.Swerve;
import org.team5557.vision.GoalPlanner;
import org.team5557.vision.VisionManager;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  public static final XboxController primary_controller = new XboxController(Constants.ports.primary_controller);
  public static final Swerve swerve = new Swerve();
  public static final VisionManager vision_manager = new VisionManager();
  public static final GoalPlanner goal_planner = new GoalPlanner();
  public static final RawControllers raw_controllers = new RawControllers();

  @SuppressWarnings("unused")
  private final DriverReadout driver_readout = new DriverReadout(this);
  public static final LEDs leds = new LEDs();
  private final AutonomousChooser autonomous_chooser = new AutonomousChooser(new AutonomousTrajectories());
  

  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(swerve);

    swerve.setDefaultCommand(new TeleopDrive(this::getForwardInput, this::getStrafeInput, this::getRotationInput));

    configureButtonBindings();

    if (Constants.tuning_mode) {
      new Alert("Tuning mode active, expect decreased network performance.",
          AlertType.INFO).set(true);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(primary_controller::getStartButton).whileTrue(
      new RunCommand(() -> Logger.getInstance().recordOutput("Marker", true)));

    new Trigger(primary_controller::getLeftBumper).whileTrue(
      new AimDrive(this::getForwardInput, this::getStrafeInput, this::getRightStickAngle)
    );

    new Trigger(() -> primary_controller.getPOV() == 90).whileTrue(
      new AimDrive(this::getForwardInput, this::getStrafeInput, -Math.PI/2)
    );

    new Trigger(primary_controller::getBackButton).onTrue(new InstantCommand(() -> swerve.zeroRotation()));

    new Trigger(primary_controller::getRightBumper).whileTrue(
      new CoPilotCommand(this::getForwardInput, this::getStrafeInput, this::getRotationInput));
  }

  public Swerve getSwerve() {
    return swerve;
  }

  public AutonomousChooser getAutonomousChooser() {
    return autonomous_chooser;
  }

  public LEDs getLEDs() {
    return leds;
  }

  public GoalPlanner getGoalPlanner() {
    return goal_planner;
  }

  public VisionManager getVisionManager() {
    return vision_manager;
  }

  private static double deadband(double value, double tolerance) {
    if (Math.abs(value) < tolerance)
        return 0.0;

    return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
  }

  private static double square(double value) {
      return Math.copySign(value * value, value);
  }

  public double getForwardInput() {
      return -square(deadband(primary_controller.getLeftY(), 0.1)) * Swerve.MAX_VELOCITY_METERS_PER_SECOND;
  }

  public double getStrafeInput() {
      return -square(deadband(primary_controller.getLeftX(), 0.1)) * Swerve.MAX_VELOCITY_METERS_PER_SECOND;
  }

  private double getRotationInput() {
      return -square(deadband(primary_controller.getRightX(), 0.1)) * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }

  private double getRightStickAngle() {
      return Math.atan2(primary_controller.getRightX(), -primary_controller.getRightY());
  }
}
