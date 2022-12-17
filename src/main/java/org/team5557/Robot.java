// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import org.team5557.Constants.Mode;
import org.team5557.subsystems.LEDs.State;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.library.team6328.Alert;
import org.library.team6328.Alert.AlertType;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private RobotContainer robotContainer;
  private WPILOGWriter logReceiver;
  private Command autoCommand;
  private double autoStart;
  private boolean autoMessagePrinted;
  private boolean batteryNameWritten = false;

  private final Alert logNoFileAlert =
      new Alert("No log path set for current robot. Data will NOT be logged.",
          AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.",
          AlertType.ERROR);
  private final Alert logOpenFileAlert = new Alert(
      "Failed to open log file. Data will NOT be logged.", AlertType.ERROR);
  private final Alert logWriteAlert =
      new Alert("Failed write to the log file. Data will NOT be logged.",
          AlertType.ERROR);
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.",
          AlertType.WARNING);

  public Robot() {
    super(Constants.kloop_period);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    Logger logger = Logger.getInstance();
    setUseTiming(Constants.getRobotMode() != Mode.REPLAY);
    logger.recordMetadata("Robot", Constants.getRobotMode().toString());
    logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuning_mode));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    /*
    logger.recordMetadata("BatteryName", BatteryTracker.scanBattery(1.0));
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    */

    switch (Constants.getRobotMode()) {
      case REAL:
        logReceiver = new WPILOGWriter("/media/sda1/");
        logger.addDataReceiver(logReceiver);
        logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kRev);
        //LoggedSystemStats.getInstance().setPowerDistributionConfig(50, ModuleType.kRev);
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    logger.start();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();
    
    Logger.getInstance().recordOutput("ActiveCommands/Scheduler",
        NetworkTableInstance.getDefault()
            .getEntry("/LiveWindow/Ungrouped/Scheduler/Names")
            .getStringArray(new String[] {}));
            
    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.getLEDs().requestState(State.RAINBOW);
  }

  @Override
  public void autonomousInit() {
    robotContainer.getAutonomousChooser().getCommand(robotContainer).schedule();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
