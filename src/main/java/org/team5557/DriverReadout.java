package org.team5557;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverReadout {

    public DriverReadout(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.shuffleboard.driver_readout_key);
        /*
        tab.addNumber("Pressure", () -> container.getSuperstructure().getCurrentPressure()).withSize(2, 2)
                .withPosition(0, 0).withWidget(BuiltInWidgets.kDial);*/
        tab.add("Alerts", SmartDashboard.getData("Alerts")).withPosition(4, 0).withSize(2, 2);
        tab.add("Autonomous Mode", container.getAutonomousChooser().getModeChooser()).withSize(2, 1).withPosition(0, 0);
        tab.addCamera("limelight", "limelight", "http://limelight.local:5800", "http://10.55.57.11:5800").withSize(3, 3)
                .withPosition(4, 0);
    }
}