// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.AlertManager;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Log build metadata
    Logger.recordMetadata("ProjectName", "2025-game");

    switch (Constants.currentMode) {
        case REAL:
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs") TODO: Set USB Path
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
            break;

        case SIM:
            // setUseTiming(false); // Run as fast as possible
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            break;

        case REPLAY:
            // Replaying a log, set up replay source
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            break;
    }

    // Initialize URCL 
    Logger.registerURCL(URCL.startExternal()); // TODO: Remove if issues with over logging occurs
    
    // Start AdvantageKit logger
    Logger.start();
    
    robotContainer = new RobotContainer();
    PathfindingCommand.warmupCommand().schedule();

  }

  @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // Check battery voltage when disabled
        checkBatteryVoltage();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

        // Check battery voltage at autonomous start
        checkBatteryVoltage();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        // Check battery voltage at teleop start
        checkBatteryVoltage();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    /**
     * Check the battery voltage and set alerts if it is low or critical.
     */
    private void checkBatteryVoltage() {
        double voltage = RobotController.getBatteryVoltage();
        if (voltage <= Constants.BATTERY_VOLTAGE_CRITICAL) {
            AlertManager.setAlert(AlertManager.Alerts.CRITICAL_BATTERY, true);
        } else if (voltage <= Constants.BATTERY_VOLTAGE_WARNING) {
            AlertManager.setAlert(AlertManager.Alerts.LOW_BATTERY, true);
        }
    }
}
