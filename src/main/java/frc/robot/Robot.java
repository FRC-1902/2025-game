// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.util.AlertManager;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    // IDK what this is for or if we need it or what to do with it.
		// Force the Redux server to start on port 7244 on the RoboRIO To use download Redux Alchemist navigate to settings then enter RoboRIO IP: roboRIO-2106-FRC.local
		if (RobotConstants.FORCE_REDUX_SERVER_ON) {
			CanandEventLoop.getInstance();
		}

		// Log build metadata
		Logger.recordMetadata("ProjectName", "2025-game");

    switch (RobotConstants.currentMode) {
			case REAL:
				Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
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
    Logger.registerURCL(URCL.startExternal());
    
    // Start AdvantageKit logger
		Logger.start();

    robotContainer = new RobotContainer();
  }

  @Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		// Check battery voltage when disabled
		double voltage = RobotController.getBatteryVoltage();
		if (voltage <= RobotConstants.BATTERY_VOLTAGE_CRITICAL) {
			AlertManager.setAlert(AlertManager.Alerts.CRITICAL_BATTERY_ON_END, true);
		}
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
		double voltage = RobotController.getBatteryVoltage();
		if (voltage <= RobotConstants.BATTERY_VOLTAGE_CRITICAL) {
			AlertManager.setAlert(AlertManager.Alerts.CRITICAL_BATTERY_ON_START, true);
		}
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
}
