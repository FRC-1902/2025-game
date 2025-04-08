package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.floorIntake.AutoIntakeFactory;
import frc.robot.util.Elastic;

public class Robot extends LoggedRobot {
  private AutoSelector autoSelector;
  private Alert lowBatteryAlert;
  private Alert criticalBatteryAlert;
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
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs")); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(Constants.PDH_ID, ModuleType.kRev); // Enables power distribution logging
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
    // Logger.registerURCL(URCL.startExternal());
    
    // Set up alerts
    lowBatteryAlert = new Alert("Low Battery", AlertType.kWarning);
    criticalBatteryAlert = new Alert("Critcal Battery", AlertType.kError);

    // Start AdvantageKit logger
    Logger.start();
    DataLogManager.start();

    robotContainer = new RobotContainer();
    autoSelector = new AutoSelector(robotContainer);
    PathfindingCommand.warmupCommand().schedule();
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run();

      if (AutoIntakeFactory.intakeFlag) {
        robotContainer.autoIntakeFactory.executeCleanup();
      }
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
      Elastic.selectTab("Autonomous");

      // Check battery voltage at autonomous start
      checkBatteryVoltage();

      autonomousCommand = autoSelector.getSelectedCommand();
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

      Elastic.selectTab("Teleoperated");

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
        Elastic.Notification criticalBatteryElastic = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Battery Level Critical", "");
        criticalBatteryAlert.set(true);
        Elastic.sendNotification(criticalBatteryElastic);
      
      } else if (voltage <= Constants.BATTERY_VOLTAGE_WARNING) {
        Elastic.Notification lowBatteryElastic = new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "Battery Level Low", "");
        lowBatteryAlert.set(true);
        Elastic.sendNotification(lowBatteryElastic);
      }
    }
}
