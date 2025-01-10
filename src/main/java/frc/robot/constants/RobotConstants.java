package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;

public final class RobotConstants {

	public enum RobotMode {
		REAL, // Physical robot hardware
		SIM, // Simulation mode
		REPLAY // Replay mode for log analysis
	}

  public static final RobotMode simMode = RobotMode.SIM;
    public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : simMode;

	public static final double BATTERY_VOLTAGE_CRITICAL = 11.5; // Volts
	public static final double BATTERY_VOLTAGE_WARNING = 12.0; // Volts

	public static final boolean FORCE_REDUX_SERVER_ON = false;

	public static final double ROBOT_MASS = 65.000; // kg
	public static final Matter CHASSIS =
			new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = 5.450; // m/s
}