package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;


// TODO: Play with this later and see whats actaully useful/ wanted
public class AlertManager {

	public enum Alerts {
		SWERVE_CONFIG("Swerve Configuration Not Found!", AlertType.kError),
		LOW_BATTERY("Low Battery", AlertType.kWarning),
		CRITICAL_BATTERY("Critical Battery", AlertType.kError),
		AUTOBUILDER("PathPlanner AutoBuilder Error", AlertType.kError),
		PATH_PLANNER("PathPlanner Setup Failed", AlertType.kError);

		private final String message;
		private final AlertType type;

		Alerts(String message, AlertType type) {
			this.message = message;
			this.type = type;
		}

		public String getMessage() {
			return message;
		}

		public AlertType getType() {
			return type;
		}
	}

	private static final Map<Alerts, Alert> alerts = new HashMap<>();

	static {
		for (Alerts alertType : Alerts.values()) {
			Alert alert = new Alert(alertType.getMessage(), alertType.getType());
			alert.set(false);
			alerts.put(alertType, alert);
		}
	}

	public static void setAlert(Alerts alertType, boolean active) {
		Alert alert = alerts.get(alertType);
		if (alert != null) {
			alert.set(active);
			if (active) {
				// Report the warning to the Driver Station
				DriverStation.reportWarning(alertType.getMessage(), false);
			}
		}
	}
}
