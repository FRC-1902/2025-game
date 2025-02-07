package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ControllerSubsystem extends SubsystemBase {
  private CommandXboxController commandDriveController;
  private CommandXboxController commandManipController;
  private XboxController driveController;
  private XboxController manipController;

  /**
   * Enumeration of buttons on the Xbox controller.
   */
  public enum Button{
    A(1), B(2), X(3), Y(4), LB(5), RB(6), LS(9), RS(10);

    public final int id; 
    Button(int id) {
      this.id = id;
    }
  }

  /**
   * Enumeration of axes on the Xbox controller.
   */
  public enum Axis{
    LX(0), LY(1), RX(4), RY(5), LT(2), RT(3);

    public final int id;
    Axis(int id) {
      this.id = id;
    }
  }

  public enum ControllerName{
    DRIVE, MANIP
  }

  /** Creates a new Controllers. */
  public ControllerSubsystem() {
    commandDriveController = new CommandXboxController(Constants.Controller.DRIVE_CONTROLLER_PORT);
    commandManipController = new CommandXboxController(Constants.Controller.MANIP_CONTROLLER_PORT);
    driveController = commandDriveController.getHID();
    manipController = commandManipController.getHID();
  }

  /**
   * Checks if the specified button is pressed.
   * @param name Controller name (DRIVE or MANIP)
   * @param button Button name
   * @return true if the button is pressed, false otherwise.
   * If the controller is specified incorrectly, returns false.
   */
  public boolean get(ControllerName name, Button b){
    switch(name){
    case DRIVE:
      return driveController.getRawButton(b.id);
    case MANIP:
      return manipController.getRawButton(b.id);
    default:
      return false;
    }
  }

  /**
   * Checks the value of the specified axis.
   * @param name Controller name (DRIVE or MANIP)
   * @param axis Axis name
   * @return the value of the axis, between -1 and 1.
   * If the controller is specified incorrectly, returns 0.
   */
  public double get(ControllerName name, Axis a){
    switch(name){
    case DRIVE:
      return driveController.getRawAxis(a.id);
    case MANIP:
      return manipController.getRawAxis(a.id);
    default:
      return 0.0;
    }
  }

  /**
   * Returns the command trigger for the specified button.
   * @param name Controller name (DRIVE or MANIP)
   * @param button Button name
   * @return the command trigger for the button.
   * If the controller is specified incorrectly, returns null.
   */
  public Trigger getTrigger(ControllerName name, Button b){
    switch(name){
    case DRIVE:
      return commandDriveController.button(b.id);
    case MANIP:
      return commandManipController.button(b.id);
    default:
      return null;
    }
  }

  /**
   * Returns the degree value of the DPAD.
   * @param name Controller name (DRIVE or MANIP)
   * @return the degree value of the DPAD.
   * If the controller is specified incorrectly, returns 0.
   */
  public int getDPAD(ControllerName name) {
    switch(name) {
      case DRIVE:
        return driveController.getPOV();
      case MANIP:
        return manipController.getPOV();
      default:
        return 0;
    }
  }

  public CommandXboxController getCommandController(ControllerName name) {
    switch(name){
      case DRIVE:
        return commandDriveController;
      case MANIP:
        return commandManipController;
      default:
        return null;
    }
  }
}
