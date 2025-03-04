package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.io.File;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FloorIntake;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeOuttakeCommand;
import frc.robot.commands.AutoPlaceFactory;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorFactory;
import frc.robot.commands.EndEffectorFactory;
import frc.robot.commands.PlaceCommand;
import frc.robot.commands.drive.AutoDriveFactory;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.intake.AutoIntakeFactory;
import frc.robot.commands.intake.DeployFloorIntakeCommand;
import frc.robot.commands.intake.IntakeFloorIntakeCommand;
import frc.robot.commands.intake.OuttakeFloorIntakeCommand;
import frc.robot.commands.intake.IndexFloorIntakeCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.Axis;
import frc.robot.subsystems.ControllerSubsystem.Button;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  SwerveSubsystem swerve;
  VisionSubsystem vision;
  AlgaeIntakeSubsystem algaeIntake;
  ElevatorSubsystem elevator;
  EndEffectorSubsystem endEffector;
  FloorIntakeSubsystem floorIntake;
  // LEDSubsystem led;
  ControllerSubsystem controllers;

  AutoDriveFactory autoDrive;
  AutoIntakeFactory autoIntake;
  AutoPlaceFactory autoPlaceFactory;
  ElevatorFactory elevatorFactory;
  EndEffectorFactory endEffectorFactory;
  private final Field2d field;

  public RobotContainer() {
    controllers = ControllerSubsystem.getInstance();
    vision = new VisionSubsystem(Robot.isSimulation() ? new VisionSim() : new VisionReal());
    swerve = new SwerveSubsystem(vision, new SwerveReal(new File(Filesystem.getDeployDirectory(), "swerve")));

    endEffector = new EndEffectorSubsystem();
    elevator = new ElevatorSubsystem();
    floorIntake = new FloorIntakeSubsystem(elevator);
    // led = new LEDSubsystem();
    algaeIntake = new AlgaeIntakeSubsystem(elevator);
    field = new Field2d();
        SmartDashboard.putData("Field", field);

            // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

    DriveCommand closedDrive = new DriveCommand(
      swerve,
      () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftY(), Constants.Controller.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftX(), Constants.Controller.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getRightX(), Constants.Controller.RIGHT_X_DEADBAND) // Right Stick Turning
      // () -> {
      //     double rightTrigger = controllers.getCommandController(ControllerName.DRIVE).getRightTriggerAxis();
      //     double leftTrigger = controllers.getCommandController(ControllerName.DRIVE).getLeftTriggerAxis();
      //     return MathUtil.applyDeadband(rightTrigger - leftTrigger, Constants.Controller.RIGHT_X_DEADBAND);
      // }      
    );

    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.DRIVE) == 90)
    .onTrue(new InstantCommand(() -> closedDrive.setSpinToWin(true)))
    .onFalse(new InstantCommand(() -> closedDrive.setSpinToWin(false)));

    autoDrive = new AutoDriveFactory(swerve);
    endEffectorFactory = new EndEffectorFactory(endEffector);
    autoIntake = new AutoIntakeFactory(floorIntake, elevator, endEffector, endEffectorFactory);
    autoPlaceFactory = new AutoPlaceFactory(endEffector, elevator, floorIntake);
    elevatorFactory = new ElevatorFactory(endEffector, elevator, floorIntake);

    swerve.setDefaultCommand(closedDrive);

    LEDPattern greenPattern = LEDPattern.solid(new Color(0, 255, 0)).atBrightness(Percent.of(50));
    LEDPattern redPattern = LEDPattern.solid(new Color(255, 0, 0)).atBrightness(Percent.of(50));
    LEDPattern yellowPattern = LEDPattern.solid(new Color(255, 255, 0)).breathe(Second.of(.5)).atBrightness(Percent.of(50));

    // led.registerPattern(elevator::isLocked, redPattern);
    // led.registerPattern(() -> { return elevator.pidAtSetpoint() && !(elevator.isAtPosition(Constants.Elevator.Position.MIN)); }, yellowPattern);
    // led.registerPattern(() -> { return algaeIntake.isAlgaeDetected() || floorIntake.pieceSensorActive(); }, greenPattern);

    bindButtons();
  }

  private void bindButtons() {

    /* Driver Controls */

    // Place Coral
    new Trigger(() -> controllers.get(ControllerName.DRIVE, Axis.RT) > 0.5)
      .whileTrue(new PlaceCommand(endEffector));

    // Score/Outtake Algae
    controllers.getTrigger(ControllerName.DRIVE, Button.RB).debounce(0.05)
      .whileTrue(new AlgaeOuttakeCommand(algaeIntake));

    // Zero Gyro
    controllers.getTrigger(ControllerName.DRIVE, Button.Y).debounce(0.05)
      .onTrue(new InstantCommand(swerve::zeroGyro));

    // Spin to win


    // Align to Reef
    controllers.getTrigger(ControllerName.DRIVE, Button.B).debounce(0.05)
      .whileTrue(autoDrive.pathAndSnapCommand(WaypointType.REEF));  

    // Align with Coral TODO: Change when Align PR is merged
    // controllers.getTrigger(ControllerName.DRIVE, Button.A).debounce(0.05)
    //       .whileTrue(new ObjectAlign());

    // Align to Processor
    //controllers.getTrigger(ControllerName.DRIVE, Button.X).debounce(0.05)
      //.whileTrue(autoDrive.pathAndSnapCommand(WaypointType.PROCESSOR));

    // Align to Cage, Removed for now
    // controllers.getTrigger(ControllerName.DRIVE, Button.X).debounce(0.05)
    //     .whileTrue(autoDrive.pathAndSnapCommand(WaypointType.CAGE));

    /* Manipulator Controls */

    // L3
    new Trigger(() -> controllers.get(ControllerName.MANIP, Axis.RT) > 0.5)
        .whileTrue(elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3));
    // L2
    controllers.getTrigger(ControllerName.MANIP, Button.RB).debounce(0.05)
        .whileTrue(elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L2));
    // L1
    controllers.getTrigger(ControllerName.MANIP, Button.Y).debounce(0.05)
        .whileTrue(elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L1));

    // Spit Floor Intake
    controllers.getTrigger(ControllerName.MANIP, Button.LS).debounce(0.05)
      .whileTrue(new OuttakeFloorIntakeCommand(floorIntake));

    // Floor Intake
    new Trigger(() -> controllers.get(ControllerName.MANIP, Axis.LT) > 0.5)
      .whileTrue(autoIntake.getIntakeSequence(Constants.FloorIntake.FLOOR_ANGLE));

    controllers.getTrigger(ControllerName.MANIP, Button.A).debounce(0.05)
      .whileTrue(new InstantCommand(() -> autoIntake.getIntakeSequence(Constants.FloorIntake.FLOOR_ANGLE)));

    // HP Intake
    controllers.getTrigger(ControllerName.MANIP, Button.X).debounce(0.05)
        .whileTrue(autoIntake.getIntakeSequence(Constants.FloorIntake.HP_ANGLE));

    // Algae Intake
    controllers.getTrigger(ControllerName.MANIP, Button.LB).debounce(0.05)
      .whileTrue(new AlgaeIntakeCommand(algaeIntake));
      
    // Climber Up
    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.MANIP) == 0) // TODO: Get Correct angle
      .onTrue(elevatorFactory.getClimberUpSequence());

    // Climber Down
    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.MANIP) == 180) // TODO: Get Correct angle
      .whileTrue(new ElevatorCommand(elevator, Constants.Elevator.Position.CLIMB_DOWN));

    // Climber Intake Out
    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.MANIP) == 90) // TODO: Get Correct angle
      .onTrue(new InstantCommand(() -> floorIntake.setAngle(Rotation2d.fromDegrees(90)), floorIntake));

    // Home
    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.DRIVE) == 90)
      .whileTrue(new ElevatorCommand(elevator, Constants.Elevator.Position.MIN));
  }
}