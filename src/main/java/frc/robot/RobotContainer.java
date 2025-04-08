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
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorFactory;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.algaeIntake.AlgaeIntakeCommand;
import frc.robot.commands.algaeIntake.AlgaeOuttakeCommand;
import frc.robot.commands.algaeIntake.AlgaeOuttakeFactory;
import frc.robot.commands.drive.AutoDriveFactory;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.ReefAlign;
import frc.robot.commands.endEffector.EndEffectorFactory;
import frc.robot.commands.endEffector.ScoreCommand;
import frc.robot.commands.floorIntake.AutoIntakeFactory;
import frc.robot.commands.floorIntake.OuttakeCommand;
import frc.robot.commands.floorIntake.IntakeCommand;
import frc.robot.commands.floorIntake.PositionIntakeCommand;
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
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionSubsystem;
// import frc.robot.commands.drive.ObjectAlign;

public class RobotContainer {

  SwerveSubsystem swerve;
  VisionSubsystem vision;
  AlgaeIntakeSubsystem algaeIntake;
  ElevatorSubsystem elevator;
  EndEffectorSubsystem endEffector;
  FloorIntakeSubsystem floorIntake;
  LEDSubsystem led;
  ControllerSubsystem controllers;
  ObjectDetectionSubsystem detectionSubsystem;
  ReefAlign reefAlignCommand;


  AutoDriveFactory autoDrive;
  AutoIntakeFactory autoIntakeFactory;
  ElevatorFactory elevatorFactory;
  EndEffectorFactory endEffectorFactory;
  AlgaeOuttakeFactory algaeOuttakeFactory;
  private final Field2d field;

  public RobotContainer() {
    controllers = ControllerSubsystem.getInstance();
    swerve = new SwerveSubsystem(new SwerveReal(new File(Filesystem.getDeployDirectory(), "swerve")));
		vision = new VisionSubsystem(
      swerve::addVisionMeasurement, 
      new VisionCamera(Constants.Vision.CAMERA_ONE, Constants.Vision.CAMERA_ONE_POS), 
      new VisionCamera(Constants.Vision.CAMERA_TWO, Constants.Vision.CAMERA_TWO_POS),
      new VisionCamera(Constants.Vision.CAMERA_THREE, Constants.Vision.CAMERA_THREE_POS)
    );

    endEffector = new EndEffectorSubsystem();
    elevator = new ElevatorSubsystem();
    floorIntake = new FloorIntakeSubsystem(elevator);
    led = new LEDSubsystem();
    algaeIntake = new AlgaeIntakeSubsystem(elevator);
    detectionSubsystem = new ObjectDetectionSubsystem(swerve);
    reefAlignCommand = new ReefAlign(swerve);

    // Path Planner logging
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    DriveCommand closedDrive = new DriveCommand(
      swerve,
      detectionSubsystem,
      () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftY(), Constants.Controller.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftX(), Constants.Controller.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getRightX(), Constants.Controller.RIGHT_X_DEADBAND), // Right Stick Turning   
      () -> controllers.getDPAD(ControllerSubsystem.ControllerName.DRIVE)
      // () -> (controllers.get(ControllerName.DRIVE, Axis.LT) > 0.5)
    );

    autoDrive = new AutoDriveFactory(swerve, detectionSubsystem);
    endEffectorFactory = new EndEffectorFactory(endEffector);
    autoIntakeFactory = new AutoIntakeFactory(floorIntake, elevator, endEffector, led);
    elevatorFactory = new ElevatorFactory(endEffector, elevator, floorIntake);
    algaeOuttakeFactory = new AlgaeOuttakeFactory(algaeIntake);

    swerve.setDefaultCommand(closedDrive);

    LEDPattern greenPattern = LEDPattern.solid(new Color(0, 255, 0)).atBrightness(Percent.of(50));
    LEDPattern redPattern = LEDPattern.solid(new Color(255, 0, 0)).atBrightness(Percent.of(50));
    LEDPattern yellowPattern = LEDPattern.solid(new Color(255, 255, 0)).breathe(Second.of(.5)).atBrightness(Percent.of(30));

    led.registerPattern(elevator::isLocked, redPattern);
    led.registerPattern(() -> { return elevator.isAtPosition() && !(elevator.isAtPosition(Constants.Elevator.Position.MIN)); }, yellowPattern);
    led.registerPattern(algaeIntake::isPieceSensorActive, greenPattern);

    bindButtons();
  }

  private void bindButtons() {

    /* Driver Controls */

    // Place Coral
    new Trigger(() -> controllers.get(ControllerName.DRIVE, Axis.RT) > 0.5)
      .whileTrue(new ScoreCommand(endEffector));

    // Re Index Coral
    controllers.getTrigger(ControllerName.DRIVE, Button.LB).debounce(0.05)
      .onTrue(endEffectorFactory.getReverseSequence());

    // Score/Outtake Algae
    controllers.getTrigger(ControllerName.DRIVE, Button.RB).debounce(0.05)
      .whileTrue(algaeOuttakeFactory.algaeOuttakeSequence());

    // Zero Gyro
    controllers.getTrigger(ControllerName.DRIVE, Button.Y).debounce(0.05)
      .onTrue(new InstantCommand(swerve::zeroGyro));
    
    // Align to Reef
    controllers.getTrigger(ControllerName.DRIVE, Button.B).debounce(0.05)
    .whileTrue(reefAlignCommand)
    .onFalse(new InstantCommand(reefAlignCommand::resetPoleIndex));
  
    // Align to left Reef Pole
    new Trigger(() -> controllers.getTrigger(ControllerName.DRIVE, Button.B).getAsBoolean() && controllers.get(ControllerName.DRIVE, Axis.LX) < -Constants.Controller.LEFT_Y_DEADBAND)
      .onTrue(new InstantCommand(() -> reefAlignCommand.navigateToDirectionalPole(false)));
    
    // Align to right Reef Pole
    new Trigger(() -> controllers.getTrigger(ControllerName.DRIVE, Button.B).getAsBoolean() && controllers.get(ControllerName.DRIVE, Axis.LX) > Constants.Controller.LEFT_Y_DEADBAND)
      .onTrue(new InstantCommand(() -> reefAlignCommand.navigateToDirectionalPole(true)));

    // Align to Processor
    controllers.getTrigger(ControllerName.DRIVE, Button.X).debounce(0.05)
      .whileTrue(autoDrive.snapCommand(WaypointType.PROCESSOR)); 

    // Align to L1
    controllers.getTrigger(ControllerName.DRIVE, Button.A).debounce(0.05)
      .whileTrue(autoDrive.snapCommand(WaypointType.TROUGH)); 

    // Align to Barge
    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.DRIVE) == 180)
      .whileTrue(autoDrive.bargeAlignCommand(WaypointType.BARGE));

    new Trigger(() -> controllers.get(ControllerName.DRIVE, Axis.LT) > 0.5)
      .whileTrue(autoDrive.pathAndSnapCoralCommand());

    /* Manipulator Controls */

    // L3
    new Trigger(() -> controllers.get(ControllerName.MANIP, Axis.RT) > 0.5)
        .whileTrue(elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3))
        .onFalse(elevatorFactory.getElevatorDownCommand());
    // L2
    controllers.getTrigger(ControllerName.MANIP, Button.RB).debounce(0.05)
        .whileTrue(elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L2))
        .onFalse(elevatorFactory.getElevatorDownCommand());
    // L1
    controllers.getTrigger(ControllerName.MANIP, Button.Y).debounce(0.05)
        .whileTrue(elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L1))
        .onFalse(elevatorFactory.getElevatorDownCommand());

    // Spit Floor Intake
    controllers.getTrigger(ControllerName.MANIP, Button.LS).debounce(0.05)
      .whileTrue(new OuttakeCommand(floorIntake));

    // Floor Intake
    new Trigger(() -> controllers.get(ControllerName.MANIP, Axis.LT) > 0.2)
      // .whileTrue(new IntakeCommand(floorIntake, led));
      .whileTrue(autoIntakeFactory.getIntakeSequence(Constants.FloorIntake.FLOOR_ANGLE));

    // Algae Intake
    controllers.getTrigger(ControllerName.MANIP, Button.LB).debounce(0.05)
      .whileTrue(new AlgaeIntakeCommand(algaeIntake));
      
    // Climber Up
    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.MANIP) == 0) // TODO: Get Correct angle
      .onTrue(elevatorFactory.getClimberUpSequence());

    // Climber Down
    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.MANIP) == 180) // TODO: Get Correct angle
      .whileTrue(new ElevatorCommand(elevator, Constants.Elevator.Position.CLIMB_DOWN));

    // Climber Intake In
    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.MANIP) == 90) // TODO: Get Correct angle
      .onTrue(new InstantCommand(() -> floorIntake.setAngle(Rotation2d.fromDegrees(70)), floorIntake));

    // Home
    new Trigger(() -> controllers.getDPAD(ControllerSubsystem.ControllerName.MANIP) == 270)
      .whileTrue(new ElevatorCommand(elevator, Constants.Elevator.Position.HOLD))
      .onFalse(new ElevatorCommand(elevator, Constants.Elevator.Position.HOME));
  }
}