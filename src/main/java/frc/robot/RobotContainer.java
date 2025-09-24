package frc.robot;

import java.io.File;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.Axis;
import frc.robot.subsystems.ControllerSubsystem.Button;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntake.FloorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.AlgaeIntake.AlgaeConstants;
import frc.robot.subsystems.AlgaeIntake.AlgaeSubsystem;
import frc.robot.commands.drive.IntakeFactory;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.vision.CoralDetectorSim;
import frc.robot.subsystems.vision.VisionCamera;
// import frc.robot.commands.drive.ObjectAlign;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  SwerveSubsystem swerve;
  LEDSubsystem led;
  ControllerSubsystem controllers;
  ElevatorSubsystem elevatorSubsystem;
  FloorSubsystem floorSubsystem; 
  AlgaeSubsystem algaeSubsystem; 
  EndEffectorSubsystem endEffectorSubsystem; 
  IntakeFactory intakeFactory; 
  CoralDetectorSim coralDetectorSim; 
  VisionSubsystem vision; 
  private final Field2d field;
  public static final boolean MAPLESIM = true; 

  public RobotContainer() {
    controllers = ControllerSubsystem.getInstance();
    
     swerve = new SwerveSubsystem(new SwerveReal(new File(Filesystem.getDeployDirectory(), "swerve")));
		vision = new VisionSubsystem(
      swerve::addVisionMeasurement, 
      new VisionCamera(Constants.Vision.CAMERA_ONE, Constants.Vision.CAMERA_ONE_POS), 
      new VisionCamera(Constants.Vision.CAMERA_TWO, Constants.Vision.CAMERA_TWO_POS)
      // new VisionCamera(Constants.Vision.CAMERA_THREE, Constants.Vision.CAMERA_THREE_POS)
    );

    coralDetectorSim = new CoralDetectorSim(4, true); 

    
    elevatorSubsystem = new ElevatorSubsystem(() -> new Rotation2d());
    floorSubsystem = new FloorSubsystem(elevatorSubsystem::isSafeIn, swerve, coralDetectorSim); 
    algaeSubsystem = new AlgaeSubsystem(elevatorSubsystem::currentPosition); 
    endEffectorSubsystem = new EndEffectorSubsystem(); 

    elevatorSubsystem.setFloorAngle(floorSubsystem::currentAngle);

    intakeFactory = new IntakeFactory(floorSubsystem, elevatorSubsystem, endEffectorSubsystem); 

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
      () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftY(), Constants.Controller.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftX(), Constants.Controller.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getRightX(), Constants.Controller.RIGHT_X_DEADBAND), // Right Stick Turning   
      () -> controllers.getDPAD(ControllerSubsystem.ControllerName.DRIVE),
      () -> (controllers.get(ControllerName.DRIVE, Axis.LT) > 0.5)
    );
    swerve.setDefaultCommand(closedDrive);

    bindButtons();
  }

  private void bindButtons() {
   controllers.getTrigger(ControllerName.DRIVE, Button.A)
        .onTrue(elevatorSubsystem.setPosition(ElevatorConstants.Position.L3));
    controllers.getTrigger(ControllerName.DRIVE, Button.B)
        .onTrue(elevatorSubsystem.setPosition(ElevatorConstants.Position.L1));
    controllers.getTrigger(ControllerName.DRIVE, Button.X)
        .onTrue(elevatorSubsystem.setPosition(ElevatorConstants.Position.HOME));

    controllers.getTrigger(ControllerName.DRIVE, Button.Y)
        .whileTrue(intakeFactory.initialIntake()); 
    
    //XXX: Testing command that tells the FloorIntake that is has Coral
    //controllers.getTrigger(ControllerName.DRIVE, Button.A)
       // .whileTrue(intakeFactory.coralButton(true))
       // .whileFalse(intakeFactory.coralButton(false));

    controllers.getTrigger(ControllerName.DRIVE, Button.LB)
        .whileTrue(algaeSubsystem.setPivotAngle(AlgaeConstants.Positions.MIN_PIVOT))
        .whileFalse(algaeSubsystem.setPivotAngle(AlgaeConstants.Positions.DEFAULT_ANGLE));
    //controllers.getTrigger(ControllerName.DRIVE, Button.A).whileTrue(new InstantCommand(() -> System.out.println("test success")));
  }
}