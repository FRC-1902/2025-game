package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.io.File;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FloorIntake;
import frc.robot.FieldConstants.WaypointType;

import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.Axis;
import frc.robot.subsystems.ControllerSubsystem.Button;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntake.FloorConstants;
import frc.robot.subsystems.FloorIntake.FloorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.AlgaeIntake.AlgaeSubsystem;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstants;

// import frc.robot.commands.drive.ObjectAlign;

public class RobotContainer {

  SwerveSubsystem swerve;
  
  LEDSubsystem led;
  ControllerSubsystem controllers;
  ElevatorSubsystem elevatorSubsystem;
  FloorSubsystem floorSubsystem;  
  EndEffectorSubsystem endSubsystem;
  AlgaeSubsystem algaeSubsystem;

  private final Field2d field;
  public static final boolean MAPLESIM = true; 

  public RobotContainer() {
    controllers = ControllerSubsystem.getInstance();
    swerve = new SwerveSubsystem(new SwerveReal(new File(Filesystem.getDeployDirectory(), "swerve")));

    elevatorSubsystem = new ElevatorSubsystem(); 
    floorSubsystem = new FloorSubsystem(); 
    endSubsystem = new EndEffectorSubsystem(); 
    algaeSubsystem = new AlgaeSubsystem();

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
    //controllers.getTrigger(ControllerName.DRIVE, Button.A).whileTrue(new InstantCommand(() -> System.out.println("test success")));
    controllers.getTrigger(ControllerName.DRIVE, Button.Y)
        .whileTrue(floorSubsystem.setPivotAngle(Rotation2d.fromDegrees(FloorConstants.Positions.FLOOR_ANGLE)))
        .whileFalse((floorSubsystem.setPivotAngle(FloorConstants.Positions.DEFAULT_ANGLE)));
  }
}