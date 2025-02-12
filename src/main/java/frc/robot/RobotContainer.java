package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeOuttakeCommand;
import frc.robot.commands.AutoPlaceFactory;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PlaceCommand;
import frc.robot.commands.drive.AutoDriveFactory;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.intake.AutoIntakeFactory;
import frc.robot.commands.intake.OuttakeFloorIntakeCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.Button;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.DetectionSubsystem;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.drive.ObjectAlign;


public class RobotContainer {

  SwerveSubsystem swerve;
  VisionSubsystem vision;
  AlgaeIntakeSubsystem algaeIntake;
  ElevatorSubsystem elevator;
  EndEffectorSubsystem endEffector;
  FloorIntakeSubsystem floorIntake;
  LEDSubsystem LED;
  ControllerSubsystem controllers;
  DetectionSubsystem detectionSubsystem;

  AutoDriveFactory autoDrive;
  AutoIntakeFactory autoIntake;
  AutoPlaceFactory autoPlaceFactory;

  public RobotContainer() {
    controllers = new ControllerSubsystem();
    vision = new VisionSubsystem(Robot.isSimulation() ? new VisionSim() : new VisionReal());
    swerve = new SwerveSubsystem(vision, new SwerveReal(new File(Filesystem.getDeployDirectory(), "swerve")));

    endEffector = new EndEffectorSubsystem();
    elevator = new ElevatorSubsystem();
    floorIntake = new FloorIntakeSubsystem(elevator);
    LED = new LEDSubsystem();
    algaeIntake = new AlgaeIntakeSubsystem();

    detectionSubsystem = new DetectionSubsystem();


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

    autoDrive = new AutoDriveFactory(swerve);
    autoIntake = new AutoIntakeFactory(floorIntake, elevator, endEffector);
    autoPlaceFactory = new AutoPlaceFactory(endEffector, elevator, floorIntake);

    swerve.setDefaultCommand(closedDrive);
    

    bindButtons();
  }

  private void bindButtons() {

    // Driver Controls
    
    // Place Coral
    //   new Trigger(() -> controllers.getCommandController(ControllerName.DRIVE).getRightTriggerAxis() > 0.5).debounce(0.05)
    //       .onTrue(new PlaceCommand(endEffector));

    //  controllers.getTrigger(ControllerName.DRIVE, Button.RB).debounce(0.05)
    //     .onTrue(new PlaceCommand(endEffector));

    // Score/Outtake Algae
    controllers.getTrigger(ControllerName.DRIVE, Button.RB).debounce(0.05)
      .whileTrue(new AlgaeOuttakeCommand(algaeIntake));

    //  controllers.getTrigger(ControllerName.DRIVE, Button.LB).debounce(0.05)
    //      .whileTrue(new AlgaeOuttakeCommand(algaeIntake));

    // Zero Gyro
    controllers.getTrigger(ControllerName.DRIVE, Button.Y).debounce(0.05)
      .onTrue(new InstantCommand(swerve::zeroGyro));
    
    // Align to Reef
    controllers.getTrigger(ControllerName.DRIVE, Button.B).debounce(0.05)
      .whileTrue(autoDrive.pathAndSnapCommand(WaypointType.REEF));
    
    // Align to Processor
    controllers.getTrigger(ControllerName.DRIVE, Button.X).debounce(0.05)
      .whileTrue(autoDrive.pathAndSnapCommand(WaypointType.PROCESSOR));

    // Align with Coral TODO: Change when Align PR is merged
    // controllers.getTrigger(ControllerName.DRIVE, Button.A).debounce(0.05)
    //       .whileTrue(new ObjectAlign());

    // Align to Cage, Removed for now
    // controllers.getTrigger(ControllerName.DRIVE, Button.X).debounce(0.05)
    //     .whileTrue(autoDrive.pathAndSnapCommand(WaypointType.CAGE));

    // Manipulator Controls

    // L3
    new Trigger(() -> controllers.getCommandController(ControllerName.MANIP).getRightTriggerAxis() > 0.5).debounce(0.05)
      .whileTrue(autoPlaceFactory.getAutoPlace(Constants.Elevator.Position.L3));
        // .onFalse(new ElevatorCommand(elevator, Constants.Elevator.Position.MIN));

    // L2
    controllers.getTrigger(ControllerName.MANIP, Button.RB).debounce(0.05)
      .whileTrue(new ElevatorCommand(elevator, Constants.Elevator.Position.L2))
      .onFalse(new ElevatorCommand(elevator, Constants.Elevator.Position.MIN));

    // L1
    controllers.getTrigger(ControllerName.MANIP, Button.Y).debounce(0.05)
      .whileTrue(autoPlaceFactory.getAutoPlace(Constants.Elevator.Position.L1));
      // .onFalse(new ElevatorCommand(elevator, Constants.Elevator.Position.MIN));

    // Spit Floor Intake
    controllers.getTrigger(ControllerName.MANIP, Button.LS).debounce(0.05)
      .whileTrue(new OuttakeFloorIntakeCommand(floorIntake));

    // Floor Intake
    //new Trigger(() -> controllers.getCommandController(ControllerName.MANIP).getLeftTriggerAxis() > 0.5).debounce(0.05)
    // controllers.getTrigger(ControllerName.MANIP, Button.A).debounce(0.05)
    //   .whileTrue(autoIntake.getIntakeSequence());

    // HP Intake TODO: Uncomment when HPIntake is implemented
    // controllers.getTrigger(ControllerName.MANIP, Button.X).debounce(0.05)
    //     .whileTrue(new HPIntake(floorIntake));

    // Algae Intake
    // controllers.getTrigger(ControllerName.MANIP, Button.LB).debounce(0.05)
    controllers.getTrigger(ControllerName.MANIP, Button.A).debounce(0.05)
      .whileTrue(new AlgaeIntakeCommand(algaeIntake));

    controllers.getTrigger(ControllerName.MANIP, Button.B).debounce(0.05)
      .whileTrue(new AlgaeOuttakeCommand(algaeIntake));

    // Climber Up
    new Trigger(() -> controllers.getCommandController(ControllerName.MANIP).povUp().getAsBoolean()).debounce(0.05)
      .onTrue(new ElevatorCommand(elevator, Constants.Elevator.Position.CLIMB_UP));

    // Climber Down
    new Trigger(() -> controllers.getCommandController(ControllerName.MANIP).povDown().getAsBoolean()).debounce(0.05)
      .whileTrue(new ElevatorCommand(elevator, Constants.Elevator.Position.CLIMB_DOWN));
    
      controllers.getTrigger(ControllerName.MANIP, Button.X).whileTrue(new ObjectAlign(detectionSubsystem, swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return swerve.getAutonomousCommand("Test");
  }
}