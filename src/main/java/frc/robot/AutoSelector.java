package frc.robot;

import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.Optional;

import javax.naming.PartialResultException;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ContinuousConditionalCommand;
import frc.robot.commands.ElevatorFactory;
import frc.robot.commands.algaeIntake.AlgaeIntakeCommand;
import frc.robot.commands.algaeIntake.AlgaeOuttakeCommand;
import frc.robot.commands.endEffector.EndEffectorFactory;
import frc.robot.commands.endEffector.ScoreCommand;
import frc.robot.commands.drive.AutoDriveFactory;
import frc.robot.commands.drive.AutoSnapToCoral;
import frc.robot.commands.drive.PathToWaypoint;
import frc.robot.commands.drive.SnapToCoral;
import frc.robot.commands.drive.SnapToWaypoint;
import frc.robot.commands.floorIntake.AutoIntakeFactory;
import frc.robot.commands.floorIntake.PositionIntakeCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import frc.robot.FieldConstants;
import frc.robot.Constants.EndEffector;
import frc.robot.Constants;

/*
 * Publishes a network table chooser to smart dashboard to select the autonomous command. 
 * Compose the auto routines here to put in the selector.
 */
public class AutoSelector {
  private LoggedDashboardChooser<Command> autoChooser;
  private LoggedDashboardChooser<String> algaeDirectionChooser;

  private String pushingP = "Auto/EnablePushingP";
  // private String leftAlgae = "Auto/Left Algae";
  // private String rightAlgae = "Auto/Right Algae";

  SwerveSubsystem swerve;
  AlgaeIntakeSubsystem algaeIntake;
  FloorIntakeSubsystem floorIntake;
  EndEffectorSubsystem endEffector;
  ObjectDetectionSubsystem detectionSubsystem;
  ElevatorSubsystem elevator;

  PositionIntakeCommand deployFloorIntakeCommand;
  AutoIntakeFactory autoIntakeFactory;
  ElevatorFactory elevatorFactory;
  AutoDriveFactory autoDriveFactory;

  ContinuousConditionalCommand continuousConditionalCommand;
  EndEffectorFactory endEffectorFactory;
  LEDSubsystem led;
  
  public AutoSelector(RobotContainer robotContainer) {
    // this.robotContainer = robotContainer;
    swerve = robotContainer.swerve;
    elevator = robotContainer.elevator;
    algaeIntake = robotContainer.algaeIntake;
    floorIntake = robotContainer.floorIntake;
    endEffector = robotContainer.endEffector;
    detectionSubsystem = robotContainer.detectionSubsystem;
    elevator = robotContainer.elevator;
    led = robotContainer.led;

    endEffectorFactory = new EndEffectorFactory(endEffector);
    autoIntakeFactory = new AutoIntakeFactory(floorIntake, elevator, endEffector, led);
    elevatorFactory = new ElevatorFactory(endEffector, elevator, floorIntake);
    autoDriveFactory = new AutoDriveFactory(swerve, detectionSubsystem);

    autoChooser = new LoggedDashboardChooser<>("Auto/Auto Chooser");
    
    SmartDashboard.putBoolean(pushingP, false);
    SmartDashboard.setPersistent(pushingP);

    autoChooser = new LoggedDashboardChooser<>("Auto/Auto Chooser");

    // Dynamic Algae auto
    algaeDirectionChooser = new LoggedDashboardChooser<>("Auto/Algae Direction");
    algaeDirectionChooser.addDefaultOption("None", "none");
    algaeDirectionChooser.addOption("Left", "left");
    algaeDirectionChooser.addOption("Right", "right");
    algaeDirectionChooser.addOption("Both", "both");

    autoChooser.addDefaultOption("Do Nothing", getDoNothingAuto());
    autoChooser.addDefaultOption("Leave", getLeave());
    autoChooser.addOption("Right 4 Piece", getRight1Face());
    autoChooser.addOption("Left 4 Piece", getLeft1Face());
    autoChooser.addOption("Right 2 L3", getRight2L3());
    autoChooser.addOption("Left 2 L3", getLeft2L3());
    autoChooser.addOption("1 L2", get1L2());
    autoChooser.addOption("Algae L2", getAlgaeL2());
  }

  /**
   * @return The selected auto from smart dashboard
  */
  public Command getSelectedCommand() {
    return autoChooser.get();
  }

  private boolean isBlue() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Blue;
    } else {
      return false; // true for default to blue alliance
    }
  }

  public Command setStartPosition(double x, double y) {
    return new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(x, y, Rotation2d.fromDegrees(180)))),
        new InstantCommand(() -> swerve.zeroGyroWithAlliance())
      ),
      new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(FieldConstants.LENGTH - x, FieldConstants.WIDTH - y, Rotation2d.fromDegrees(0)))), 
        new InstantCommand(() -> swerve.zeroGyroWithAlliance())
      ),
      this::isBlue
    );
  }

  public Command setStartPosition(double x, double y, double rotation) {
    return new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(x, y, Rotation2d.fromDegrees(180)))),
        new InstantCommand(() -> swerve.zeroGyroWithAlliance())
      ),
      new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(FieldConstants.LENGTH - x, FieldConstants.WIDTH - y, Rotation2d.fromDegrees(0+rotation)))), 
        new InstantCommand(() -> swerve.zeroGyroWithAlliance())
      ),
      this::isBlue
    );
  }


  /**
   * Aligns and drives to coral and ends once intaken
   * <b><h1>Gussy likes to call this "get verified"</h1></b>
   * @param pathName
   * @return
   */
  private Command getCoralWithDetection(){
    return new ParallelDeadlineGroup(
      new ParallelRaceGroup(
        new WaitCommand(10),
        autoIntakeFactory.getAutonomousIntakeSequence(Constants.FloorIntake.FLOOR_ANGLE)
      ),
      new SequentialCommandGroup(
        new AutoSnapToCoral(swerve, detectionSubsystem),
        new AutoSnapToCoral(swerve, detectionSubsystem),
        new AutoSnapToCoral(swerve, detectionSubsystem)
      )
    );
  } 
  private Command getPushingPCommand() {
    return new ParallelRaceGroup(
      new WaitCommand(4),
      new SnapToWaypoint(swerve, () -> {
        // Get current pose
        Pose2d currentPose = swerve.getPose();
        
        double distance = Units.inchesToMeters(-4); 
        
        Translation2d backwardsVector = new Translation2d(
          distance * Math.cos(currentPose.getRotation().getRadians()), 
          distance * Math.sin(currentPose.getRotation().getRadians())
        );
        
        Translation2d targetTranslation = currentPose.getTranslation().plus(backwardsVector);
        
        return new Pose2d(targetTranslation, currentPose.getRotation());
      })
    );
  } 

  /**
   * Checks if the PushingP SmartDashboard toggle is enabled
   */
  private boolean isPushingPEnabled() {
    return SmartDashboard.getBoolean(pushingP, false);
  }

  // Replace the boolean methods with string-based checks
  private boolean isLeftAlgaeEnabled() {
    String selection = algaeDirectionChooser.get();
    return selection.equals("left") || selection.equals("both");
  }
  
  private boolean isRightAlgaeEnabled() {
    String selection = algaeDirectionChooser.get();
    return selection.equals("right") || selection.equals("both");
  }
  // Auto definitions

  /**
   * Auto that doesn't do anything
   */
  private Command getDoNothingAuto() {
    return setStartPosition(7.140, 7.500);
  }

  /**
   * Auto that leaves the starting line
   */
  private Command getLeave() {
    return new SequentialCommandGroup(
      setStartPosition(7.140, 7.500),
      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),
      swerve.getFollowPathCommand("Leave")
    );
  }

  /**
   * Auto that leaves the starting line
   */
  private Command get1L2() {
    return new SequentialCommandGroup(
      setStartPosition(7.182, 4.182),

      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),

      endEffectorFactory.getIndexSequence(),

      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L2),
        swerve.getFollowPathCommand("1 L2 1"),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),
      swerve.getFollowPathCommand("1 L2 2")
    );
  }


  private Command getAlgaeL2() {
    return new SequentialCommandGroup(
      // Set initial position
      setStartPosition(7.200, 4.19),
      
      // Check for pushing P
      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),
      
      // Index end effector
      endEffectorFactory.getIndexSequence(),
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L2),
        swerve.getFollowPathCommand("Algae 1"),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      autoDriveFactory.autoSnapCommand(FieldConstants.WAYPOINTS.POLES[6]),

      // Place
      new ScoreCommand(endEffector),
      new ParallelCommandGroup(
        swerve.getFollowPathCommand("Algae 2"),
        new SequentialCommandGroup(
          new WaitCommand(.5),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
        )
      ),
      new AlgaeOuttakeCommand(algaeIntake),
      
      // Left algae check
      new ConditionalCommand(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
            swerve.getFollowPathCommand("Algae Left 1"),
            new AlgaeIntakeCommand(algaeIntake)
          ),          
          new ParallelCommandGroup(
            swerve.getFollowPathCommand("Algae Left 2"),
            new SequentialCommandGroup(
              new WaitCommand(.5),
              elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
            )
          ),
          new AlgaeOuttakeCommand(algaeIntake)
        ),
        // If path 1 is NOT enabled
        new WaitCommand(0),
        this::isLeftAlgaeEnabled
      ),
      
      // Right conditional path
      new ConditionalCommand(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
            swerve.getFollowPathCommand("Algae Right 1"),
            new AlgaeIntakeCommand(algaeIntake)
          ),          
          new ParallelCommandGroup(
            swerve.getFollowPathCommand("Algae Right 2"),
            new SequentialCommandGroup(
              new WaitCommand(.5),
              elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
            )
          ),
          new AlgaeOuttakeCommand(algaeIntake)
        ),
        // If path 1 is NOT enabled
        new WaitCommand(0),
        this::isRightAlgaeEnabled
      )
      
      // Return to safe position
      // elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
    );
  }

  private SequentialCommandGroup getRight1Face(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.2, 2.2),

      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),

      endEffectorFactory.getIndexSequence(),

      // Drive to reef and grab algae from L2
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L2),
        new SequentialCommandGroup(
          swerve.getFollowPathCommand("1F 1"),
          autoDriveFactory.autoSnapCommand(FieldConstants.WAYPOINTS.POLES[10])
        ),
        new AlgaeIntakeCommand(algaeIntake)
      ),

      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE ONE

      // Deploy floor intake, outtake algae, and look for a piece

      new ParallelCommandGroup(
        // new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.FLOOR_ANGLE), floorIntake),
        swerve.getFollowPathCommand("1F 2"),
        new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.FLOOR_ANGLE), floorIntake)
        // new InstantCommand(()->floorIntake.setSpeed(1))
      ),
      new AlgaeOuttakeCommand(algaeIntake),
      getCoralWithDetection(),

      // Drive to reef
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          autoIntakeFactory.getAutonomousIndexSequence(),
          new WaitCommand(.02),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3)
        ),
        autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[10])
      ),
      // autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[10]),

      // Place 
      new ScoreCommand(endEffector),

      // END OF CYCLE TWO

      getCoralWithDetection(),

      // Drive to reef & grab algae
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          autoIntakeFactory.getAutonomousIndexSequence(),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3)
        ),
        autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[11])
      ),
      // autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[11]),


      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE THREE

      getCoralWithDetection(),
      
      // Drive to reef & grab algae
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          autoIntakeFactory.getAutonomousIndexSequence(),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L2)
        ),
        autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[11])
      ),
      // autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[11]),

      // Place
      new ScoreCommand(endEffector),     // End of Auto

      elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
    );
  }

  private SequentialCommandGroup getLeft1Face(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.2, 6),

      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),

      endEffectorFactory.getIndexSequence(),

      // Drive to reef and grab algae from L2
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L2),
        new SequentialCommandGroup(
          swerve.getFollowPathCommand("1F 1 Left"),
          autoDriveFactory.autoSnapCommand(FieldConstants.WAYPOINTS.POLES[2])
        ),
        new AlgaeIntakeCommand(algaeIntake)
      ),

      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE ONE

      // Deploy floor intake, outtake algae, and look for a piece

      new ParallelCommandGroup(
        // new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.FLOOR_ANGLE), floorIntake),
        swerve.getFollowPathCommand("1F 2 Left"),
        new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.FLOOR_ANGLE), floorIntake)
        // new InstantCommand(()->floorIntake.setSpeed(1))
      ),
      new AlgaeOuttakeCommand(algaeIntake),
      getCoralWithDetection(),

      // Drive to reef
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          autoIntakeFactory.getAutonomousIndexSequence(),
          new WaitCommand(.02),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3)
        ),
        autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[2])
      ),
      // autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[10]),

      // Place 
      new ScoreCommand(endEffector),

      // END OF CYCLE TWO

      getCoralWithDetection(),

      // Drive to reef & grab algae
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          autoIntakeFactory.getAutonomousIndexSequence(),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3)
        ),
        autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[3])
      ),
      // autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[11]),


      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE THREE

      getCoralWithDetection(),
      
      // Drive to reef & grab algae
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          autoIntakeFactory.getAutonomousIndexSequence(),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L2)
        ),
        autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[3])
      ),
      // autoDriveFactory.autoSnapOffsetCommand(FieldConstants.WAYPOINTS.POLES[11]),

      // Place
      new ScoreCommand(endEffector),     // End of Auto

      elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
    );
  }

  private SequentialCommandGroup getLeft4L3(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.14, 5.8118),

      // Pushing P
      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),

      endEffectorFactory.getIndexSequence(),

      // Drive to reef and grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("4 L3 1 Left"),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE ONE

      // Drive to HP

      // drive far enough away to get rid of algae

      // drive, while outaking algae, intaking, and looking for a piece

      new ParallelCommandGroup(
        swerve.getFollowPathCommand("4 L3 2 Left"),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AlgaeOuttakeCommand(algaeIntake),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
        )
      ),
      getCoralWithDetection(),
      autoIntakeFactory.getAutonomousIndexSequence(),

      // Drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        autoDriveFactory.pathAndSnapCommand(FieldConstants.WAYPOINTS.POLES[0]),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      
      // Place 
      new ScoreCommand(endEffector),

      // END OF CYCLE TWO

      new ParallelCommandGroup(
        swerve.getFollowPathCommand("4 L3 3 Left"),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AlgaeOuttakeCommand(algaeIntake),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
        )
      ),
      getCoralWithDetection(),
      autoIntakeFactory.getAutonomousIndexSequence(),

      // Drive to reef & grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        autoDriveFactory.pathAndSnapCommand(FieldConstants.WAYPOINTS.POLES[1]),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE THREE

      // Drive to get coral
      new ParallelCommandGroup(
        swerve.getFollowPathCommand("4 L3 4 Left"),
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
      ),
      
      getCoralWithDetection(),
      autoIntakeFactory.getAutonomousIndexSequence()
      // End of Auto
    );
  }

  private SequentialCommandGroup getRight2L3(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.14, 2.240), // TODO: fix start pos

      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),

      endEffectorFactory.getIndexSequence(),

      // Drive to reef and grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        new SequentialCommandGroup(
          swerve.getFollowPathCommand("2 L3 1"),
          autoDriveFactory.pathAndSnapCommand(FieldConstants.WAYPOINTS.POLES[8])
        ),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE ONE

      // Drive to HP

      // drive far enough away to get rid of algae

      // drive, while outaking algae, intaking, and looking for a piece

      new ParallelCommandGroup(
        new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.FLOOR_ANGLE), floorIntake),
        swerve.getFollowPathCommand("2 L3 2")
      ),
      new AlgaeOuttakeCommand(algaeIntake),

      // new InstantCommand(()-> DataLogManager.log("Before Coral")),
      getCoralWithDetection(),

      // Drive to reef
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          autoIntakeFactory.getAutonomousIndexSequence(),  // Put inside SequentialCommandGroup 
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3)
        ),
        autoDriveFactory.autoSnapCommand(FieldConstants.WAYPOINTS.POLES[9])
      ),
      
      // Place 
      new ScoreCommand(endEffector),

      // END OF CYCLE TWO

      // Clean Up
      elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)

      // End of Auto
    );
  }

  private SequentialCommandGroup getLeft2L3(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.14, 5.8118),

      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),

      endEffectorFactory.getIndexSequence(),

      // Drive to reef and grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        new SequentialCommandGroup(
          swerve.getFollowPathCommand("2 L3 1 Left"),
          autoDriveFactory.pathAndSnapCommand(FieldConstants.WAYPOINTS.POLES[5])
        ),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE ONE

      // Drive to HP

      // drive far enough away to get rid of algae

      // drive, while outaking algae, intaking, and looking for a piece

      new ParallelCommandGroup(
        swerve.getFollowPathCommand("2 L3 2 Left"),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AlgaeOuttakeCommand(algaeIntake),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN),
          new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.FLOOR_ANGLE), floorIntake)
        )
      ),
      new WaitCommand(1),
      getCoralWithDetection(),

      // Drive to reef
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          autoIntakeFactory.getAutonomousIndexSequence(),  // Put inside SequentialCommandGroup 
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3)
        ),
        autoDriveFactory.autoSnapCommand(FieldConstants.WAYPOINTS.POLES[9])
      ),
      
      // Place 
      new ScoreCommand(endEffector),

      // END OF CYCLE TWO

      // Clean Up
      elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)

      // End of Auto
    );
  }
}

