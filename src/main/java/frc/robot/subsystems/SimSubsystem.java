// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimSubsystem extends SubsystemBase {
  /** Creates a new SimSubsystem. */

  private double simAlgaeAngle;
  private double simElevatorPosition; 
  private double simfloorIntakeAngle; 
  private double simEndEffectorPosition;

  private boolean simEnabled = false;

  private final AlgaeIntakeSubsystem algaeIntake;
  private final ElevatorSubsystem elevator;
  private final FloorIntakeSubsystem floorIntake;
  private final EndEffectorSubsystem endEffector;

  private static final double ALGAE_DELTA = 0.3;
  private static final double ELEVATOR_DELTA = 0.05; 
  private static final double FLOOR_INTAKE_DELTA = 0.5; 

  public SimSubsystem(AlgaeIntakeSubsystem algaeIntake, ElevatorSubsystem elevator, FloorIntakeSubsystem floorIntake, EndEffectorSubsystem endEffector) {
    this.algaeIntake = algaeIntake;
    this.elevator = elevator;
    this.floorIntake = floorIntake;
    this.endEffector = endEffector;
  }


  public void setSimEnabled(boolean enabled) {
    simEnabled = enabled;
  }

  @Override
  public void periodic() {

    if (!simEnabled) {
      return;
    }

    // Algae Intake Rotation
    double targetAlgaeAngle = algaeIntake.targetAngle.getRadians();

    if (Math.abs(targetAlgaeAngle - simAlgaeAngle) > ALGAE_DELTA) {
      if (targetAlgaeAngle > simAlgaeAngle) {
        simAlgaeAngle += ALGAE_DELTA;
      } else {
        simAlgaeAngle -= ALGAE_DELTA;
      }
    } else {
      simAlgaeAngle = targetAlgaeAngle;
    }
    Logger.recordOutput("AlgaeIntake/TargetAngle", targetAlgaeAngle);
    Logger.recordOutput("AlgaeIntake/simTargetAngle", simAlgaeAngle);

    // Elevator Position
    double targetElevatorPosition = (elevator.targetPosition.getHeight()/2.4);

    if (Math.abs(targetElevatorPosition - simElevatorPosition) > ELEVATOR_DELTA) {
      if (targetElevatorPosition > simElevatorPosition) {
        simElevatorPosition += ELEVATOR_DELTA;
      } else {
        simElevatorPosition -= ELEVATOR_DELTA;
      }
    } else {
      simElevatorPosition = targetElevatorPosition;
    }

    Pose3d elevatorStagePose = new Pose3d(new Translation3d(0.2, 0, simElevatorPosition+0.05), new Rotation3d());
    Pose3d intakeArmPose = new Pose3d(new Translation3d(0.312, 0, 0.4+(simElevatorPosition*2)) ,new Rotation3d(0.0, simAlgaeAngle, 0.0));
    Pose3d endEffector = new Pose3d(new Translation3d(0.0, 0, simElevatorPosition*2), new Rotation3d());


    Logger.recordOutput("AlgaeIntake/IntakeArmPose", intakeArmPose);
    Logger.recordOutput("Elevator/EndEffector", new Pose3d[] { endEffector });
    Logger.recordOutput("Elevator/Stage", new Pose3d[] { elevatorStagePose });
  }
}
