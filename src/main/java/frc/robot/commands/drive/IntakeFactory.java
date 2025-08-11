// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.FloorIntake.FloorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntake.FloorConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

/** Add your docs here. */
public class IntakeFactory {

    FloorSubsystem floorSubsystem; 
    ElevatorSubsystem elevatorSubsystem; 
    EndEffectorSubsystem endEffectorSubsystem; 
    public static Boolean intakeFlag;
    double startTime; 

    
    public IntakeFactory(FloorSubsystem floorSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem){
        this.floorSubsystem = floorSubsystem; 
        this.elevatorSubsystem = elevatorSubsystem; 
        this.endEffectorSubsystem = endEffectorSubsystem;  

        intakeFlag = false; 
    }

    public boolean canRun() {
        if (floorSubsystem.hasCoral()
                || endEffectorSubsystem.isFrontPieceSensorActive()
                || endEffectorSubsystem.isBackPieceSensorActive()
            ) {
            return false;
        } else {
            return true;
        }
    }

    public Command initialIntake() {
        return Commands.either(
            floorSubsystem.setPivotAngle(FloorConstants.Positions.MAX_PIVOT)
                .alongWith(elevatorSubsystem.setPosition(ElevatorConstants.Position.MIN))
                .andThen(floorSubsystem.runRollers(1))
                .until(() -> floorSubsystem.hasCoral()),
            Commands.waitUntil(this::canRun),
            this::canRun
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .finallyDo((wasCancelled) -> {intakeFlag = true;});
    }

    public Command initialIndex() {
        return Commands.run(() -> floorSubsystem.runRollers(-1))
            .alongWith(Commands.run(() -> endEffectorSubsystem.setSpeed(0.5)))
            .until(() -> endEffectorSubsystem.isBackPieceSensorActive());           
    }

    public Command adjustIndex() {
        return Commands.run(() -> endEffectorSubsystem.setSpeed(-0.1))
            .until(() -> !endEffectorSubsystem.isBackPieceSensorActive())
            .andThen(() -> endEffectorSubsystem.setSpeed(0.1))
            .until(() -> endEffectorSubsystem.isBackPieceSensorActive());
    }

    /**
     * Spoofs a coral for sim testing
     * @param coral
     * @return
     */
    public Command coralButton(boolean coral){
        return Commands.run(() -> floorSubsystem.setCoral(coral));
    }

    // execute clean up
    public void cleanupSequence(){
        DataLogManager.log("Execute clean up");

        intakeFlag = false; 

        if(floorSubsystem.hasCoral()){
        DataLogManager.log("Got a piece -> intaking"); 

            elevatorSubsystem.setPosition(ElevatorConstants.Position.MIN)
                .deadlineFor(floorSubsystem.setPivotAngle(Rotation2d.fromDegrees(FloorConstants.Positions.ELEVATOR_ANGLE)))
                .andThen(floorSubsystem.setPivotAngle(FloorConstants.Positions.DEFAULT_ANGLE))
                .andThen(initialIndex())
                .andThen(adjustIndex())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .schedule();
        }
        else { 
            DataLogManager.log("No piece -> stowing");
            Commands.deadline(
                    elevatorSubsystem.setPosition(ElevatorConstants.Position.MIN)
                        .deadlineFor(floorSubsystem.setPivotAngle(Rotation2d.fromDegrees(FloorConstants.Positions.ELEVATOR_ANGLE)))
                        .andThen(floorSubsystem.setPivotAngle(FloorConstants.Positions.DEFAULT_ANGLE)),
                    floorSubsystem.runRollers(-1)
                ).schedule();
        }
    }
}
