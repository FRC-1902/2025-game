package frc.robot.commands.algaeIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeOuttakeFactory {
  private AlgaeIntakeSubsystem algaeIntake;

    public AlgaeOuttakeFactory(AlgaeIntakeSubsystem algaeIntake) {
      this.algaeIntake = algaeIntake;
    }

    public Command algaeOuttakeSequence() {
      return new SequentialCommandGroup(
        new InstantCommand(() -> algaeIntake.setAngle(Rotation2d.fromDegrees(40))),
        new WaitCommand(.3),
        new AlgaeOuttakeCommand(algaeIntake)
      );
    }
}

