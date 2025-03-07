package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.AlgaeArticulator;
import frc.robot.subsystems.elevator.AlgaeIntake;
import frc.robot.subsystems.elevator.AlgaeArticulator.ArticulatorPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.util.ReefscapeUtils.RobotZone;

public class ElevatorCommands {


    public static Command getHomeCommand(Elevator elevator, AlgaeArticulator articulator){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator),
                new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.SAFE_POS), elevator)
            ).until(() -> articulator.isAtPosition(ArticulatorPosition.IN)),
            new RunCommand(() -> elevator.setPosition(() ->ElevatorPosition.HOME), elevator)
        );
    }

    public static Command getAlgaeRemovalCommand(Elevator elevator, AlgaeArticulator articulator, Supplier<RobotZone> zone) {
        return new ParallelCommandGroup(
            new ConditionalCommand(
                new RunCommand(() -> elevator.setPosition(() ->ElevatorPosition.L2_ALGAE), elevator), 
                new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.L3_ALGAE), elevator), 
                () -> zone.get() == RobotZone.BARGE || zone.get() == RobotZone.CLOSE_LEFT || zone.get() == RobotZone.CLOSE_RIGHT),
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator)
        );
    }

    public static Command getAlgaeElevatorCommand(Elevator elevator, AlgaeArticulator articulator, Supplier<RobotZone> zone, AlgaeIntake intake) {
        return new ParallelCommandGroup(
            getAlgaeRemovalCommand(elevator, articulator, zone),
            new RunCommand(() -> intake.intakeAlgae(), intake)
        );
    }
    
}
