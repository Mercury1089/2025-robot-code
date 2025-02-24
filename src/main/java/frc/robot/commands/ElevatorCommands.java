package frc.robot.commands;

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
                new RunCommand(() -> elevator.setPosition(ElevatorPosition.SAFE_POS), elevator)
            ).until(() -> articulator.isAtPosition(ArticulatorPosition.IN)),
            new RunCommand(() -> elevator.setPosition(ElevatorPosition.HOME), elevator)
        );
    }

    public static Command getArticulatorOutCommand(Elevator elevator, AlgaeArticulator articulator, /* make this a supplier? */RobotZone zone) {
        return new SequentialCommandGroup(
            new ConditionalCommand(
                new RunCommand(() -> elevator.setPosition(ElevatorPosition.L2_ALGAE), elevator), 
                new RunCommand(() -> elevator.setPosition(ElevatorPosition.L3_ALGAE), elevator), 
                () -> zone == RobotZone.BARGE || zone == RobotZone.CLOSE_LEFT || zone == RobotZone.CLOSE_RIGHT).until(() -> elevator.isInPosition()),
            new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator)
        );
    }

    public static Command getAlgaeElevatorCommand(Elevator elevator, AlgaeArticulator articulator, RobotZone zone, AlgaeIntake intake) {
        return new ParallelCommandGroup(
            getArticulatorOutCommand(elevator, articulator, zone),
            new RunCommand(() -> intake.intakeAlgae(), intake)
        );
    }
    
}
