package frc.robot.commands.ElevatorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetPositionCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double targetPosition;

    public ElevatorSetPositionCommand(ElevatorSubsystem elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
        System.out.println("ElevatorSetPositionCommand initialized");
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        System.out.println(elevator.getCurrentPosition());
        if(Math.abs(elevator.getCurrentPosition() - targetPosition) < frc.robot.Constants.ElevatorConstants.TOLERANCE){
            System.out.println("ElevatorSetPositionCommand isFinished");
        }
        return Math.abs(elevator.getCurrentPosition() - targetPosition) < frc.robot.Constants.ElevatorConstants.TOLERANCE;
    }
}
