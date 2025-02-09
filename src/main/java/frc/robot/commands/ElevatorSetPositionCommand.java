package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetPositionCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double targetPosition;

    public ElevatorSetPositionCommand(ElevatorSubsystem elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getCurrentPosition() - targetPosition) < frc.robot.Constants.ElevatorConstants.TOLERANCE;
    }
}
