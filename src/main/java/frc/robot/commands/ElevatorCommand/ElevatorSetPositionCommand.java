package frc.robot.commands.ElevatorCommand;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorSetPositionCommand extends Command {
    private final ElevatorSubsystem elevator;
    private Distance targetPosition;
    

    public ElevatorSetPositionCommand(ElevatorSubsystem elevator, Distance targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
        System.out.println("ElevatorSetPositionCommand initialized");
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(targetPosition, ElevatorConstants.GEAR_RATIO, ElevatorConstants.PULLEY_DIAMETER);
    }

    @Override
    public boolean isFinished() {
        System.out.println(elevator.getCurrentPosition());
        if(Math.abs(elevator.getCurrentPosition() - targetPosition.in(Meters)) < frc.robot.Constants.ElevatorConstants.TOLERANCE){
            System.out.println("ElevatorSetPositionCommand isFinished");
        }
        return Math.abs(elevator.getCurrentPosition() - targetPosition.in(Meters)) < frc.robot.Constants.ElevatorConstants.TOLERANCE;
    }
}
