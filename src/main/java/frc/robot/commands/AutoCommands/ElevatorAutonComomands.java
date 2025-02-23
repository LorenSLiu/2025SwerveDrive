package frc.robot.commands.AutoCommands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorAutonComomands extends Command {
    private final ElevatorSubsystem elevator;
    private Distance targetPosition;
    

    public ElevatorAutonComomands(ElevatorSubsystem elevator, Distance targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("Elevator Target Position: "  + targetPosition);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Elevator Distance Error", Math.abs(elevator.getCurrentPosition_Meters() - targetPosition.in(Meters)));
        elevator.setElevatorPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
