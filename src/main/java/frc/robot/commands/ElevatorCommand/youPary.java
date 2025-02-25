package frc.robot.commands.ElevatorCommand;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class youPary extends Command {
    private final ElevatorSubsystem elevator;
    

    public youPary(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
