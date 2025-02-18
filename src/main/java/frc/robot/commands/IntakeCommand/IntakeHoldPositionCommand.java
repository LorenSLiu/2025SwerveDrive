package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.hardware.CANrange;

public class IntakeHoldPositionCommand extends Command {
    private final IntakeSubsystem intake;
    private final double position;

    public IntakeHoldPositionCommand(IntakeSubsystem intake, double position){
        this.intake = intake;
        this.position = position;
        addRequirements(intake);
        System.out.println("Intake H O L D  P O S I T I O N");
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
       intake.holdPosition(position);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
