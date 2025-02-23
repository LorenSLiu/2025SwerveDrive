package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;



public class IntakeHoldPositionCommand extends Command {
    private final IntakeSubsystem intake;

    public IntakeHoldPositionCommand(IntakeSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
        System.out.println("Intake HOLD initialized");
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        System.out.println("HOLDING HOLDING WOWW");
       intake.holdPositionWrite(intake.READPositionPoint());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
