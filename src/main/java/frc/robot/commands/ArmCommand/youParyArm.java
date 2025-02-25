package frc.robot.commands.ArmCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class youParyArm extends Command {
    private final ArmSubsystem arm;
    

    public youParyArm(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
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
