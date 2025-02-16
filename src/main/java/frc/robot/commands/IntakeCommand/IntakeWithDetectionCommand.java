package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

import static edu.wpi.first.units.Units.Inches;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.hardware.CANrange;

public class IntakeWithDetectionCommand extends Command {
    private final IntakeSubsystem intake;
    private final CANrange CANrangeE;

    public IntakeWithDetectionCommand(IntakeSubsystem intake, CANrange CANrangeE){
        this.intake = intake;
        this.CANrangeE = CANrangeE;
        addRequirements(intake);
        System.out.println("Intake With Detection Command Initialized");
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        intake.feedWest();
    }

    @Override
    public boolean isFinished(){
        //hiiiiiiiiiiiiiiiiiiiiii
        if(CANrangeE.getDistance().getValue().in(Inches) > 4){
            return false;
        }
        else{
            try {TimeUnit.MILLISECONDS.sleep(100);}
            catch(InterruptedException e){System.out.println("got interrupted!");}
            intake.stop();
            return true;
        }
    }
    
}
