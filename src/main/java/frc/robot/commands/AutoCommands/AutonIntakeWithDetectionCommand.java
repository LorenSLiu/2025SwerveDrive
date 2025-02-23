package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import static edu.wpi.first.units.Units.Centimeters;
import com.ctre.phoenix6.hardware.CANrange;

public class AutonIntakeWithDetectionCommand extends Command {
    private final IntakeSubsystem intake;
    private final CANrange CANrangeELeft;
    private final CANrange CANrangeERight;

    private boolean isSad;

    public AutonIntakeWithDetectionCommand(IntakeSubsystem intake, CANrange CANrangeLeft, CANrange CANrangeERight, boolean isSad){
        this.intake = intake;
        this.CANrangeELeft = CANrangeLeft;
        this.CANrangeERight = CANrangeERight;
        this.isSad = isSad;
        addRequirements(intake);
        System.out.println("Intake With Detection Command Initialized");
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        System.out.println("weird yayy");
        if(isSad){
            System.out.println("sad yayyyyy");
            intake.manualControl(0.3);
        }
        else{
            System.out.println("no sad yayyyyy");
            intake.manualControl(-0.3);
        }
    }

    @Override
    public boolean isFinished(){
        double distance = isSad 
        ? CANrangeERight.getDistance().getValue().in(Centimeters) 
        : CANrangeELeft.getDistance().getValue().in(Centimeters);

        if(distance <= 18){
            intake.stop();
            intake.holdPositionStore(intake.getCurrentPosition_Rotations());
            return true;
        }
        else{
            return false;
        }



        // //can you simplify the logic, if it's sad mode, we are constantly check for Right CANRange, and if it's not sad mode, we are constantly checking for Left CANRange, if the distance greater than 4, return false, else return true
        // if(isSad){
        //     if(CANrangeELeft.getDistance().getValue().in(Centimeters) > 5){
        //         return false;
        //     }
            
        // }
        // else if(!isSad){
        //     if(CANrangeELeft.getDistance().getValue().in(Centimeters) > 4){
        //         return false;
        // }
        // }
        // else{
        
        //     try {
        //         TimeUnit.MILLISECONDS.sleep(25);
        //     }
        //     catch(InterruptedException e){
        //         System.out.println("got interrupted!");
        //     }
        //     intake.stop();
        //     // new IntakeHoldPositionCommand(intake).schedule();


        //     return true;
        // }
        
    }
    
}
