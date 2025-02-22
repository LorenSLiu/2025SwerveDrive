package frc.robot.commands.ArmCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetPositionCommand extends Command {
    private final ArmSubsystem arm;
    private final double targetPositionRotations;

    public ArmSetPositionCommand(ArmSubsystem arm, double targetPosition) {
        this.arm = arm;
        this.targetPositionRotations = targetPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        System.out.println("Arm initialized");        
        arm.setArmAngle(targetPositionRotations);

    }

    @Override
    public void execute(){
//        System.out.println("--------------------------Target"+targetPositionRotations);
        System.out.println("---------------------------current"+arm.getArmAngle_Rotation()+"my ideal postition: "+targetPositionRotations);
    }

    @Override
    public boolean isFinished() {
        return false;
        // System.out.println(arm.getArmAngle());
        // if(Math.abs(arm.getArmAngle() - targetPositionRotations) < frc.robot.Constants.ArmConstant.TOLERANCE){
        //     System.out.println("ElevatorSetPositionCommand isFinished");
        // }
        // return Math.abs(arm.getArmAngle() - targetPositionRotations) < frc.robot.Constants.ArmConstant.TOLERANCE;
    }
}
