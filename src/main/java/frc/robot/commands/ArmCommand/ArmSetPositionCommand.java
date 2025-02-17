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
        System.out.println("ElevatorSetPositionCommand initialized");
    }

    @Override
    public void execute(){
        arm.setArmAngle(targetPositionRotations);
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
