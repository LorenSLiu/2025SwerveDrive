// package frc.robot.commands.ArmCommand;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.newElevatorSubsystem;

// public class ArmSetPositionCommand extends Command {
//     private final ArmSubsystem arm;
//     private final double targetPositionRotations;

//     public ArmSetPositionCommand(ArmSubsystem arm, double targetPosition) {
//         this.arm = arm;
//         this.targetPositionRotations = targetPosition;
//         addRequirements(arm);
//         System.out.println("ElevatorSetPositionCommand initialized");
//     }

//     @Override
//     public void initialize() {
//         arm.setArmAngle(targetPositionRotations);
//     }

//     @Override
//     public boolean isFinished() {
//         System.out.println(arm.getArmAngle());
//         if(Math.abs(arm.getArmAngle() - targetPositionRotations) < frc.robot.Constants.ArmConstant.TOLERANCE){
//             System.out.println("ElevatorSetPositionCommand isFinished");
//         }
//         return Math.abs(arm.getArmAngle() - targetPositionRotations) < frc.robot.Constants.ArmConstant.TOLERANCE;
//     }
// }
