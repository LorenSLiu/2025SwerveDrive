// package frc.robot.commands.ElevatorCommand;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ElevatorSubsystem;

// public class ElevatorManualCommand extends Command {
//     private final ElevatorSubsystem elevator;
//     private final Supplier<Double> speedSupplier;

//     public ElevatorManualCommand(ElevatorSubsystem elevator, Supplier<Double> speedSupplier) {
//         this.elevator = elevator;
//         this.speedSupplier = speedSupplier;
//         addRequirements(elevator);
//     }

//     @Override
//     public void execute() {
//         elevator.manualControl(speedSupplier.get());
// //        elevator.manualControl(speedSupplier.());
//     }

//     @Override
//     public void end(boolean interrupted) {
//         elevator.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
