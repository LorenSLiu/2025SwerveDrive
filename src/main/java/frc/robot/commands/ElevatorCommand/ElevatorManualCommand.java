package frc.robot.commands.ElevatorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManualCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier speedSupplier;

    public ElevatorManualCommand(ElevatorSubsystem elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.manualControl(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
