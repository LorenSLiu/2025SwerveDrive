package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private double m_targetHeight;
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetHeight) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_targetHeight = targetHeight;
        addRequirements(m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Elevator command initialized");
        m_elevatorSubsystem.setElevatorHeight(m_targetHeight);
    }

    @Override
    public void execute() {
        System.out.println("Elevator command executed ");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Elevator command ended");
    }

    @Override
    public boolean isFinished() {
        // System.out.println("Test command isFinished");
        // if (counter >= 10) {
        //     return true;
        // }
        System.out.println("Elevator command isFinished");
        return false;
    }

}