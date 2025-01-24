package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;



public class Test extends Command {
    private int counter;
  public Test() {
    addRequirements();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Test command initialized");
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Test command executed " + counter);
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Test command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Test command isFinished");
    if (counter >= 10) {
      return true;
    }
    return false;
  }

}