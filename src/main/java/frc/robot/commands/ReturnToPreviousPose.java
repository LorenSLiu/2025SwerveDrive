package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;

public class ReturnToPreviousPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private  Pose2d targetPose;
    private final PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    private Command followCommand;
    private boolean isFinished = false;

     // Constructor

    public ReturnToPreviousPose(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.targetPose = RecordLastPose.getRecordedPose(); // Get the recorded pose from the RecordLastPose command
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Build and schedule the command to follow a path to the target pose.
    }

    @Override
    public void execute() {
        // Continuously execute the follow command until it is finished.
        if(targetPose != null) {
            isFinished = true;
            targetPose = RecordLastPose.getRecordedPose();
            System.out.println("Target Pose: " + targetPose);
            followCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
        }

    }

    @Override
    public void end(boolean interrupted) {
        //run the path and schedule it
        if (followCommand != null) {
            followCommand.schedule();
        }

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
