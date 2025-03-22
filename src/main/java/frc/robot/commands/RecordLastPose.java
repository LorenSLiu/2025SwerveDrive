package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class RecordLastPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d lastPose;
    private static Pose2d recordedPose; // accessible after command ends

    public RecordLastPose(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Capture initial pose
        lastPose = drivetrain.getState().Pose;
    }

    @Override
    public void execute() {
        // Continuously update the last pose
        lastPose = drivetrain.getState().Pose;
    }

    @Override
    public void end(boolean interrupted) {
        // Store the final pose
        recordedPose = lastPose;
    }

    // Getter to access the recorded pose after command completion
    public static Pose2d getRecordedPose() {
        return recordedPose;
    }

    @Override
    public boolean isFinished() {
        // Run indefinitely until interrupted so the most recent pose is available when the command ends.
        return false;
    }
}
