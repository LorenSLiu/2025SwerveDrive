package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
  private CommandSwerveDrivetrain drivebase;
  
  private Command pathCommand;
  private Pose2d targetPose;
  private Pose2d robotPose;
  private double distanceAway = -0.55;
  
  /** Creates a new SingleTagAlign. */
  public AutoAlign(CommandSwerveDrivetrain drivebase) {
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Using the current field position where the tag is detected
    Pose2d selectedPosition = determineTargetPosition();
    
    // Calculate target pose - offset from the tag position
    targetPose = new Pose2d(
        Math.cos(selectedPosition.getRotation().getRadians()) * distanceAway
            - Math.sin(selectedPosition.getRotation().getRadians())
                * Constants.AutonConstants.X_SETPOINT_REEF_ALIGNMENT
            + selectedPosition.getTranslation().getX(),
        Math.sin(selectedPosition.getRotation().getRadians()) * distanceAway
            + Math.cos(selectedPosition.getRotation().getRadians())
                * Constants.AutonConstants.X_SETPOINT_REEF_ALIGNMENT
            + Constants.AutonConstants.Y_SETPOINT_REEF_ALIGNMENT,
        selectedPosition.getRotation());

    SmartDashboard.putNumber("AutoLineup/Target Pose X", targetPose.getX());
    SmartDashboard.putNumber("AutoLineup/Target Pose Y", targetPose.getY());
    SmartDashboard.putNumber("AutoLineup/Target Pose Rot", targetPose.getRotation().getDegrees());

    // Use PathPlanner to generate a path to the target pose
    pathCommand = AutoBuilder.pathfindToPose(targetPose, new PathConstraints(1, 1, 180, 180));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update robot pose with vision if tag is visible
    if (LimelightHelpers.getFiducialID("limelight-happy") != -1) {
      LimelightHelpers.PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-happy");
      if (poseEst.tagCount > 0) {
        robotPose = poseEst.pose;
        drivebase.addVisionMeasurement(robotPose, poseEst.timestampSeconds);
        SmartDashboard.putNumber("AutoLineup/robotPose X", robotPose.getX());
        SmartDashboard.putNumber("AutoLineup/robotPose Y", robotPose.getY());
      }
    }
    
    // Continue following the path
    pathCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
  
  // Helper method to determine target position based on visible tag
  private Pose2d determineTargetPosition() {
    // Default to current robot pose if no tag is visible
    Pose2d currentPose = drivebase.getState().Pose;
    
    // Try to get pose from Limelight
    if (LimelightHelpers.getFiducialID("limelight-happy") != -1) {
      LimelightHelpers.PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-happy");
      if (poseEst.tagCount > 0) {
        // Use tag-based estimation if available
        return poseEst.pose;
      }
    }
    
    return currentPose;
  }
}
