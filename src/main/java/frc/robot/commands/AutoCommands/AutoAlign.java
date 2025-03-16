package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private double tagID = -1;
  private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();


  public AutoAlign(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(Constants.AutonConstants.X_REEF_ALIGNMENT_P, 0.01, 0);  // Vertical movement
    yController = new PIDController(Constants.AutonConstants.Y_REEF_ALIGNMENT_P, 0.01, 0);  // Horitontal movement
    rotController = new PIDController(Constants.AutonConstants.ROT_REEF_ALIGNMENT_P, 0.01, 0);  // Rotation

    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.AutonConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.AutonConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.AutonConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.AutonConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.AutonConstants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.AutonConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.AutonConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-happy");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-happy") && LimelightHelpers.getFiducialID("limelight-happy") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-happy");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = -xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);

      drivebase.setControl(
        m_driveRequest.withVelocityX(xSpeed)
           .withVelocityY(ySpeed)
           .withRotationalRate(rotValue)
     );
      

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.setControl(m_driveRequest.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setControl(m_driveRequest.withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(0));
    }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.AutonConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.AutonConstants.POSE_VALIDATION_TIME);
  }
}