package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;

public class AutoAlignSim extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private double tagID = -1;
  private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric();
  private Timer simTimer; // added for simulation timing
  private double simDuration; // added for simulation duration
  private static Pose2d startingPose = new Pose2d(); // added for simulation starting pose


  public AutoAlignSim(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(Constants.AutonConstants.X_REEF_ALIGNMENT_P, 0.01, 0);  // Vertical movement
    yController = new PIDController(Constants.AutonConstants.Y_REEF_ALIGNMENT_P, 0.01, 0);  // Horitontal movement
    rotController = new PIDController(Constants.AutonConstants.ROT_REEF_ALIGNMENT_P, 0.01, 0);  // Rotation


    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    startingPose = drivebase.getState().Pose; // Get the starting pose of the robot
    // Begin simulation: drive forward for a random duration between 1 and 3 seconds.
    simTimer = new Timer();
    simTimer.start();
    simDuration = Math.random() * 2 + 1; // random duration in seconds
    SmartDashboard.putNumber("SimDuration", simDuration);
    // Mark auto align as active
    SmartDashboard.putBoolean("AutoAlignActive", true);
  }

  public static Pose2d getStartingPose2d(){
    return startingPose;
  }

  @Override
  public void execute() {
    if (simTimer.get() < simDuration) {
      drivebase.setControl(
        m_driveRequest.withVelocityX(-1)  // forward movement
                     .withVelocityY(0)
                     .withRotationalRate(0)
      );
      SmartDashboard.putBoolean("AutoAlignActive", true);
    } else {
      drivebase.setControl(
        m_driveRequest.withVelocityX(0)
                     .withVelocityY(0)
                     .withRotationalRate(0)
      );
      SmartDashboard.putBoolean("AutoAlignActive", false);
    }
    SmartDashboard.putNumber("SimTimer", simTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.setControl(m_driveRequest.withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(0));
    // Ensure SmartDashboard is updated at the end
    SmartDashboard.putBoolean("AutoAlignActive", false);
    PPHolonomicDriveController.clearFeedbackOverrides();
    
    

    // Overrride feedback Pathplanner
    
    
  }
    

  @Override
  public boolean isFinished() {
    // Command finishes when the simulation duration elapses
    return simTimer.hasElapsed(simDuration);
  }
}