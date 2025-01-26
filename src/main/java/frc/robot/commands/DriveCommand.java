// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.Utils.MathUtils;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;


// public class DriveCommand extends Command {
//     private final SwerveSubsystem m_swerveSubsystem;
//     private final double m_forwardSpeed;
//     private final double m_sidewaySpeed;
//     private final double m_rotationSpeed;
//     private final ChassisSpeeds m_chassisSpeeds;

//     private static final double kTolerance = 0.1; // Tolerance for module speed
//     private static final double kToleranceRotation = 0.1; // Tolerance for module speed
    
//     public DriveCommand(SwerveSubsystem swerveSubsystem, double forwardSpeed, double sidewaySpeed, double rotationSpeed) {
//         m_swerveSubsystem = swerveSubsystem;
//         m_forwardSpeed = forwardSpeed;
//         m_sidewaySpeed = sidewaySpeed;
//         m_rotationSpeed = rotationSpeed;
//         m_chassisSpeeds = new ChassisSpeeds(m_forwardSpeed, m_sidewaySpeed, m_rotationSpeed);
//         addRequirements(m_swerveSubsystem);
//         // Use addRequirements() here to declare subsystem dependencies.
//       }
//       public boolean isWithinTolerance() {
//         // Check if all module speeds are within tolerance
//         return MathUtils.isWithinTolerance(m_swerveSubsystem.getForwardSpeed(), m_forwardSpeed, kTolerance) &&
//                MathUtils.isWithinTolerance(m_swerveSubsystem.getSidewaySpeed(), m_sidewaySpeed, kTolerance) &&
//                MathUtils.isWithinTolerance(m_rotationSpeed, 0, kToleranceRotation);
//     }
    
//       // Called when the command is initially scheduled.
//       @Override
//       public void initialize() {
//         m_swerveSubsystem.setChassisSpeeds(m_chassisSpeeds);
//         System.out.println("Test command initialized");
//       }
    
//       // Called every time the scheduler runs while the command is scheduled.
//       @Override
//       public void execute() {
//         System.out.println("Test command executed ");
//       }
    
//       // Called once the command ends or is interrupted.
//       @Override
//       public void end(boolean interrupted) {
//         if(interrupted){
//           m_swerveSubsystem.zeroChassisSpeeds();
//         }
//       }
    
//       // Returns true when the command should end.
//       @Override
//       public boolean isFinished() {
//         //find a way that when module speed = desired speed, return true
//         if (isWithinTolerance()) {
//           System.out.println("Test command isFinished");
//           return true;
//         }
//         return false;
//       }
    
// }
