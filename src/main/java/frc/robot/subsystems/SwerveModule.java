package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
@SuppressWarnings("all")

public class SwerveModule {
    private TalonFX m_drivingKraken;
    private TalonFX m_turningFalcon;

    private SwerveModuleState m_moduleState = new SwerveModuleState();//contain the speed and angle of the module

    public SwerveModule(int drivingKrakenID, int turningFalconID) {
        m_drivingKraken = new TalonFX(drivingKrakenID);
        m_turningFalcon = new TalonFX(turningFalconID);
        m_moduleState = new SwerveModuleState(1, new Rotation2d(Units.degreesToRadians(30)));

    }

    public SwerveModuleState setDesiredState(SwerveModuleState desiredState){
        m_moduleState = desiredState;
        return m_moduleState;
    }
    
    public SwerveModuleState getSwerveModuleState(){
        return m_moduleState;
    }
}
