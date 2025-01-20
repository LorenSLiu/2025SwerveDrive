package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
@SuppressWarnings("all")

public class SwerveModule {
    private TalonFX m_drivingKraken;
    private TalonFX m_turningFalcon;

    private final double m_turningEncoderOffset;//check the offset value
    private final AnalogInput m_absoluteEncoder;//check the port number
    private final boolean m_turningEncoderReversed;//check if the encoder is reversed
    private final double m_absoluteEncoderOffsetRadians;//check the offset value

    private final CANcoder m_turningEncoder;

    PIDController m_drivePIDController = new PIDController(0.1, 0, 0);//Double check the PID values
    PIDController m_turnPIDController = new PIDController(0.1, 0, 0);//same as above

    private SwerveModuleState m_moduleCurrentState;//contain the speed and angle of the module
    private SwerveModuleState m_moduleDesiredState;//contain the speed and angle of the module

    public SwerveModule(int drivingKrakenID, int turningFalconID, double turningEncoderOffset, int absoluteEncoderID, double absoluteEncoderOffsetRadians, boolean turningEncoderReversed){ 
        m_drivingKraken = new TalonFX(drivingKrakenID);
        m_turningFalcon = new TalonFX(turningFalconID);

        m_turningEncoder = new CANcoder(m_turningFalcon.getDeviceID());//check if the ID is same as the turningFalconID

        m_moduleCurrentState = new SwerveModuleState();
        m_moduleDesiredState = new SwerveModuleState();

        m_turningEncoderOffset = turningEncoderOffset;
        m_absoluteEncoder = new AnalogInput(absoluteEncoderID);
        m_absoluteEncoderOffsetRadians = absoluteEncoderOffsetRadians;
        m_turningEncoderReversed = turningEncoderReversed;
    }

    public SwerveModuleState setDesiredState(SwerveModuleState newState){
        m_moduleDesiredState = newState;
        return m_moduleDesiredState;
    }
    
    public SwerveModuleState getSwerveModuleState(){
        return m_moduleCurrentState;
    }

    public void periodic(){

    }
}
