package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
@SuppressWarnings("all")//hater just be hating

public class SwerveModule {
    private TalonFX m_drivingKraken;
    private TalonFX m_turningFalcon;

    private final double m_absoluteEncoderOffsetRadians;//check the offset value

    private final CANcoder m_turningEncoder;
    
    private VelocityDutyCycle m_velocityDutyCycle;

    private SwerveModuleState m_moduleCurrentState;//contain the speed and angle of the module
    private SwerveModuleState m_moduleDesiredState;//contain the speed and angle of the module

    public SwerveModule(int drivingKrakenID, int turningFalconID, int absoluteEncoderID, double absoluteEncoderOffsetRadians){ 
        m_drivingKraken = new TalonFX(drivingKrakenID,"driveTrainCANivore");
        m_turningFalcon = new TalonFX(turningFalconID,"driveTrainCANivore");

        m_turningEncoder = new CANcoder(absoluteEncoderID);

        m_moduleCurrentState = new SwerveModuleState();
        m_moduleDesiredState = new SwerveModuleState();
        
        m_absoluteEncoderOffsetRadians = absoluteEncoderOffsetRadians;//Todo: Get the offsest angles

        m_velocityDutyCycle = new VelocityDutyCycle(0);

        var drivingSlot0Configs = new Slot0Configs();
        drivingSlot0Configs.kS = ModuleConstants.kDrivingFF; // Add 0.25 V output to overcome static friction
        drivingSlot0Configs.kP = ModuleConstants.kDrivingP; 
        drivingSlot0Configs.kI = ModuleConstants.kDrivingI; //
        drivingSlot0Configs.kD = ModuleConstants.kDrivingD; //

        var drivingCurrentConfig = new CurrentLimitsConfigs();
        drivingCurrentConfig.StatorCurrentLimit = frc.robot.Constants.ModuleConstants.kDrivingMotorCurrentLimit;
        drivingCurrentConfig.StatorCurrentLimitEnable = true;

        m_drivingKraken.getConfigurator().apply(drivingSlot0Configs);
        m_drivingKraken.getConfigurator().apply(drivingCurrentConfig);


        //following are the turning motor configurations
        var turningSlot0Configs = new Slot0Configs();
        turningSlot0Configs.kS = ModuleConstants.kTurningFF; // Add 0.25 V output to overcome static friction
        turningSlot0Configs.kP = ModuleConstants.kTurningP; // A position error of 2.5 rotations results in 12 V output
        turningSlot0Configs.kI = ModuleConstants.kTurningI; // no output for integrated error
        turningSlot0Configs.kD = ModuleConstants.kTurningD; // A velocity error of 1 rps results in 0.1 V output
  
      
        var turningCurrentConfig = new CurrentLimitsConfigs();
        turningCurrentConfig.StatorCurrentLimit = ModuleConstants.kTurningMotorCurrentLimit;
        turningCurrentConfig.StatorCurrentLimitEnable = true;
  
        m_turningFalcon.getConfigurator().apply(turningSlot0Configs);
        m_turningFalcon.getConfigurator().apply(turningCurrentConfig);

        m_drivingKraken.setPosition(0);
        m_drivingKraken.setNeutralMode(NeutralModeValue.Brake);

    }

    public SwerveModuleState setDesiredState(SwerveModuleState moduleDesiredState){
        m_moduleDesiredState = moduleDesiredState;
        m_moduleDesiredState.optimize(moduleDesiredState.angle);
        if(moduleDesiredState.angle.getDegrees() != m_moduleDesiredState.angle.getDegrees()){
            System.out.println("Subsystem>SwerveModule>setDesiredState(): Angle Before optimize: " + moduleDesiredState.angle);
            System.out.println("Subsystem>SwerveModule>setDesiredState(): Angle After optimize: " + m_moduleDesiredState.angle);
        }

        //System.out.println("optimized speed: "+m_moduleDesiredState.speedMetersPerSecond);
        m_drivingKraken.setControl(m_velocityDutyCycle.withVelocity(m_moduleDesiredState.speedMetersPerSecond * ModuleConstants.kDrivingEncoderVelocityFactor));
        //m_drivingKraken.setControl(m_velocityDutyCycle.withVelocity(m_moduleDesiredState.speedMetersPerSecond));//tf why would i do this, need to figure out the unit for it
        m_turningFalcon.setControl(m_velocityDutyCycle.withVelocity(m_moduleDesiredState.angle.getRadians() * ModuleConstants.kTurningEncoderVelocityFactor));

        return m_moduleDesiredState;
    }
    
    public SwerveModuleState getSwerveModuleState(){
        return m_moduleCurrentState;
    }

    public void periodic(){
        SmartDashboard.putNumber("Driving Kraken velocity CAN ID: "+m_drivingKraken.getDeviceID(), m_drivingKraken.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turning Falcon velocity CAN ID: "+m_turningFalcon.getDeviceID(), m_turningFalcon.getVelocity().getValueAsDouble());
    }
}
