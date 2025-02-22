package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstant;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX m_armKraken;
    private PositionDutyCycle m_pidPosition;
    private double setpoint = 0; // Stores the last commanded position
    private int state = 0;
    private ArmState currentState = ArmState.UNKNOWN;
    
    // 0 = base, 1/-1 = L1/Sad L1, 2/-2 = L2/Sad L2, 3/-3 = L3/Sad L3, 4/-4 = L4/Sad L4, 5/-5 = Source/Sad Source

    public ArmSubsystem() {
        m_armKraken = new TalonFX(ArmConstant.kArmMotorID, ArmConstant.kArmCANbus);

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = ArmConstant.kArmP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = ArmConstant.kArmI; // no output for integrated error
        slot0Configs.kD = ArmConstant.kArmD; // A velocity error of 1 rps results in 0.1 V output
        m_armKraken.getConfigurator().apply(slot0Configs);
        

        // current limit
        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = ArmConstant.kArmCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        m_armKraken.getConfigurator().apply(currentLimitConfigs);



        m_pidPosition = new PositionDutyCycle(0);
        m_armKraken.setNeutralMode(NeutralModeValue.Brake);
        resetEncoder();

    }

    public enum ArmState{
        LEVEL1, LEVEL2, LEVEL3, LEVEL4,
        SAD_LEVEL1, SAD_LEVEL2, SAD_LEVEL3, SAD_LEVEL4, 
        SOURCE,
        SAD_SOURCE,
        UNKNOWN
    }


    public ArmState getStateE() {
     return currentState;
    }

    public void setState(ArmState newState) {
        currentState = newState;
    }
    

     public void setArmAngle(double targetAngle) {

        double Rotations = (targetAngle/360) * ArmConstant.ArmGearRatio;

        setpoint = Math.min(Rotations, ArmConstant.kMaxAngle);


        if(setpoint !=  Rotations){
            System.out.println("Warning: Requested arm angle is out of bounds. Setting to " + setpoint + " rotations");
        }


        System.out.println("Setting elevator position to final " + Rotations + " rotations");

        m_armKraken.setControl(m_pidPosition.withPosition(setpoint));

    }

    public void setState(int newState){
        state = newState;
    }

    public int getState(){
        return state;
    }

    public void manualControl(double speed) {
            m_armKraken.set(speed);        
    }

    public double getArmAngle_Rotation() {
        return m_armKraken.getPosition().getValueAsDouble() / ArmConstant.ArmGearRatio;
    }

    public void resetEncoder() {
        m_armKraken.setPosition(0);
    }

    

    public void stop() {
        m_armKraken.set(0);
    }


    @Override
    public void periodic() {
    //    SmartDashboard.putNumber("Arm Angle (Rotations)", getArmAngle());
        SmartDashboard.putNumber("Arm Motor Output", m_armKraken.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Arm Degrees", getArmAngle_Rotation());
        
    }

}
