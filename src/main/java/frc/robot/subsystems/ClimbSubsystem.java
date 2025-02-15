package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstant;

public class ClimbSubsystem extends SubsystemBase {
    private TalonFX m_climbKraken;
    private PositionDutyCycle m_pidPosition;
    private double setpoint = 0; // Stores the last commanded position

    public ClimbSubsystem() {
        m_climbKraken = new TalonFX(ClimbConstant.kClimbMotorID, ClimbConstant.kClimbCANbus);

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.5;
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
        m_climbKraken.getConfigurator().apply(slot0Configs);

        // current limit
        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = 40;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        m_climbKraken.getConfigurator().apply(currentLimitConfigs);

        m_pidPosition = new PositionDutyCycle(0);
        m_climbKraken.setNeutralMode(NeutralModeValue.Brake);
        resetEncoder();

    }

    public void resetEncoder() {
        m_climbKraken.setPosition(0);
    }

    

    public void stop() {
        m_climbKraken.set(0);
    }

    public void expand(){
        m_climbKraken.set(-1);

    }
    public void retract(){
        m_climbKraken.set(1);

    }
    public void manualControl(double speed) {
        m_climbKraken.set(speed);        
    }

    public double ClimbAngle_Rotation() {
        return m_climbKraken.getPosition().getValueAsDouble() / 152.4444444444;
    }

    


    @Override
    public void periodic() {

    }

}
