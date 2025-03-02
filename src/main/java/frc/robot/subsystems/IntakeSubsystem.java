package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstant;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX m_IntakeKraken;
    private CANrange CANrangeELeft;
    private CANrange CANrangeERight;

    private PositionDutyCycle m_pidPosition;
    private double setpoint = 0; // Stores the last commanded position

    public IntakeSubsystem() {
        m_IntakeKraken = new TalonFX(IntakeConstant.kIntakeMotorID, IntakeConstant.kIntakeCANbus);
        CANrangeELeft = new CANrange(IntakeConstant.kCANrange1ID, IntakeConstant.kIntakeCANbus);
        CANrangeERight = new CANrange(IntakeConstant.kCANrange2ID, IntakeConstant.kIntakeCANbus);
        CANrangeConfiguration configs = new CANrangeConfiguration();
        CANrangeELeft.getConfigurator().apply(configs);
        CANrangeERight.getConfigurator().apply(configs);

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.5;
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
        m_IntakeKraken.getConfigurator().apply(slot0Configs);

        // current limit
        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = IntakeConstant.kIntakeCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        m_IntakeKraken.getConfigurator().apply(currentLimitConfigs);

        m_pidPosition = new PositionDutyCycle(0);
        m_IntakeKraken.setNeutralMode(NeutralModeValue.Brake);
        resetEncoder();

    }

    public CANrange getLeftCaNrange(){
        return CANrangeELeft;
    }

    public CANrange getRIghtCaNrange(){
        return CANrangeERight;
    }
    public void holdPositionWrite(double setpoint){
        m_IntakeKraken.setControl(m_pidPosition.withPosition(setpoint));
    }

    public double READPositionPoint(){
        return setpoint; 
    }

    public void resetEncoder() {
        m_IntakeKraken.setPosition(0);
    }

    public void holdPositionStore(double setpoint){
        this.setpoint = setpoint;
    }

    public CANrange getCANrangeLeft() {
        return CANrangeELeft;
    }

    public CANrange getCANrangeRight() {
        return CANrangeERight;
    }


    public double getCurrentPosition_Rotations() {
        return m_IntakeKraken.getPosition().getValueAsDouble();
    }

    public void stop() {
        m_IntakeKraken.set(0);
    }

    public void feedWest(){
        m_IntakeKraken.set(0.3);

    }
    public void feedEast(){
        m_IntakeKraken.set(-0.3);

    }
    public void feedWest(double speed){
        m_IntakeKraken.set(speed);

    }
    public void feedEast(double speed){
        m_IntakeKraken.set(-speed);

    }
    public void manualControl(double speed) {
        m_IntakeKraken.set(speed);        
    }

    @Override
    public void periodic() {

    }
    
}
