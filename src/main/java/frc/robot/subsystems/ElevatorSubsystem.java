package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.units.measure.Distance;

@SuppressWarnings("all")//hater just be hating
public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX m_elevatorKraken;
    private TalonFX m_elevatorKrakenFollower;
    private PositionDutyCycle m_pidPosition;
    private final double kElevatorMinRotation = 0;
    private final double kElevatorMaxRotation = (3 * ElevatorConstants.kMaxHeight.in(Meters)) / (2* Math.PI * ElevatorConstants.SprocketRadius.in(Meters));

    private double setpoint = 0; // Stores the last commanded position

    public ElevatorSubsystem() {
        m_elevatorKraken = new TalonFX(ElevatorConstants.kElevatorMotorID, ElevatorConstants.kElevatorCANbus);
        m_elevatorKrakenFollower = new TalonFX(ElevatorConstants.kElevatorMotorFollowerID,ElevatorConstants.kElevatorCANbus);

        var talonFXConfigs = new TalonFXConfiguration();
        //asdf

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = ElevatorConstants.kElevatorP; 
        slot0Configs.kI = ElevatorConstants.kElevatorI; 
        slot0Configs.kD = ElevatorConstants.kElevatorD; 
        slot0Configs.kG = ElevatorConstants.kElevatorG;
        m_elevatorKraken.getConfigurator().apply(slot0Configs);

        var cuurentLimitConfigs = new CurrentLimitsConfigs();
        cuurentLimitConfigs.StatorCurrentLimit = frc.robot.Constants.ElevatorConstants.kElevatorCurrentLimit;
        cuurentLimitConfigs.StatorCurrentLimitEnable = true;
        m_elevatorKraken.getConfigurator().apply(cuurentLimitConfigs);

        m_pidPosition = new PositionDutyCycle(0);
        m_elevatorKraken.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorKrakenFollower.setControl(new Follower(m_elevatorKraken.getDeviceID(), false));

        resetEncoder();

    }

    public void setElevatorPosition(Distance targetHeight) {

        double Rotations = (3 * targetHeight.in(Meters)) / (2* Math.PI * ElevatorConstants.SprocketRadius.in(Meters));

        setpoint = Math.max(kElevatorMinRotation, Math.min(Rotations, kElevatorMaxRotation));
        if(setpoint !=  Rotations){
            System.out.println("Warning: Requested elevator position is out of bounds. Setting to " + setpoint + " rotations");
            DriverStation.reportWarning("Requested elevator position is out of bounds. Setting to " + setpoint + " rotations", true);
        }

        System.out.println("Setting elevator position to final " + Rotations + " rotations");

        m_elevatorKraken.setControl(m_pidPosition.withPosition(setpoint));

    }

    public void manualControl(double speed) {
            m_elevatorKraken.set(speed); 
    }

    public void manualControlHold(){
        m_elevatorKraken.setControl(m_pidPosition.withPosition(m_elevatorKraken.getPosition().getValueAsDouble()));
    }


    public double getCurrentPosition_Rotations() {
        return m_elevatorKraken.getPosition().getValueAsDouble();
    }

    public double getCurrentPosition_Meters() {
        return (2 * Math.PI * ElevatorConstants.SprocketRadius.in(Meters) * m_elevatorKraken.getPosition().getValueAsDouble())/3;
    }

    

    public void resetEncoder() {
        m_elevatorKraken.setPosition(0);
    }

    public void stop() {
        m_elevatorKraken.set(0);
    }

 

}
