package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX m_elevator;


    public ElevatorSubsystem(){
        m_elevator = new TalonFX(frc.robot.Constants.ElevatorConstants.kElevatorMotorID);
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = frc.robot.Constants.ElevatorConstants.kElevatorP;
        slot0Configs.kI = frc.robot.Constants.ElevatorConstants.kElevatorI;
        slot0Configs.kD = frc.robot.Constants.ElevatorConstants.kElevatorD;
        slot0Configs.kS = frc.robot.Constants.ElevatorConstants.kElevatorFF;

        var cuurentLimitConfigs = new CurrentLimitsConfigs();
        cuurentLimitConfigs.StatorCurrentLimit = frc.robot.Constants.ElevatorConstants.kElevatorCurrentLimit;
        cuurentLimitConfigs.StatorCurrentLimitEnable = true;

        m_elevator.getConfigurator().apply(slot0Configs);
        m_elevator.getConfigurator().apply(cuurentLimitConfigs);

        m_elevator.setNeutralMode(NeutralModeValue.Brake);

        //set if motor is inverted
        m_elevator.setInverted(false);
        
    }

    public double getElevatorHeightMeters(){
        return m_elevator.getPosition().getValueAsDouble() * frc.robot.Constants.ElevatorConstants.METERS_PER_ROTATION;
    }

    public double convertMetersToTicks(double meters){
        // i mean it's just my personal preference to do with ticks, but you can do it with rotations
        return meters / frc.robot.Constants.ElevatorConstants.METERS_PER_TICK;
    }

    public double convertTicksToRotations(double ticks){
        return ticks / frc.robot.Constants.ElevatorConstants.ENCODER_TICKS_PER_REV;
    }

    public double convertMetersToRotations(double meters){
        return meters / frc.robot.Constants.ElevatorConstants.METERS_PER_ROTATION;
    }

    public void setElevatorHeight(double targetMeters){

        double TargetRotations = convertMetersToRotations(targetMeters);//convert meters to ticks, then convert ticks to rotations
        System.out.println("Target Rotations: " + TargetRotations);
        m_elevator.setPosition(TargetRotations);
    }


}
