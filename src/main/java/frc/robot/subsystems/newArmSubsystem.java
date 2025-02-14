package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstant;
import frc.robot.Constants.ElevatorConstants;

public class newArmSubsystem extends SubsystemBase {
    private TalonFX m_armKraken;
    private final MotionMagicVoltage motionMagicControl;

    public newArmSubsystem() {
        m_armKraken = new TalonFX(ArmConstant.kArmMotorID, ArmConstant.kArmCANbus);

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = ArmConstant.kArmP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = ArmConstant.kArmI; // no output for integrated error
        slot0Configs.kD = ArmConstant.kArmD; // A velocity error of 1 rps results in 0.1 V output
        m_armKraken.getConfigurator().apply(talonFXConfigs);

        // current limit
        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = ArmConstant.kArmCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        m_armKraken.getConfigurator().apply(currentLimitConfigs);

        m_armKraken.setNeutralMode(NeutralModeValue.Brake);
        motionMagicControl = new MotionMagicVoltage(0);
        resetEncoder();

        // set if motor is inverted
        // m_elevator.setInverted(false);

    }

    //  public void setArmPosition(Angle targetHeight) {

    //     double Rotations = (3 * targetHeight.in(Meters)) / (2* Math.PI * ElevatorConstants.SprocketRadius.in(Meters));

    //     setpoint = Math.max(kElevatorMinRotation, Math.min(Rotations, kElevatorMaxRotation));
    //     if(setpoint !=  Rotations){
    //         System.out.println("Warning: Requested elevator position is out of bounds. Setting to " + setpoint + " rotations");
    //         DriverStation.reportWarning("Requested elevator position is out of bounds. Setting to " + setpoint + " rotations", true);
    //     }

    //     System.out.println("Setting elevator position to final " + Rotations + " rotations");

    //     m_elevatorKraken.setControl(m_pidPosition.withPosition(setpoint));

    // }

    public void resetEncoder() {
        m_armKraken.setPosition(0);// know later for the encoder the reading is different from the actaul value as
                                   // there's a gear ratio change
    }

    //this is for later if we are dealing with encoder in a different starting position
    // public void resetEncoder() {
    //     // Assume starting position is the lowest angle (adjust if needed)
    //     m_armKraken.setPosition(ArmConstant.kStartingAngle * ArmConstant.ArmGearRatio);
    // }
    

    public void stop() {
        m_armKraken.set(0);
    }

    /**
     * Get the current arm angle in rotations.
     * 
     * @return the arm angle in rotations.
     */
    //public double getArmAngle() {
        //return m_armKraken.getPosition().getValueAsDouble() / ArmConstant.ArmGearRatio;
    //}

    public void setArmAngle(double targetRotations) {
        //i was thingking aobut the optimization of the angle but i think it's not needed
        // double currentAngle = getArmAngle(); // Get current arm position in degrees
        // double optimizedAngle = ((targetAngle - currentAngle + 180) % 360) - 180;
        // double newSetpoint = (currentAngle + optimizedAngle) * ArmConstant.ArmGearRatio;
        m_armKraken.setControl(motionMagicControl.withPosition(targetRotations));
    }

    public void setArmAngleFromDegrees(double targetDegrees) {
        setArmAngle(targetDegrees / 360.0);
    }

    @Override
    public void periodic() {
    //    SmartDashboard.putNumber("Arm Angle (Rotations)", getArmAngle());
        SmartDashboard.putNumber("Arm Motor Output", m_armKraken.getMotorVoltage().getValueAsDouble());
        
    }

}
