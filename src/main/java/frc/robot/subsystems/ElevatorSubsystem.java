package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

@SuppressWarnings("all") // hater just be hating
public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX m_elevatorKraken;
    private double setpoint = 0;  // Stores the last commanded position
    private final MotionMagicVoltage motionMagicControl;

    // Shuffleboard Tab for Elevator
    private final ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
    private final GenericEntry elevatorPositionEntry = elevatorTab.add("Current Position", 0).getEntry();
    private final GenericEntry targetPositionEntry = elevatorTab.add("Target Position", 0).getEntry();
    private final GenericEntry motorOutputEntry = elevatorTab.add("Motor Output", 0).getEntry();

    private final ElevatorSim elevatorSim = new ElevatorSim(
        LinearSystemId.identifyPositionSystem(1.0, 0.4), // System gain (tune this)
        DCMotor.getFalcon500(1),  // Using a Kraken (Falcon equivalent)
        0,  
        1.84,// Gear Ratio
        true,
        0  // Simulated gravity
    );




    public ElevatorSubsystem() {
        m_elevatorKraken = new TalonFX(frc.robot.Constants.ElevatorConstants.kElevatorMotorID);

        var talonFXConfigs = new TalonFXConfiguration();


        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = frc.robot.Constants.ElevatorConstants.kElevatorS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = frc.robot.Constants.ElevatorConstants.kElevatorV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = frc.robot.Constants.ElevatorConstants.kElevatorA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = frc.robot.Constants.ElevatorConstants.kElevatorP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = frc.robot.Constants.ElevatorConstants.kElevatorI; // no output for integrated error
        slot0Configs.kD = frc.robot.Constants.ElevatorConstants.kElevatorD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        m_elevatorKraken.getConfigurator().apply(talonFXConfigs);
        motionMagicControl  = new MotionMagicVoltage(0);

        //current limit
        var cuurentLimitConfigs = new CurrentLimitsConfigs();
        cuurentLimitConfigs.StatorCurrentLimit = frc.robot.Constants.ElevatorConstants.kElevatorCurrentLimit;
        cuurentLimitConfigs.StatorCurrentLimitEnable = true;
        m_elevatorKraken.getConfigurator().apply(cuurentLimitConfigs);

        m_elevatorKraken.setNeutralMode(NeutralModeValue.Brake);
        resetEncoder();


        // set if motor is inverted
        // m_elevator.setInverted(false);

    }


    @Override
    public void periodic() {
// Send data to NetworkTables
        SmartDashboard.putNumber("Elevator Position", getCurrentPosition());
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putNumber("Elevator Motor Output", m_elevatorKraken.getMotorVoltage().getValueAsDouble());

        // Update Shuffleboard Entries
        elevatorPositionEntry.setDouble(getCurrentPosition());
        targetPositionEntry.setDouble(setpoint);
        motorOutputEntry.setDouble(m_elevatorKraken.getMotorVoltage().getValueAsDouble());
        
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            elevatorSim.setInput(m_elevatorKraken.getMotorVoltage().getValueAsDouble());
            elevatorSim.update(0.02);  // 20ms timestep
            m_elevatorKraken.setPosition(elevatorSim.getPositionMeters()); // Update encoder
        }
    }

    public void setElevatorPosition(double position) {
        // Prevent exceeding software limits
        setpoint = Math.max(ElevatorConstants.kMinHeight, Math.min(position, ElevatorConstants.kMaxHeight));//todo: check the safe height
        System.out.println("Setting elevator position to " + position);

        m_elevatorKraken.setControl(motionMagicControl.withPosition(setpoint));
    }

    public void resetEncoder() {
        m_elevatorKraken.setPosition(0);
    }

    public void stop() {
        m_elevatorKraken.set(0);
    }

    public double getCurrentPosition() {
        return m_elevatorKraken.getPosition().getValueAsDouble();
    }

    public void manualControl(double speed) {
        double newPosition = m_elevatorKraken.getPosition().getValueAsDouble()
                + (speed * ElevatorConstants.kManualSpeedMultiplier);

        // Enforce software limits
        if ((speed < 0 && newPosition <= ElevatorConstants.kMinHeight) ||
                (speed > 0 && newPosition >= ElevatorConstants.kMaxHeight)) {
            stop();
        } else {
            m_elevatorKraken.set(speed);
        }
    }



    //below are deprecated stuff
    @Deprecated
    public void setElevatorHeight(double targetMeters) {

        double TargetRotations = convertMetersToRotations(targetMeters);// convert meters to ticks, then convert ticks
                                                                        // to rotations
        System.out.println("Target Rotations: " + TargetRotations);
        m_elevatorKraken.setPosition(TargetRotations);
    }

    @Deprecated
    public double getElevatorHeightMeters() {
        return m_elevatorKraken.getPosition().getValueAsDouble()
                * frc.robot.Constants.ElevatorConstants.METERS_PER_ROTATION;
    }

    @Deprecated
    public double convertMetersToTicks(double meters) {
        // i mean it's just my personal preference to do with ticks, but you can do it
        // with rotations
        return meters / frc.robot.Constants.ElevatorConstants.METERS_PER_TICK;
    }

    @Deprecated
    public double convertTicksToRotations(double ticks) {
        return ticks / frc.robot.Constants.ElevatorConstants.ENCODER_TICKS_PER_REV;
    }

    @Deprecated
    public double convertMetersToRotations(double meters) {
        return meters / frc.robot.Constants.ElevatorConstants.METERS_PER_ROTATION;
    }

}
