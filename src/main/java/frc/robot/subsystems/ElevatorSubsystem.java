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



@SuppressWarnings("all") // hater just be hating
public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX m_elevatorKraken;
    private TalonFX m_elevatorKrakenFollower;
    private PositionDutyCycle m_pidPosition = new PositionDutyCycle(0);

    private double setpoint = 0;  // Stores the last commanded position
    private final MotionMagicVoltage motionMagicControl;

    // NetworkTables publishers
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable elevatorTable = inst.getTable("Elevator");
    private final DoublePublisher elevatorHeightPublisher = elevatorTable.getDoubleTopic("Height").publish();
    private final DoublePublisher elevatorSpeedPublisher = elevatorTable.getDoubleTopic("Speed").publish();


    public ElevatorSubsystem() {
        m_elevatorKraken = new TalonFX(ElevatorConstants.kElevatorMotorID, ElevatorConstants.kElevatorCANbus);
        m_elevatorKrakenFollower = new TalonFX(ElevatorConstants.kElevatorMotorFollowerID, ElevatorConstants.kElevatorCANbus);


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
        // var motionMagicConfigs = talonFXConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 200; // Target cruise velocity of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 300; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicJerk = 100; // Target jerk of 1600 rps/s/s (0.1 seconds)
        m_elevatorKraken.getConfigurator().apply(talonFXConfigs);

        //current limit
        var cuurentLimitConfigs = new CurrentLimitsConfigs();
        cuurentLimitConfigs.StatorCurrentLimit = frc.robot.Constants.ElevatorConstants.kElevatorCurrentLimit;
        cuurentLimitConfigs.StatorCurrentLimitEnable = true;
        m_elevatorKraken.getConfigurator().apply(cuurentLimitConfigs);


        motionMagicControl  = new MotionMagicVoltage(0);

        m_elevatorKraken.setNeutralMode(NeutralModeValue.Brake);
        m_elevatorKrakenFollower.setControl(new Follower(ElevatorConstants.kElevatorMotorID, false));
        
        resetEncoder();

        


        // set if motor is inverted
        // m_elevator.setInverted(false);

    }

    public void setElevatorPosition(Distance targetHeight, Distance PULLEY_DIAMETER) {
        // Prevent exceeding software limits
        double Rotations = (3*targetHeight.in(Meters))/(Math.PI*PULLEY_DIAMETER.in(Meters));
        System.out.println("Setting elevator position to " + Rotations + " rotations");
        setpoint = Math.max(ElevatorConstants.kMinHeight, Math.min(Rotations, ElevatorConstants.kMaxHeight));//todo: check the safe height
        System.out.println("Setting elevator position to final " + Rotations + " rotations");

        System.out.println("setted");
        m_elevatorKraken.setControl(m_pidPosition.withPosition(10));
        //m_elevatorKraken.setControl(motionMagicControl.withPosition(-10));
// //        m_elevatorKraken.setPosition(10);
// m_elevatorKraken.set(0.09);  
}

    public void resetEncoder() {
        m_elevatorKraken.setPosition(0);
    }

    public void stop() {
        m_elevatorKraken.set(0);
    }

    /*
     * Get the current elevator height in rotations
     */
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
        m_elevatorKrakenFollower.set(speed);
        }
    }

    public double getElevatorHeightMeters() {
        return m_elevatorKraken.getPosition().getValueAsDouble()
                * frc.robot.Constants.ElevatorConstants.METERS_PER_ROTATION;
    }

    public void updateTelemetry() {
        // Add telemetry data using NetworkTables
        elevatorHeightPublisher.set(getElevatorHeightMeters());
        elevatorSpeedPublisher.set(m_elevatorKraken.getMotorVoltage().getValueAsDouble());
        
        // Add more telemetry data as needed
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Postition", getCurrentPosition());
        updateTelemetry();
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
