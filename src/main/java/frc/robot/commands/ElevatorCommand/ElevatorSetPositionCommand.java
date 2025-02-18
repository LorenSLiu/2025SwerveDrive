package frc.robot.commands.ElevatorCommand;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorSetPositionCommand extends Command {
    private final ElevatorSubsystem elevator;
    private Distance targetPosition;
    

    public ElevatorSetPositionCommand(ElevatorSubsystem elevator, Distance targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
        //System.out.println("ElevatorSetPositionCommand initialized");
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Elevator Error", Math.abs(elevator.getCurrentPosition_Meters() - targetPosition.in(Meters)));
        elevator.setElevatorPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // System.out.println("Commandline output, Elevator Error: " + Math.abs(elevator.getCurrentPosition_Meters() - targetPosition.in(Meters)));

        // if(Math.abs(elevator.getCurrentPosition_Meters() - targetPosition.in(Meters)) < 0.02){
        //     System.out.println("ElevatorSetPositionCommand isFinished");
        // }
        //return Math.abs(elevator.getCurrentPosition_Meters() - targetPosition.in(Meters)) < 0.02;
        return false;
    }
    
}
