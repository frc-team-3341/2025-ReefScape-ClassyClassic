package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorCMStateMachine extends SubsystemBase{
    private State currentRobotState;

    private Elevator elevator;
    private CoralManipulator cm;

    public ElevatorCMStateMachine(Elevator elevator, CoralManipulator cm) {
        currentRobotState = State.IDLE;

        this.elevator = elevator;
        this.cm = cm;
    }

    public State getRobotState() {
        return currentRobotState;
    }

    public void setRobotState(State newState) {
        currentRobotState = newState;
    }

    //HUGE inspiration from FRC Team 3255 SuperNURDS
    public Command switchState(State desiredState) {
        return null;
    }
    
}

//TODO finish creating states based on brainstorm
enum State {
    //system is at rest or holding height
    IDLE,

    //different heights, prep coral manipulator and elevator
    GO_TO_INTAKE,
    GO_TO_L1,
    GO_TO_L2,
    GO_TO_L3,
    GO_TO_L4
    
}
