package frc.robot.subsystems.endeffectorElevator;

public class EndEffectorElevatorStates {

    public EndEffectorElevatorState Home;
    public EndEffectorElevatorState Intake;
    public EndEffectorElevatorState StageL1;
    public EndEffectorElevatorState StageL2;
    public EndEffectorElevatorState StageL3;
    public EndEffectorElevatorState StageL4;
    public EndEffectorElevatorState PassOff;
    public EndEffectorElevatorState StowCoral;
    public EndEffectorElevatorState StowAlgae;
    public EndEffectorElevatorState PreScoreCoral;

    public EndEffectorElevatorStates(){
        Home = new EndEffectorElevatorState(0, 0);
        Intake = new EndEffectorElevatorState(0, 0);
        StageL1 = new EndEffectorElevatorState(0, 0);
        StageL2 = new EndEffectorElevatorState(0, 0);
        StageL3 = new EndEffectorElevatorState(0, 0);
        StageL4 = new EndEffectorElevatorState(0, 0);
        PassOff = new EndEffectorElevatorState(0, 0);
        StowCoral = new EndEffectorElevatorState(0, 0);
        StowAlgae = new EndEffectorElevatorState(0, 0);
        PreScoreCoral = new EndEffectorElevatorState(0, 0);
    }

}
