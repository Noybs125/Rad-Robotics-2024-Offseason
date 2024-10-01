package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autonomous extends SubsystemBase {

    public enum AutoCommands {
        FindGamePiece,
        GetGamePiece,
        PathfindSpeaker,
        PathfindAmp,
        PathfindSource,
        PathfindStage,
        EndAuto,
    }


    public Autonomous(){
        
    }

    public void periodic(){

    }

}
