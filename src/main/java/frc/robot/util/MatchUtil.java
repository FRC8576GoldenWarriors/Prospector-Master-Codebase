package frc.robot.util;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MatchUtil extends SubsystemBase{

    @AutoLogOutput(key = "MatchUtil/Alliance")
    private Alliance alliance;

    @AutoLogOutput(key = "MatchUtil/GameMessage")
    private String gameMessage;

    private enum GameState {
        Autonomous,
        RedShift,
        BlueShift
    }

    @AutoLogOutput(key = "MatchUtil/AutonWinner")
    private volatile Alliance autonWinner = null;

    @AutoLogOutput(key = "MatchUtil/CurrentGameState")
    private GameState currentGameState;


    public MatchUtil() {
        gameMessage = DriverStation.getGameSpecificMessage();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            this.alliance = alliance.get();
        } else {
            this.alliance = Alliance.Blue;
        }
        updateGameMessage();
    }

    @Override
    public void periodic() {
        updateGameMessage();

       if(autonWinner == null && (currentGameState == GameState.RedShift || currentGameState == GameState.BlueShift)) {
        if(currentGameState == GameState.RedShift) {
            autonWinner = Alliance.Red;
        } else if(currentGameState == GameState.BlueShift) {
            autonWinner = Alliance.Blue;
        }
       }
    }

    public void updateGameMessage() {
        gameMessage = DriverStation.getGameSpecificMessage();
        if(gameMessage == null) {
            gameMessage = "";
        }

        if(gameMessage.equals("")) {
         currentGameState = GameState.Autonomous;
        } else if(gameMessage.equals("R")) {
         currentGameState = GameState.RedShift;
        } else if(gameMessage.equals("B")) {
         currentGameState = GameState.BlueShift;
        }
    }

    public GameState getCurrentShift() {
        return currentGameState;
    }

    public boolean isRedShift() {
        return currentGameState == GameState.RedShift;
    }

    public boolean isBlueShift() {
        return currentGameState == GameState.BlueShift;
    }

    public boolean isAutonomous() {
        return currentGameState == GameState.Autonomous;
    }

    public Optional<Alliance> getAutonWinner() {
        if(autonWinner == null) {
            return Optional.empty();
        }
        return Optional.of(autonWinner);
    }

    public Optional<Boolean> wonAuton() {
        Optional<Alliance> autonWinner = getAutonWinner();
        if (autonWinner.isPresent()) {
            return Optional.of(autonWinner.get() == getAlliance());
        }
        return Optional.empty();
    }

    public Alliance getAlliance() {
        return alliance;
    }
}
