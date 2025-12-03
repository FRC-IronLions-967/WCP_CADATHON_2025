// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  public enum WantedState {
    IDLE,
    SCORINGFOOTHILLS,
    SCORINGUPTOWN,
    SCORINGDOWNTOWN,
    INTAKINGPANEL,
    INTAKINGBALLS
  }

  private enum CurrentState {
    IDLE,
    SCORINGBOTHFOOTHILLS,
    SCORINGBALLSFOOTHILLS,
    SCORINGPANELFOOTHILLS,
    SCORINGBOTHUPTOWN,
    SCORINGPANELUPTOWN,
    SCORINGBALLSUPTOWN,
    SCORINGBOTHDOWNTOWN,
    SCORINGPANELDOWNTOWN,
    SCORINGBALLSDOWNTOWN,
    INTAKINGPANEL,
    INTAKINGBALLS
  }

  private CurrentState currentState;
  private WantedState wantedState;

  /** Creates a new Superstructure. */
  public Superstructure() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // private CurrentState updateState() {
  //   return switch(wantedState) {
  //     case IDLE:
  //       yield CurrentState.IDLE;
  //     case SCORINGFOOTHILLS:

  //   }
  // }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public Command setStateCommand(WantedState wantedState) {
    return new InstantCommand(
        () -> {
          setWantedState(wantedState);
        });
  }
}
