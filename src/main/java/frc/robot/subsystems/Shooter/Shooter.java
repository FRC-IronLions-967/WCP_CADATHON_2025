// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double shooterArmSetPosition;
  private double feederSetSpeed;
  private double flyWheelSetSpeed;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  private enum SystemState {
    IDLE,
    FEEDING,
    SCORING,
    EJECTING,
    RESTING
  }

  public enum WantedState {
    IDLE,
    FEEDING,
    SCORING,
    RESTING
  }

  private SystemState systemState = SystemState.IDLE;
  private SystemState previousState = SystemState.IDLE;
  private WantedState wantedState = WantedState.IDLE;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    systemState = updateState();
    applyState();

    io.moveShooter(shooterArmSetPosition);
    io.runShooter(flyWheelSetSpeed);
    io.runFeeder(feederSetSpeed);

    Logger.recordOutput("ShooterStates/shooterArmSetPosition", shooterArmSetPosition);
    Logger.recordOutput("ShooterStates/flyWheelSetSpeed", flyWheelSetSpeed);
    Logger.recordOutput("ShooterStates/feederSetSpeed", feederSetSpeed);

    Logger.recordOutput("ShooterStates/SystemState", systemState);
    Logger.recordOutput("ShooterStates/PreviousState", previousState);
    Logger.recordOutput("ShooterStates/WantedState", wantedState);
  }

  public boolean hasObjectInFeeder() {
    return inputs.hasObjectInFeeder;
  }

  public boolean hasObjectInShooter() {
    return inputs.hasObjectInShooter;
  }

  public boolean isArmInPosition(double position) {
    return position + ShooterConstants.armTolerance < inputs.shooterAngle
        || inputs.shooterAngle < position + ShooterConstants.armTolerance;
  }

  private SystemState updateState() {
    previousState = systemState;
    return switch (wantedState) {
      case IDLE:
        yield systemState.IDLE;
      case FEEDING:
        if (hasObjectInShooter()) yield systemState.SCORING;
        yield systemState.FEEDING;
      case SCORING:
        if (!isArmInPosition(ShooterConstants.armScoringPosition)) {
          yield SystemState.SCORING;
        }
        yield SystemState.EJECTING;
      case RESTING:
        yield SystemState.RESTING;
    };
  }

  private void applyState() {
    switch (systemState) {
      case IDLE:
        break;
      case FEEDING:
        feederSetSpeed = ShooterConstants.feederFeedingSpeed;
        break;
      case SCORING:
        shooterArmSetPosition = ShooterConstants.armScoringPosition;
        feederSetSpeed = ShooterConstants.feederRestingSpeed;
        flyWheelSetSpeed = ShooterConstants.flyWheelRestingSpeed;
        break;
      case EJECTING:
        shooterArmSetPosition = ShooterConstants.armScoringPosition;
        feederSetSpeed = ShooterConstants.feederFeedingSpeed;
        flyWheelSetSpeed = ShooterConstants.flyWheelShootingSpeed;
        break;
      case RESTING:
        shooterArmSetPosition = ShooterConstants.armRestingPosition;
        flyWheelSetSpeed = ShooterConstants.flyWheelRestingSpeed;
        feederSetSpeed = ShooterConstants.feederRestingSpeed;
        break;
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
