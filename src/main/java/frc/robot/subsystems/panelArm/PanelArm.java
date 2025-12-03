// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.panelArm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PanelArm extends SubsystemBase {
  private final PanelArmIO io;
  private final PanelArmIOInputsAutoLogged inputs = new PanelArmIOInputsAutoLogged();

  private double armSetPosition;
  private double manipulatorSetSpeed;

  public PanelArm(PanelArmIO io) {
    this.io = io;
  }

  private enum SystemState {
    IDLE,
    INTAKING,
    SCORING,
    EJECTING,
    RESTING
  }

  public enum WantedState {
    IDLE,
    INTAKING,
    SCORING,
    RESTING
  }

  private SystemState systemState = SystemState.IDLE;
  private SystemState previousState = SystemState.IDLE;
  private WantedState wantedState = WantedState.IDLE;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PanelArm", inputs);

    systemState = updateState();
    applyState();

    io.runManipulator(manipulatorSetSpeed);
    io.moveArm(armSetPosition);

    Logger.recordOutput("PanelArmStates/ManipulatorSetSpeed", manipulatorSetSpeed);
    Logger.recordOutput("PanelArmStates/ArmSetPosition", armSetPosition);

    Logger.recordOutput("PanelArmStates/SystemState", systemState);
    Logger.recordOutput("PanelArmStates/PreviousState", previousState);
    Logger.recordOutput("PanelArmStates/WantedState", wantedState);
  }

  public boolean hasPanelInArm() {
    return inputs.hasPanelInArm;
  }
}
