// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.panelArm;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface PanelArmIO {

  @AutoLog
  public static class PanelArmIOInputs {
    public double armAngle = 0.0;
    public double manipulatorSpeed = 0.0;
    public boolean hasPanelInArm = false;
  }

  public default void updateInputs(PanelArmIOInputs inputs) {}

  public default void runManipulator(double speed) {}

  public default void moveArm(double angle) {}
}
