// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean hasBallInShooter = false;
    public double ShooterSpeed = 0.0;
    public double FeederSpeed = 0.0;
    public double ShooterAngle = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void moveShooterArm(double angle) {}

  public default void moveFeederWheel(double speed) {}

  public default void MoveShooterWheel(double speed) {}
}
