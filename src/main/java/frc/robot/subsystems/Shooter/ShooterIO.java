// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterAngle;
    public double shooterSpeed;
    public double feederSpeed;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void moveShooter(double angle) {}

  public default void runShooter(double speed) {}
}
