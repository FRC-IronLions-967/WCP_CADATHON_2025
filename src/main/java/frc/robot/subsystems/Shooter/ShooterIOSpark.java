// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Add your docs here. */
public class ShooterIOSpark implements ShooterIO {

  private SparkFlex flyWheel;
  private SparkFlexConfig flyWheelConfig;
  private SparkFlex shooterArm;
  private SparkFlex feeder;

  public ShooterIOSpark() {}
}
