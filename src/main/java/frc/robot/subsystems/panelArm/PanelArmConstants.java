// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.panelArm;

/** Add your docs here. */
public class PanelArmConstants {

  public static final double armP = 1.0;
  public static final double armI = 0.0;
  public static final double armD = 0.0;

  public static final double intakeP = 0.1;
  public static final double intakeI = 0.0;
  public static final double intakeD = 0.0;

  public static final double armMinPosition = 3.9;
  public static final double armMaxPosition = 2 * Math.PI;

  public static final double armZeroOffset = 0.4;
  public static final double armPercentPower = 0.25;

  public static final double panelArmIntakeSpeed = 1;
  public static final double panelArmScoringSpeed = 1;
  public static final boolean panelArmIntakeSpeedPositive = panelArmIntakeSpeed > 0;

  public static final double armIntakePosition = 220 * Math.PI / 180;
  public static final double armIntakeToFeederPosition = 15 * Math.PI / 180;
  public static final double armRestingPosition = 0 * Math.PI / 180;
  public static final double armScoringPosition = 300 * Math.PI / 180;
  public static final double armTolerance = 0.5;
}
