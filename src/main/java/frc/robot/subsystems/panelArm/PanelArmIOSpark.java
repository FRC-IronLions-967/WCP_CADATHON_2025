// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.panelArm;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.LimitSwitchManager;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class PanelArmIOSpark implements PanelArmIO {
  public BooleanSupplier panelArmLimitSwitch;

  protected SparkMax panelManipulator;
  private SparkMaxConfig panelManipulatorConfig;
  private SparkClosedLoopController panelManipulatorController;

  protected SparkMax arm;
  private SparkMaxConfig armConfig;
  private SparkClosedLoopController armController;

  public PanelArmIOSpark() {
    panelManipulator = new SparkMax(10, MotorType.kBrushless);
    panelManipulatorConfig = new SparkMaxConfig();
    panelArmLimitSwitch = LimitSwitchManager.getSwitch(10);

    panelManipulatorController = panelManipulator.getClosedLoopController();

    panelManipulatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    panelManipulatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 0, 0);
    panelManipulator.configure(
        panelManipulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    arm = new SparkMax(11, MotorType.kBrushless);
    armConfig = new SparkMaxConfig();

    armController = arm.getClosedLoopController();

    armConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    armConfig
        .absoluteEncoder
        .apply(new AbsoluteEncoderConfig())
        .positionConversionFactor(2 * Math.PI)
        .zeroOffset(PanelArmConstants.armZeroOffset);
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(PanelArmConstants.armP, PanelArmConstants.armI, PanelArmConstants.armD)
        .outputRange(-PanelArmConstants.armPercentPower, PanelArmConstants.armPercentPower);

    panelManipulatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 0, 0);
    arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(PanelArmIOInputs inputs) {
    inputs.hasPanelInArm = panelArmLimitSwitch.getAsBoolean();
    inputs.manipulatorSpeed = panelManipulator.getEncoder().getVelocity();
    inputs.armAngle = arm.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void runManipulator(double speed) {
    panelManipulatorController.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void moveArm(double angle) {
    armController.setReference(angle, ControlType.kPosition);
  }
}
