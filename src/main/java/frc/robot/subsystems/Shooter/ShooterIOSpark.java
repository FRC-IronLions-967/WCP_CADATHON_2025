// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.util.LimitSwitchManager;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class ShooterIOSpark implements ShooterIO {
  public BooleanSupplier feederLimitSwitch;
  public BooleanSupplier shooterLimitSwitch;

  private SparkFlex flyWheel;
  private SparkFlex shooterArm;
  private SparkFlexConfig flyWheelConfig;
  private SparkFlexConfig shooterArmConfig;
  private SparkClosedLoopController flyWheelController;
  private SparkClosedLoopController feederController;
  private SparkClosedLoopController armController;
  private SparkFlex feeder;
  private SparkFlexConfig feederConfig;

  public ShooterIOSpark() {
    flyWheel = new SparkFlex(11, MotorType.kBrushless);
    flyWheelConfig = new SparkFlexConfig();
    shooterLimitSwitch = LimitSwitchManager.getSwitch(11);

    flyWheelController = flyWheel.getClosedLoopController();

    flyWheelConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
    flyWheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    flyWheel.configure(
        flyWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterArm = new SparkFlex(12, MotorType.kBrushless);
    shooterArmConfig = new SparkFlexConfig();

    armController = shooterArm.getClosedLoopController();

    shooterArmConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
    shooterArmConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    shooterArm.configure(
        shooterArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feeder = new SparkFlex(13, MotorType.kBrushless);
    feederConfig = new SparkFlexConfig();
    feederLimitSwitch = LimitSwitchManager.getSwitch(13);

    feederController = feeder.getClosedLoopController();

    feederConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
    feederConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.hasObjectInFeeder = feederLimitSwitch.getAsBoolean();
    inputs.hasObjectInShooter = shooterLimitSwitch.getAsBoolean();
    inputs.shooterAngle = shooterArm.getAbsoluteEncoder().getPosition();
    inputs.shooterSpeed = flyWheel.getEncoder().getVelocity();
    inputs.feederSpeed = feeder.getEncoder().getVelocity();
  }

  @Override
  public void moveShooter(double angle) {
    armController.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void runShooter(double speed) {
    flyWheelController.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void runFeeder(double speed) {
    feederController.setReference(speed, ControlType.kVelocity);
  }
}
