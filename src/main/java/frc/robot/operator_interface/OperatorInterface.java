// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  // drivetrain, generic

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseToVisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTranslationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLock180Button() {
    return new Trigger(() -> false);
  }

  public default Trigger getVisionIsEnabledTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicReverse() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticReverse() {
    return new Trigger(() -> false);
  }

  // DRIVER TRIGGERS, mostly game-specific

  public default Trigger getDriveToPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getOverrideDriveToPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getCurrentPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getInterruptAll() {
    return new Trigger(() -> false);
  }

  // OPERATOR TRIGGERS, mostly game-specific
  public default Trigger getEnablePrimaryIRSensorsTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getEnableAutoScoringTrigger() {
    return new Trigger(() -> false);
  }

  // XRP EXAMPLE TRIGGERS
  public default Trigger getMoveArmMiddlePositionTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveArmHighPositionTrigger() {
    return new Trigger(() -> false);
  }

  // ELEVATOR EXAMPLE TRIGGERS
  public default Trigger getRaiseElevatorSlowButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLowerElevatorSlowButton() {
    return new Trigger(() -> false);
  }

  // Jays Commands \\

  public default Trigger theShooterFix() {
    return new Trigger(() -> false);
  }

  public default Trigger getHubShot() {
    return new Trigger(() -> false);
  }

  public default Trigger extendHoodAngle() {
    return new Trigger(() -> false);
  }

  public default Trigger dShot() {
    return new Trigger(() -> false);
  }

  public default Trigger retractHoodAngle() {
    return new Trigger(() -> false);
  }

  public default Trigger extendHopper() {
    return new Trigger(() -> false);
  }

  public default Trigger retractHopper() {
    return new Trigger(() -> false);
  }

  public default Trigger runAccelerator() {
    return new Trigger(() -> false);
  }

  public default Trigger runIntake() {
    return new Trigger(() -> false);
  }

  public default Trigger runBelt() {
    return new Trigger(() -> false);
  }

  public default Trigger extendHood() {
    return new Trigger(() -> false);
  }

  public default Trigger retractHood() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotation() {
    return new Trigger(() -> false);
  }

  public default Trigger resetVisionPos() {
    return new Trigger(() -> false);
  }

  public default Trigger reverseIntake() {
    return new Trigger(() -> false);
  }

  // Jays Commands \\
}
