package frc.robot.subsystems.hopper;

import frc.lib.team3061.RobotConfig;

public class HopperConstants {
  public static final String SUBSYSTEM_NAME = "Hopper";

  public static final double kdefaultIntakeSpeed = 80;
  public static final double kreverseIntakeSpeed = -75;
  public static final double kdefaultBeltSpeed = 50;
  public static final double kdefaultAgitatorSpeed = 70;

  public static final double kFeedRpm = 60;

  public static final double HOPPER_START_POSITION = 0;
  public static final double HOPPER_HOME_POSITION = 2;
  public static final double HOPPER_INTAKE_POSITION = 22.3; // 21;
  public static final double HOPPER_MAX_POSITION = 22.7; // 21.5;

  public static final double HOPPER_PULSE_IN_POSITION = 5;
  public static final double HOPPER_PULSE_OUT_POSITION = 17;
  public static final double HOPPER_PULSE_IN_EXTENDED_POSITION = 14.5; // 15;
  public static final double HOPPER_PULSE_OUT_EXTENDED_POSITION = 21.5; // 21;

  public static final double HOPPER_MIN_POSITION = 0;
  public static final double HOPPER_POSITION_TOLERANCE = 1;

  public static final double HOPPER_MOTOR_MM_CRUISE_VELOCITY = 150; // 100; // 50; // 100; // 80;
  public static final double HOPPER_MOTOR_MM_ACCELERATION =
      350; // 300; // 200; // 100; // 200; // 160;
  public static final double HOPPER_MOTOR_MM_JERK = 800; // 600; // 800;

  public static final double HOPPER_KP = 4.5; // 4.8;
  public static final double HOPPER_KI = 0.0;
  public static final double HOPPER_KD = 0.015; // 0.01;
  public static final double HOPPER_KS = 0.3; // 0.24;
  public static final double HOPPER_KV = 0.14; // 0.12;
  public static final double HOPPER_KA = 0.0;

  public static final HopperConfig hopperConfig =
      new HopperConfig(RobotConfig.getInstance().getCANBusName(), 21, 31, 33, 32, 22, true);

  public static record HopperConfig(
      String CanBusName,
      int beltMotorId,
      int intakeMotorId,
      int hopperMotorId,
      int agitatorMotorId,
      int feederMotorId,
      boolean feederMotorInverted) {}
}
