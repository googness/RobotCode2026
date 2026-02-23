package frc.robot.subsystems.flywheels;

import frc.lib.team3061.RobotConfig;

public class FlywheelConstants {

  public static final double kMinCoastVelocityRpm = 1000;
  public static final double kRpmInRangeOffset = 250;
  public static final double kShootRpm = 4000;
  public static final double kShuttleRpm = 1500;

  public static final double FLYWHEEL_KP = 0;
  public static final double FLYWHEEL_KI = 0;
  public static final double FLYWHEEL_KD = 0;
  public static final double FLYWHEEL_KS = 0;
  public static final double FLYWHEEL_KV = 0;
  public static final double FLYWHEEL_KA = 0;

  public static final FlywheelConfig kFlywheelConfig =
      new FlywheelConfig(RobotConfig.getInstance().getCANBusName(), 21, true, 22, true);

  public record FlywheelConfig(
      String canBusName,
      int frontMotorCANId,
      boolean frontMotorClockwisePositive,
      int backMotorCANId,
      Boolean backMotorClockwisePositive) {}
}
