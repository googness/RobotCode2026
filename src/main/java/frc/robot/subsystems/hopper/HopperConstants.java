package frc.robot.subsystems.hopper;

import frc.lib.team3061.RobotConfig;

public class HopperConstants {
  public static final String SUBSYSTEM_NAME = "Hopper";

  public static final double kdefaultIntakeSpeed = 90;
  public static final double kreverseIntakeSpeed = -75;
  public static final double kdefaultBeltSpeed = 40;

  public static final HopperConfig hopperConfig =
      new HopperConfig(RobotConfig.getInstance().getCANBusName(), 21, false, 31, false);

  public static record HopperConfig(
      String CanBusName,
      int beltMotorId,
      boolean beltMotorInverted,
      int intakeMotorId,
      boolean intakeMotorInverted) {}
}
