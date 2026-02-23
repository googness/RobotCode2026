package frc.robot.subsystems.hopper;

import frc.lib.team3061.RobotConfig;

public class HopperConstants {
  public static final String SUBSYSTEM_NAME = "Hopper";

  public static final double kdefaultIntakeSpeed = 0.5;
  public static final double kreverseIntakeSpeed = -0.5;

  public static final HopperConfig hopperConfig =
      new HopperConfig(RobotConfig.getInstance().getCANBusName(), 25, false, 26, false);

  public static record HopperConfig(
      String CanBusName,
      int beltMotorId,
      boolean beltMotorInverted,
      int intakeMotorId,
      boolean intakeMotorInverted) {}
}
