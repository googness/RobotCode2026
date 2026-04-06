package frc.robot.subsystems.flywheels;

import frc.lib.team3061.RobotConfig;

public class FlywheelConstants {

  public static final String SUBSYSTEM_NAME = "Flywheel";

  public static final double kDefaultShootingRps = 30;

  public static final FlywheelConfig kFlywheelConfig =
      new FlywheelConfig(RobotConfig.getInstance().getCANBusName(), 23, 24);

  public record FlywheelConfig(String canBusName, int frontMotorCANId, int backMotorCANId) {}
}
