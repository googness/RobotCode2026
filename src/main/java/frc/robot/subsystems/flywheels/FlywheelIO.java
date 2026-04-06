package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    boolean frontFlywheelMotorConnected = false;
    public double frontFlywheelMotorVelocityRps = 0.0;
    public double frontFlywheelMotorStatorCurrent = 0.0;
    public double frontFlywheelMotorSupplyCurrent = 0.0;
    public double frontFlywheelMotorTempCelsius = 0.0;

    boolean backFlywheelMotorConnected = false;
    public double backFlywheelMotorVelocityRps = 0.0;
    public double backFlywheelMotorStatorCurrent = 0.0;
    public double backFlywheelMotorSupplyCurrent = 0.0;
    public double backFlywheelMotorTempCelsius = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void runVelocity(double rps) {}

  default void stopFlywheel() {}
}
