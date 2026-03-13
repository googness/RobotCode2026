package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {

    // Front Flywheel Motor Inputs
    boolean frontFlywheelMotorConnected = false;
    public double frontFlywheelMotorPositionRads = 0.0;
    public double frontFlywheelMotorVelocityRpm = 0.0;
    public double frontFlywheelMotorStatorCurrent = 0.0;
    public double frontFlywheelMotorSupplyCurrent = 0.0;
    public double frontFlywheelMotorTorqueCurrentAmps = 0.0;
    public double frontFlywheelMotorTempCelsius = 0.0;

    // Back Flywheel Motor Inputs
    boolean backFlywheelMotorConnected = false;
    public double backFlywheelMotorPositionRads = 0.0;
    public double backFlywheelMotorVelocityRpm = 0.0;
    public double backFlywheelMotorStatorCurrent = 0.0;
    public double backFlywheelMotorSupplyCurrent = 0.0;
    public double backFlywheelMotorTorqueCurrentAmps = 0.0;
    public double backFlywheelMotorTempCelsius = 0.0;

    // Feeder Motor Inputs
    boolean feederMotorConnected = false;
    public double feederMotorPositionRads = 0.0;
    public double feederMotorVelocityRpm = 0.0;
    public double feederMotorStatorCurrent = 0.0;
    public double feederMotorSupplyCurrent = 0.0;
    public double feederMotorTorqueCurrentAmps = 0.0;
    public double feederMotorTempCelsius = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void runVelocity() {}

  default void runVelocity(double rps) {}

  default void runAccelerator() {}

  default void stopAccelerator() {}

  default void stopFlywheel() {}

  default void extendHood() {}

  default void retractHood() {}

  default void setGains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
