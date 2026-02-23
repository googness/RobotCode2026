package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    boolean intakeMotorConnected = false;

    public double intakeMotorPositionRads = 0.0;
    public double inakeMotorVelocityRpm = 0.0;
    public double intakeMotorAppliedVolts = 0.0;
    public double intakeMotorSupplyCurrentAmps = 0.0;
    public double intakeMotorTorqueCurretAmps = 0.0;
    public double intakeMotorTempCelsius = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void stop() {}

  public default void runIntake() {}
}
