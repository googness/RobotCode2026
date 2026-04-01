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
    public double hopperPositon = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  default void setHopperPosition(double position) {}

  default void setHopperSpeed(double speed) {}

  default void stopHopper() {}

  default void manualHopperControl() {}

  public default void stop() {}

  public default void runBelt() {}

  public default void runClearBelt() {}

  public default void runIntake() {}

  public default void reverseIntake() {}

  public default void runClearIntake() {}

  public default void stopIntake() {}

  public default void stopBelt() {}

  public default void setAgitator(double speed) {}
}
