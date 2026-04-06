package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    boolean intakeMotorConnected = false;
    public double intakeMotorVelocityRps = 0.0;
    public double intakeMotorStatorCurrent = 0.0;
    public double intakeMotorSupplyCurrent = 0.0;
    public double intakeMotorTempCelsius = 0.0;

    boolean beltMotorConnected = false;
    public double beltMotorVelocityRps = 0.0;
    public double beltMotorStatorCurrent = 0.0;
    public double beltMotorSupplyCurrent = 0.0;
    public double beltMotorTempCelsius = 0.0;

    boolean agitatorMotorConnected = false;
    public double agitatorMotorVelocityRps = 0.0;
    public double agitatorMotorStatorCurrent = 0.0;
    public double agitatorMotorSupplyCurrent = 0.0;
    public double agitatorMotorTempCelsius = 0.0;

    boolean feederMotorConnected = false;
    public double feederMotorVelocityRps = 0.0;
    public double feederMotorStatorCurrent = 0.0;
    public double feederMotorSupplyCurrent = 0.0;
    public double feederMotorTempCelsius = 0.0;

    boolean hopperMotorConnected = false;
    public double hopperMotorPosition = 0.0;
    public double hopperMotorStatorCurrent = 0.0;
    public double hopperMotorSupplyCurrent = 0.0;
    public double hopperMotorTempCelsius = 0.0;
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

  default void runAccelerator() {}

  default void stopAccelerator() {}

  default void runAgitator() {}

  public default void reverseAgitator() {}

  default void stopAgitator() {}

  public default void setAgitator(double speed) {}
}
