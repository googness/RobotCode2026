package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX frontShooter;
  private final TalonFX backShooter;

  private final VelocityVoltage frontVelocityRequest;
  private final NeutralOut frontNeutralOut = new NeutralOut();

  private final VelocityVoltage backVelocityRequest;
  private final NeutralOut backNeutralOut = new NeutralOut();

  private StatusSignal<Current> frontSupplyCurrentSignal;
  private StatusSignal<Current> frontStatorCurrentSignal;
  private StatusSignal<Temperature> frontTempSignal;
  private StatusSignal<AngularVelocity> frontVelocitySignal;

  private StatusSignal<Current> backSupplyCurrentSignal;
  private StatusSignal<Current> backStatorCurrentSignal;
  private StatusSignal<Temperature> backTempSignal;
  private StatusSignal<AngularVelocity> backVelocitySignal;

  private final Debouncer frontConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer backConnectedDebouncer = new Debouncer(0.5);

  private Alert frontConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert backConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  public FlywheelIOTalonFX() {

    frontShooter = new TalonFX(kFlywheelConfig.frontMotorCANId(), kFlywheelConfig.canBusName());
    backShooter = new TalonFX(kFlywheelConfig.backMotorCANId(), kFlywheelConfig.canBusName());

    frontVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    backVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    frontSupplyCurrentSignal = frontShooter.getSupplyCurrent();
    frontStatorCurrentSignal = frontShooter.getStatorCurrent();
    frontTempSignal = frontShooter.getDeviceTemp();
    frontVelocitySignal = frontShooter.getVelocity();

    backSupplyCurrentSignal = backShooter.getSupplyCurrent();
    backStatorCurrentSignal = backShooter.getStatorCurrent();
    backTempSignal = backShooter.getDeviceTemp();
    backVelocitySignal = backShooter.getVelocity();

    Phoenix6Util.registerSignals(
        true,
        frontSupplyCurrentSignal,
        frontStatorCurrentSignal,
        frontTempSignal,
        frontVelocitySignal,
        backSupplyCurrentSignal,
        backStatorCurrentSignal,
        backTempSignal,
        backVelocitySignal);

    configureBackMotor(backShooter);
    configureFrontMotor(frontShooter);
  }

  @Override
  public void runVelocity(double rps) {
    frontShooter.setControl(frontVelocityRequest.withVelocity(rps));
    backShooter.setControl(backVelocityRequest.withVelocity(rps));
  }

  @Override
  public void stopFlywheel() {
    frontShooter.setControl(frontNeutralOut);
    backShooter.setControl(backNeutralOut);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.frontFlywheelMotorConnected =
        frontConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                frontVelocitySignal,
                frontSupplyCurrentSignal,
                frontStatorCurrentSignal,
                frontTempSignal));

    inputs.backFlywheelMotorConnected =
        backConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                backVelocitySignal,
                backSupplyCurrentSignal,
                backStatorCurrentSignal,
                backTempSignal));

    inputs.frontFlywheelMotorStatorCurrent = frontStatorCurrentSignal.getValueAsDouble();
    inputs.frontFlywheelMotorSupplyCurrent = frontSupplyCurrentSignal.getValueAsDouble();
    inputs.frontFlywheelMotorVelocityRps = frontVelocitySignal.getValueAsDouble();
    inputs.frontFlywheelMotorTempCelsius = frontTempSignal.getValueAsDouble();

    inputs.backFlywheelMotorStatorCurrent = backStatorCurrentSignal.getValueAsDouble();
    inputs.backFlywheelMotorSupplyCurrent = backSupplyCurrentSignal.getValueAsDouble();
    inputs.backFlywheelMotorVelocityRps = backVelocitySignal.getValueAsDouble();
    inputs.backFlywheelMotorTempCelsius = backTempSignal.getValueAsDouble();
  }

  public void configureFrontMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.2;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = 0.13;
    config.Slot0.kV = 0.130;
    config.Slot0.kS = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kD = 0.0;

    Phoenix6Util.applyAndCheckConfiguration(frontShooter, config, frontConfigAlert);
    FaultReporter.getInstance()
        .registerHardware(FlywheelConstants.SUBSYSTEM_NAME, "Flywheel Motor Front", motor);
  }

  public void configureBackMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.2;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = 0.13;
    config.Slot0.kV = 0.130;
    config.Slot0.kS = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kD = 0.0;

    Phoenix6Util.applyAndCheckConfiguration(backShooter, config, backConfigAlert);
    FaultReporter.getInstance()
        .registerHardware(FlywheelConstants.SUBSYSTEM_NAME, "Flywheel Motor Back", motor);
  }
}
