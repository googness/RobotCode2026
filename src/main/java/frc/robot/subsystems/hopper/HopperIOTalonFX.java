package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;

public class HopperIOTalonFX implements HopperIO {

  private TalonFX beltMotor;
  private TalonFX intakeMotor;
  private TalonFX hopperMotor;
  private TalonFX agitatorMotor;
  private final TalonFX feederMotor;

  private final MotionMagicVoltage hopperMotionMagicControlRequest;

  private final VelocityVoltage intakeVelocityRequest;
  private final NeutralOut intakeNeutralOut = new NeutralOut();

  private final VelocityVoltage beltVelocityRequest;
  private final NeutralOut beltNeutralOut = new NeutralOut();

  private final VelocityVoltage feederVelocityRequest;
  private final NeutralOut feederNeutralOut = new NeutralOut();

  private final VelocityVoltage agitatorVelocityRequest;
  private final NeutralOut agitatorNeutralOut = new NeutralOut();

  private StatusSignal<Current> beltSupplyCurrentSignal;
  private StatusSignal<Current> beltStatorCurrentSignal;
  private StatusSignal<Temperature> beltTempSignal;
  private StatusSignal<AngularVelocity> beltVelocitySignal;

  private StatusSignal<Current> intakeSupplyCurrentSignal;
  private StatusSignal<Current> intakeStatorCurrentSignal;
  private StatusSignal<Temperature> intakeTempSignal;
  private StatusSignal<AngularVelocity> intakeVelocitySignal;

  private StatusSignal<Current> feederSupplyCurrentSignal;
  private StatusSignal<Current> feederStatorCurrentSignal;
  private StatusSignal<Temperature> feederTempSignal;
  private StatusSignal<AngularVelocity> feederVelocitySignal;

  private StatusSignal<Current> agitatorSupplyCurrentSignal;
  private StatusSignal<Current> agitatorStatorCurrentSignal;
  private StatusSignal<Temperature> agitatorTempSignal;
  private StatusSignal<AngularVelocity> agitatorVelocitySignal;

  private StatusSignal<Current> hopperSupplyCurrentSignal;
  private StatusSignal<Current> hopperStatorCurrentSignal;
  private StatusSignal<Temperature> hopperTempSignal;
  private StatusSignal<Angle> hopperPositionSignal;

  private Alert beltConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert intakeConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert hopperConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert feederConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert agitatorConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private final Debouncer beltConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer intakeConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer feederConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer agitatorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer hopperConnectedDebouncer = new Debouncer(0.5);

  private double mHopperPos = 0;

  // Constructor
  public HopperIOTalonFX() {
    beltMotor = new TalonFX(hopperConfig.beltMotorId(), hopperConfig.CanBusName());
    intakeMotor = new TalonFX(hopperConfig.intakeMotorId(), hopperConfig.CanBusName());
    hopperMotor = new TalonFX(hopperConfig.hopperMotorId(), hopperConfig.CanBusName());
    agitatorMotor = new TalonFX(hopperConfig.agitatorMotorId(), hopperConfig.CanBusName());
    feederMotor = new TalonFX(hopperConfig.feederMotorId(), hopperConfig.CanBusName());

    hopperMotionMagicControlRequest = new MotionMagicVoltage(0);
    intakeVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    beltVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    feederVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    agitatorVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    hopperSupplyCurrentSignal = beltMotor.getSupplyCurrent();
    hopperStatorCurrentSignal = beltMotor.getStatorCurrent();
    hopperTempSignal = beltMotor.getDeviceTemp();
    hopperPositionSignal = beltMotor.getPosition();

    beltSupplyCurrentSignal = beltMotor.getSupplyCurrent();
    beltStatorCurrentSignal = beltMotor.getStatorCurrent();
    beltTempSignal = beltMotor.getDeviceTemp();
    beltVelocitySignal = beltMotor.getVelocity();

    intakeSupplyCurrentSignal = intakeMotor.getSupplyCurrent();
    intakeStatorCurrentSignal = intakeMotor.getStatorCurrent();
    intakeTempSignal = intakeMotor.getDeviceTemp();
    intakeVelocitySignal = intakeMotor.getVelocity();

    feederSupplyCurrentSignal = feederMotor.getSupplyCurrent();
    feederStatorCurrentSignal = feederMotor.getStatorCurrent();
    feederTempSignal = feederMotor.getDeviceTemp();
    feederVelocitySignal = feederMotor.getVelocity();

    agitatorSupplyCurrentSignal = agitatorMotor.getSupplyCurrent();
    agitatorStatorCurrentSignal = agitatorMotor.getStatorCurrent();
    agitatorTempSignal = agitatorMotor.getDeviceTemp();
    agitatorVelocitySignal = agitatorMotor.getVelocity();

    Phoenix6Util.registerSignals(
        true,
        beltSupplyCurrentSignal,
        beltStatorCurrentSignal,
        beltTempSignal,
        beltVelocitySignal,
        intakeSupplyCurrentSignal,
        intakeStatorCurrentSignal,
        intakeTempSignal,
        intakeVelocitySignal,
        feederSupplyCurrentSignal,
        feederStatorCurrentSignal,
        feederTempSignal,
        feederVelocitySignal,
        agitatorSupplyCurrentSignal,
        agitatorStatorCurrentSignal,
        agitatorTempSignal,
        agitatorVelocitySignal,
        hopperSupplyCurrentSignal,
        hopperStatorCurrentSignal,
        hopperTempSignal,
        hopperPositionSignal);

    configureHopperMotor(hopperMotor);
    configureBeltMotor(beltMotor);
    configureIntakeMotor(intakeMotor);
    configureAgitatorMotor(agitatorMotor);
    configureFeederMotor(feederMotor);
  }

  @Override
  public void setHopperPosition(double position) {
    if (position > HopperConstants.HOPPER_MAX_POSITION) {
      position = HopperConstants.HOPPER_MAX_POSITION;
    } else if (position < HopperConstants.HOPPER_MIN_POSITION) {
      position = HopperConstants.HOPPER_MIN_POSITION;
    }

    hopperMotor.setControl(hopperMotionMagicControlRequest.withPosition(position));
  }

  @Override
  public void setAgitator(double speed) {
    agitatorMotor.set(speed);
  }

  @Override
  public void setHopperSpeed(double speed) {
    hopperMotor.set(speed);
  }

  @Override
  public void stopHopper() {
    setHopperPosition(mHopperPos);
  }

  @Override
  public void runIntake() {
    intakeMotor.setControl(intakeVelocityRequest.withVelocity(HopperConstants.kdefaultIntakeSpeed));
  }

  @Override
  public void runClearIntake() {
    reverseIntake();
  }

  @Override
  public void reverseIntake() {
    intakeMotor.setControl(intakeVelocityRequest.withVelocity(HopperConstants.kreverseIntakeSpeed));
  }

  @Override
  public void stopIntake() {
    intakeMotor.setControl(intakeNeutralOut);
  }

  @Override
  public void runBelt() {
    beltMotor.setControl(beltVelocityRequest.withVelocity(HopperConstants.kdefaultBeltSpeed));
  }

  @Override
  public void runClearBelt() {
    beltMotor.setControl(beltVelocityRequest.withVelocity(-HopperConstants.kdefaultBeltSpeed));
  }

  @Override
  public void stopBelt() {
    beltMotor.setControl(beltNeutralOut);
  }

  @Override
  public void runAccelerator() {
    feederMotor.setControl(feederVelocityRequest.withVelocity(HopperConstants.kFeedRpm));
  }

  @Override
  public void stopAccelerator() {
    feederMotor.setControl(feederNeutralOut);
  }

  @Override
  public void runAgitator() {
    agitatorMotor.setControl(
        agitatorVelocityRequest.withVelocity(HopperConstants.kdefaultAgitatorSpeed));
  }

  @Override
  public void reverseAgitator() {
    agitatorMotor.setControl(
        agitatorVelocityRequest.withVelocity(-HopperConstants.kdefaultAgitatorSpeed));
  }

  @Override
  public void stopAgitator() {
    agitatorMotor.setControl(agitatorNeutralOut);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.intakeMotorConnected =
        intakeConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                intakeVelocitySignal,
                intakeSupplyCurrentSignal,
                intakeStatorCurrentSignal,
                intakeTempSignal));

    inputs.feederMotorConnected =
        feederConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                feederVelocitySignal,
                feederSupplyCurrentSignal,
                feederStatorCurrentSignal,
                feederTempSignal));

    inputs.agitatorMotorConnected =
        agitatorConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                agitatorVelocitySignal,
                agitatorSupplyCurrentSignal,
                agitatorStatorCurrentSignal,
                agitatorTempSignal));

    inputs.beltMotorConnected =
        beltConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                beltVelocitySignal,
                beltSupplyCurrentSignal,
                beltStatorCurrentSignal,
                beltTempSignal));

    inputs.hopperMotorConnected =
        hopperConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                hopperPositionSignal,
                hopperSupplyCurrentSignal,
                hopperStatorCurrentSignal,
                hopperTempSignal));

    inputs.beltMotorStatorCurrent = beltStatorCurrentSignal.getValueAsDouble();
    inputs.beltMotorSupplyCurrent = beltSupplyCurrentSignal.getValueAsDouble();
    inputs.beltMotorVelocityRps = beltVelocitySignal.getValueAsDouble();
    inputs.beltMotorTempCelsius = beltTempSignal.getValueAsDouble();

    inputs.feederMotorStatorCurrent = feederStatorCurrentSignal.getValueAsDouble();
    inputs.feederMotorSupplyCurrent = feederSupplyCurrentSignal.getValueAsDouble();
    inputs.feederMotorVelocityRps = feederVelocitySignal.getValueAsDouble();
    inputs.feederMotorTempCelsius = feederTempSignal.getValueAsDouble();

    inputs.agitatorMotorStatorCurrent = agitatorStatorCurrentSignal.getValueAsDouble();
    inputs.agitatorMotorSupplyCurrent = agitatorSupplyCurrentSignal.getValueAsDouble();
    inputs.agitatorMotorVelocityRps = agitatorVelocitySignal.getValueAsDouble();
    inputs.agitatorMotorTempCelsius = agitatorTempSignal.getValueAsDouble();

    inputs.intakeMotorStatorCurrent = intakeStatorCurrentSignal.getValueAsDouble();
    inputs.intakeMotorSupplyCurrent = intakeSupplyCurrentSignal.getValueAsDouble();
    inputs.intakeMotorVelocityRps = intakeVelocitySignal.getValueAsDouble();
    inputs.intakeMotorTempCelsius = intakeTempSignal.getValueAsDouble();

    inputs.hopperMotorStatorCurrent = hopperStatorCurrentSignal.getValueAsDouble();
    inputs.hopperMotorSupplyCurrent = hopperSupplyCurrentSignal.getValueAsDouble();
    inputs.hopperMotorPosition = hopperPositionSignal.getValueAsDouble();
    inputs.hopperMotorTempCelsius = hopperTempSignal.getValueAsDouble();
  }

  public void configureFeederMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = 0.11; // .11
    config.Slot0.kV = 0.132; // .117;
    config.Slot0.kD = 0.0;

    Phoenix6Util.applyAndCheckConfiguration(feederMotor, config, feederConfigAlert);
    FaultReporter.getInstance()
        .registerHardware(HopperConstants.SUBSYSTEM_NAME, "Hopper Feeder Motor", motor);
  }

  public void configureIntakeMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.2;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;

    config.Slot0.kV = 0.128; // 0.128;
    config.Slot0.kP = 0.13; // 0.15;
    config.Slot0.kD = 0.0;

    Phoenix6Util.applyAndCheckConfiguration(intakeMotor, config, intakeConfigAlert);
    FaultReporter.getInstance()
        .registerHardware(HopperConstants.SUBSYSTEM_NAME, "Hopper Intake Motor", motor);
  }

  public void configureBeltMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kV = 0.128;
    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0.0;

    Phoenix6Util.applyAndCheckConfiguration(beltMotor, config, beltConfigAlert);
    FaultReporter.getInstance()
        .registerHardware(HopperConstants.SUBSYSTEM_NAME, "Hopper Belt Motor", motor);
  }

  public void configureAgitatorMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kV = 0.128;
    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0.0;

    Phoenix6Util.applyAndCheckConfiguration(agitatorMotor, config, agitatorConfigAlert);
    FaultReporter.getInstance()
        .registerHardware(HopperConstants.SUBSYSTEM_NAME, "Hopper Agitator Motor", motor);
  }

  public void configureHopperMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 100;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;

    config.Slot0.kP = HopperConstants.HOPPER_KP; // kHopper_kP.get();
    config.Slot0.kI = HopperConstants.HOPPER_KI; // kHopper_kI.get();
    config.Slot0.kD = HopperConstants.HOPPER_KD; // kHopper_kD.get();
    config.Slot0.kS = HopperConstants.HOPPER_KS; // kHopper_kS.get();
    config.Slot0.kV = HopperConstants.HOPPER_KV; // kHopper_kV.get();
    config.Slot0.kA = HopperConstants.HOPPER_KA; // kHopper_kA.get();

    // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kHopperMaxPosition.get();
    // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kHopperMinPosition.get();

    config.MotionMagic.MotionMagicCruiseVelocity =
        HopperConstants
            .HOPPER_MOTOR_MM_CRUISE_VELOCITY; // kHopperMotorMMCruiseVelocity.get(); // 80 rps
    // cruise velocity
    config.MotionMagic.MotionMagicAcceleration =
        HopperConstants
            .HOPPER_MOTOR_MM_ACCELERATION; // kHopperMotorMMAcceleration.get(); // 160 rps/s
    // acceleration (0.5 seconds)
    config.MotionMagic.MotionMagicJerk =
        HopperConstants
            .HOPPER_MOTOR_MM_JERK; // kHopperMotorMMJerk.get(); // 1600 rps/s^2 jerk (0.1 seconds)

    hopperMotor.setPosition(0);

    motor.getConfigurator().apply(config);

    Phoenix6Util.applyAndCheckConfiguration(hopperMotor, config, hopperConfigAlert);
    FaultReporter.getInstance()
        .registerHardware(HopperConstants.SUBSYSTEM_NAME, "Hopper Motor", motor);
  }
}
