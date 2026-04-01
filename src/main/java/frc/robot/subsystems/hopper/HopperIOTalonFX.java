package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.team6328.util.LoggedTunableNumber;

public class HopperIOTalonFX implements HopperIO {

  private TalonFX beltMotor;
  private TalonFX intakeMotor;
  private TalonFX hopperMotor;
  private TalonFX agitatorMotor;
  private DutyCycleEncoder hopperEncoder;

  private final MotionMagicVoltage hopperMotionMagicControl = new MotionMagicVoltage(0);

  private double mHopperPos = 0;

  private final VelocityVoltage m_intakeVelocityRequest =
      new VelocityVoltage(0).withEnableFOC(true);
  private final NeutralOut m_intakeNeutralOut = new NeutralOut();

  private final VelocityVoltage m_beltVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final NeutralOut m_beltNeutralOut = new NeutralOut();

  private Alert beltConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert intakeConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert hopperConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private static final LoggedTunableNumber m_intakeRps =
      new LoggedTunableNumber("Hopper/IntakeRps", HopperConstants.kdefaultIntakeSpeed);

  private static final LoggedTunableNumber m_beltRps =
      new LoggedTunableNumber("Hopper/beltRps", HopperConstants.kdefaultBeltSpeed);

  private static final LoggedTunableNumber m_intakeMotor =
      new LoggedTunableNumber("Hopper/IntakeRps", HopperConstants.kdefaultIntakeSpeed);

  private static final LoggedTunableNumber m_beltMotor =
      new LoggedTunableNumber("Hopper/beltRps", HopperConstants.kdefaultBeltSpeed);

  // Hopper logged tunables
  private static final LoggedTunableNumber kHopperMaxPosition =
      new LoggedTunableNumber("Hopper/kHopperMaxPosition", HopperConstants.HOPPER_MAX_POSITION);

  private static final LoggedTunableNumber kHopperMinPosition =
      new LoggedTunableNumber("Hopper/kHopperMaxPosition", HopperConstants.HOPPER_MIN_POSITION);

  private static final LoggedTunableNumber kHopperMotorMMCruiseVelocity =
      new LoggedTunableNumber(
          "Hopper/MMCruiseVelocity", HopperConstants.HOPPER_MOTOR_MM_CRUISE_VELOCITY);

  private static final LoggedTunableNumber kHopperMotorMMAcceleration =
      new LoggedTunableNumber(
          "Hopper/MMAcceleration", HopperConstants.HOPPER_MOTOR_MM_ACCELERATION);

  private static final LoggedTunableNumber kHopperMotorMMJerk =
      new LoggedTunableNumber("Hopper/MMJerk", HopperConstants.HOPPER_MOTOR_MM_JERK);

  // PID values
  private static final LoggedTunableNumber kHopper_kP =
      new LoggedTunableNumber("Hopper/kP", HopperConstants.HOPPER_KP);
  private static final LoggedTunableNumber kHopper_kI =
      new LoggedTunableNumber("Hopper/kI", HopperConstants.HOPPER_KI);
  private static final LoggedTunableNumber kHopper_kD =
      new LoggedTunableNumber("Hopper/kD", HopperConstants.HOPPER_KD);
  private static final LoggedTunableNumber kHopper_kS =
      new LoggedTunableNumber("Hopper/kS", HopperConstants.HOPPER_KS);
  private static final LoggedTunableNumber kHopper_kV =
      new LoggedTunableNumber("Hopper/kV", HopperConstants.HOPPER_KV);
  private static final LoggedTunableNumber kHopper_kA =
      new LoggedTunableNumber("Hopper/kA", HopperConstants.HOPPER_KA);

  // Constructor
  public HopperIOTalonFX() {
    beltMotor = new TalonFX(hopperConfig.beltMotorId(), hopperConfig.CanBusName());
    intakeMotor = new TalonFX(hopperConfig.intakeMotorId(), hopperConfig.CanBusName());
    hopperMotor = new TalonFX(hopperConfig.hopperMotorId(), hopperConfig.CanBusName());
    agitatorMotor = new TalonFX(hopperConfig.agitatorMotorId(), hopperConfig.CanBusName());

    configureHopperMotor(hopperMotor);
    configureBeltMotor(beltMotor);
    configureIntakeMotor(intakeMotor);
    configureAgitatorMotor(agitatorMotor);
  }

  // set intake velocity
  // set belt velocity
  // open/close hopper

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

    motor.getConfigurator().apply(config);

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    // Phoenix6Util.applyAndCheckConfiguration(intakeMotor, config, intakeConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    // FaultReporter.getInstance()
    //   .registerHardware(HopperConstants.SUBSYSTEM_NAME, "Hopper Motor Intake", motor);
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

    motor.getConfigurator().apply(config);

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    // Phoenix6Util.applyAndCheckConfiguration(beltMotor, config, beltConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    // FaultReporter.getInstance()
    // .registerHardware(HopperConstants.SUBSYSTEM_NAME, "Hopper Motor Belt", motor);
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

    motor.getConfigurator().apply(config);

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    // Phoenix6Util.applyAndCheckConfiguration(beltMotor, config, beltConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    // FaultReporter.getInstance()
    // .registerHardware(HopperConstants.SUBSYSTEM_NAME, "Hopper Motor Belt", motor);
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

    config.Slot0.kV = 0.105; // 0.128;
    config.Slot0.kP = 0.13; // 0.15;
    config.Slot0.kD = 0.0;

    config.Slot0.kP = kHopper_kP.get();
    config.Slot0.kI = kHopper_kI.get();
    config.Slot0.kD = kHopper_kD.get();
    config.Slot0.kS = kHopper_kS.get();
    config.Slot0.kV = kHopper_kV.get();
    config.Slot0.kA = kHopper_kA.get();

    // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kHopperMaxPosition.get();
    // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kHopperMinPosition.get();

    config.MotionMagic.MotionMagicCruiseVelocity =
        kHopperMotorMMCruiseVelocity.get(); // 80 rps cruise velocity
    config.MotionMagic.MotionMagicAcceleration =
        kHopperMotorMMAcceleration.get(); // 160 rps/s acceleration (0.5 seconds)
    config.MotionMagic.MotionMagicJerk =
        kHopperMotorMMJerk.get(); // 1600 rps/s^2 jerk (0.1 seconds)

    hopperMotor.setPosition(0);

    motor.getConfigurator().apply(config);

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    // Phoenix6Util.applyAndCheckConfiguration(intakeMotor, config, intakeConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    // FaultReporter.getInstance()
    //   .registerHardware(HopperConstants.SUBSYSTEM_NAME, "Hopper Motor Intake", motor);
  }

  // Motion Magic Controls
  @Override
  public void setHopperPosition(double position) {
    if (position > kHopperMaxPosition.get()) {
      position = kHopperMaxPosition.get();
    } else if (position < kHopperMinPosition.get()) {
      position = kHopperMinPosition.get();
    }

    hopperMotor.setControl(hopperMotionMagicControl.withPosition(position));
  }

  // @Override
  // public void setHopperSpeed(double speed) {
  //   if (speed <= 0 && mHopperPos <= kHopperMinPosition.get()) {
  //     setHopperPosition(kHopperMinPosition.get());
  //   } else if (speed >= 0 && mHopperPos >= kHopperMaxPosition.get()) {
  //     setHopperPosition(kHopperMaxPosition.get());
  //   } else {
  //     hopperMotor.set(speed);
  //   }
  // }

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
    intakeMotor.setControl(m_intakeVelocityRequest.withVelocity(m_intakeRps.get()));
  }

  @Override
  public void runClearIntake() {
    intakeMotor.setControl(m_intakeVelocityRequest.withVelocity(-75));
  }

  @Override
  public void reverseIntake() {
    intakeMotor.set(-0.5);
  }

  @Override
  public void runBelt() {
    beltMotor.setControl(m_beltVelocityRequest.withVelocity(m_intakeRps.get()));
  }

  @Override
  public void runClearBelt() {
    beltMotor.setControl(m_beltVelocityRequest.withVelocity(-m_intakeRps.get()));
  }

  @Override
  public void stopIntake() {
    intakeMotor.setControl(m_intakeNeutralOut);
  }

  @Override
  public void stopBelt() {
    beltMotor.setControl(m_beltNeutralOut);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {

    double intakeRps = intakeMotor.getVelocity().getValueAsDouble();
    double beltRps = beltMotor.getVelocity().getValueAsDouble();
    inputs.hopperPositon = hopperMotor.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("intakeRps", intakeRps);
    SmartDashboard.putNumber("intakeRpms", intakeRps * 60);

    SmartDashboard.putNumber("beltRps", beltRps);
    SmartDashboard.putNumber("beltRpms", beltRps * 60);
  }
}
