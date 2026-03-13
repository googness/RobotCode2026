package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.configs.ThunderRobotConfig;

public class HopperIOTalonFX implements HopperIO {

  private TalonFX beltMotor;
  private TalonFX intakeMotor;

  private static final LoggedTunableNumber m_intakeRps =
      new LoggedTunableNumber("Hopper/IntakeRps", HopperConstants.kdefaultIntakeSpeed);
  private static final LoggedTunableNumber m_beltRps =
      new LoggedTunableNumber("Hopper/beltRps", HopperConstants.kdefaultBeltSpeed);

  private Alert beltConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert intakeConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private int PH_CAN_ID = ThunderRobotConfig.PNEUMATICS_HUB_ID;
  private static int leftIntakeForwardChannel = 6;
  private static int leftIntakeReverseChannel = 7;

  private static int rightIntakeForwardChannel = 2;
  private static int rightIntakeReverseChannel = 3;

  private final VelocityVoltage m_intakeVelocityRequest =
      new VelocityVoltage(0).withEnableFOC(true);
  private final NeutralOut m_intakeNeutralOut = new NeutralOut();

  private final VelocityVoltage m_beltVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final NeutralOut m_beltNeutralOut = new NeutralOut();

  private static final LoggedTunableNumber m_intakeMotor =
      new LoggedTunableNumber("Hopper/IntakeRps", HopperConstants.kdefaultIntakeSpeed);
  private static final LoggedTunableNumber m_beltMotor =
      new LoggedTunableNumber("Hopper/beltRps", HopperConstants.kdefaultBeltSpeed);

  PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
  DoubleSolenoid leftIntakeDoubleSolenoid =
      m_pH.makeDoubleSolenoid(leftIntakeForwardChannel, leftIntakeReverseChannel);
  DoubleSolenoid rightIntakeDoubleSolenoid =
      m_pH.makeDoubleSolenoid(rightIntakeForwardChannel, rightIntakeReverseChannel);

  public HopperIOTalonFX() {
    beltMotor = new TalonFX(hopperConfig.beltMotorId(), hopperConfig.CanBusName());
    intakeMotor = new TalonFX(hopperConfig.intakeMotorId(), hopperConfig.CanBusName());

    rightIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    leftIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  // set intake velocity
  // set belt velocity
  // open/close hopper

  public void configureIntakeMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
  public void extendHopper() {
    leftIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    rightIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void retractHopper() {
    leftIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void periodic() {

    double intakeRps = intakeMotor.getVelocity().getValueAsDouble();
    double beltRps = beltMotor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("intakeRps", intakeRps);
    SmartDashboard.putNumber("intakeRpms", intakeRps * 60);

    SmartDashboard.putNumber("beltRps", beltRps);
    SmartDashboard.putNumber("beltRpms", beltRps * 60);
  }
}
