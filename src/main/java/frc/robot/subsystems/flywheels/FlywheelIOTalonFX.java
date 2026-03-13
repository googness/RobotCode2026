package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.configs.ThunderRobotConfig;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX frontShooter;
  private final TalonFX backShooter;
  private final TalonFX acceleratorMotor;

  private static final LoggedTunableNumber m_frontShooter =
      new LoggedTunableNumber("Flywheel/frontShooterRps", FlywheelConstants.kShootRps);
  private static final LoggedTunableNumber m_backShooter =
      new LoggedTunableNumber("Flywheel/backShooterRps", FlywheelConstants.kShootRps);
  private static final LoggedTunableNumber m_feederMotor =
      new LoggedTunableNumber("Flywheel/feederMotorRps", FlywheelConstants.kFeedRpm);

  private final VelocityVoltage m_frontVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage m_backVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage m_feederVelocityRequest =
      new VelocityVoltage(0).withEnableFOC(true);

  private final NeutralOut m_frontNeutralOut = new NeutralOut();
  private final NeutralOut m_backNeutralOut = new NeutralOut();
  private final NeutralOut m_feedeNeutralOut = new NeutralOut();

  private Alert frontConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert backConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert accelerationConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private int PH_CAN_ID = ThunderRobotConfig.PNEUMATICS_HUB_ID;
  private static int hoodForwardChannel = 4;
  private static int hoodReverseChannel = 5;

  PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
  DoubleSolenoid hoodDoubleSolenoid =
      m_pH.makeDoubleSolenoid(hoodForwardChannel, hoodReverseChannel);

  public FlywheelIOTalonFX() {
    frontShooter = new TalonFX(kFlywheelConfig.frontMotorCANId(), kFlywheelConfig.canBusName());
    backShooter = new TalonFX(kFlywheelConfig.backMotorCANId(), kFlywheelConfig.canBusName());
    acceleratorMotor =
        new TalonFX(kFlywheelConfig.acceleratorMotorCANId(), kFlywheelConfig.canBusName());

    hoodDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  // set front velocity
  // set bottom velocity
  // set acceleration velocity
  // set hood position (Up or Down)

  public void configureFrontMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.2;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = 0.13; // .11
    config.Slot0.kV = 0.130; // .117;
    config.Slot0.kS = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kD = 0.0;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(frontShooter, config, frontConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance()
        .registerHardware(FlywheelConstants.SUBSYSTEM_NAME, "Flywheel Motor Front", motor);
  }

  public void configureBackMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.2;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = 0.13; // .11
    config.Slot0.kV = 0.130; // .117;
    config.Slot0.kS = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kD = 0.0;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(backShooter, config, backConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance()
        .registerHardware(FlywheelConstants.SUBSYSTEM_NAME, "Flywheel Motor Back", motor);
  }

  public void configureAccelerationMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.Slot0.kP = 0.11; // .11
    config.Slot0.kV = 0.132; // .117;

    config.Slot0.kD = 0.0;

    // config.CurrentLimits.StatorCurrentLimit = 60;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;

    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    //    Phoenix6Util.applyAndCheckConfiguration(acceleratorMotor, config,
    // accelerationConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    //  FaultReporter.getInstance()
    //  .registerHardware(FlywheelConstants.SUBSYSTEM_NAME, "Flywheel Motor Acceleration", motor);
  }

  @Override
  public void runVelocity() {
    frontShooter.setControl(m_frontVelocityRequest.withVelocity(m_frontShooter.get()));
    backShooter.setControl(m_backVelocityRequest.withVelocity(m_backShooter.get()));
  }

  @Override
  public void runVelocity(double rps) {
    frontShooter.setControl(m_frontVelocityRequest.withVelocity(rps));
    backShooter.setControl(m_backVelocityRequest.withVelocity(rps));
  }

  @Override
  public void stopFlywheel() {
    frontShooter.setControl(m_frontNeutralOut);
    backShooter.setControl(m_backNeutralOut);
  }

  @Override
  public void runAccelerator() {
    acceleratorMotor.setControl(m_feederVelocityRequest.withVelocity(m_feederMotor.get()));
  }

  @Override
  public void stopAccelerator() {
    acceleratorMotor.setControl(m_feedeNeutralOut);
  }

  @Override
  public void extendHood() {
    hoodDoubleSolenoid.set(Value.kForward);
  }

  @Override
  public void retractHood() {
    hoodDoubleSolenoid.set(Value.kReverse);
  }

  public void periodic() {

    double frontShooterRps = frontShooter.getVelocity().getValueAsDouble();
    double backShooterRps = backShooter.getVelocity().getValueAsDouble();
    double acceleratorMotorRps = acceleratorMotor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("frontShooterRps", frontShooterRps);
    SmartDashboard.putNumber("frontShooterRpms", frontShooterRps * 60);

    SmartDashboard.putNumber("backShooterRps", backShooterRps);
    SmartDashboard.putNumber("backShooterRpm", backShooterRps * 60);

    SmartDashboard.putNumber("acceleratorMotorRps", acceleratorMotorRps);
    SmartDashboard.putNumber("acceleratorMotorRpm", acceleratorMotorRps * 60);
  }
}
