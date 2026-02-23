package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.configs.ThunderRobotConfig;

public class HopperIOTalonFX implements HopperIO {

  private int PH_CAN_ID = ThunderRobotConfig.PNEUMATICS_HUB_ID;
  private static int leftIntakeForwardChannel = 3;
  private static int leftIntakeReverseChannel = 4;

  private static int rightIntakeForwardChannel = 5;
  private static int rightIntakeReverseChannel = 6;

  PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
  DoubleSolenoid leftIntakeDoubleSolenoid =
      m_pH.makeDoubleSolenoid(leftIntakeForwardChannel, leftIntakeReverseChannel);
  DoubleSolenoid rightIntakeDoubleSolenoid =
      m_pH.makeDoubleSolenoid(rightIntakeForwardChannel, rightIntakeReverseChannel);

  private TalonFX beltMotor;
  private TalonFX intakeMotor;

  public HopperIOTalonFX() {
    beltMotor = new TalonFX(hopperConfig.beltMotorId(), hopperConfig.CanBusName());
    intakeMotor = new TalonFX(hopperConfig.intakeMotorId(), hopperConfig.CanBusName());

    // rightIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }
}
