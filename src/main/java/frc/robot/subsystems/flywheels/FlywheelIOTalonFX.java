package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.configs.ThunderRobotConfig;

public class FlywheelIOTalonFX implements FlywheelIO {

  private int PH_CAN_ID = ThunderRobotConfig.PNEUMATICS_HUB_ID;
  private static int hoodForwardChannel = 7;
  private static int hoodReverseChannel = 8;

  PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
  DoubleSolenoid hoodDoubleSolenoid =
      m_pH.makeDoubleSolenoid(hoodForwardChannel, hoodReverseChannel);

  private final TalonFX frontMotor;
  private final TalonFX backMotor;

  public FlywheelIOTalonFX() {
    frontMotor = new TalonFX(kFlywheelConfig.frontMotorCANId(), kFlywheelConfig.canBusName());
    backMotor = new TalonFX(kFlywheelConfig.backMotorCANId(), kFlywheelConfig.canBusName());
  }
}
