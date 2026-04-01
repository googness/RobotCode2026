package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO io;

  private final HopperIOInputsAutoLogged HopperInputs = new HopperIOInputsAutoLogged();

  private double mHopperMotorSetpoint = 0;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // the first step in periodic is to update the inputs from the IO implementation.
    io.updateInputs(HopperInputs);

    Logger.processInputs(HopperConstants.SUBSYSTEM_NAME, HopperInputs);
  }

  public void GoToIntakePOS() {
    setHopperPosition(HopperConstants.HOPPER_INTAKE_POSITION);
  }

  public void GoToHomePOS() {
    setHopperPosition(HopperConstants.HOPPER_HOME_POSITION);
  }

  public void GoToHalfPOS() {
    setHopperPosition(HopperConstants.HOPPER_HALF_POSITION);
  }

  public void GoToAgitatePOS() {
    setHopperPosition(HopperConstants.HOPPER_AGITATE_POSITION);
  }

  public void setHopperPosition(double pos) {
    mHopperMotorSetpoint = pos;
    io.setHopperPosition(mHopperMotorSetpoint);
  }

  // public void setHopperSpeed(double speed) {
  //   // setState(State.MANUAL);
  //   if (speed != 0) {
  //     io.setHopperSpeed(speed);
  //   } else {
  //     io.stopHopper();
  //   }
  // }

  public void setHopperSpeed(double speed) {
    io.setHopperSpeed(speed);
  }

  public void setAgitator(double speed) {
    io.setAgitator(speed);
  }

  public double getCurrentElevatorPos() {
    return HopperInputs.hopperPositon;
  }

  // set the speed of the intake
  public void runIntake() {
    io.runIntake();
  }

  public void reverseIntake() {
    io.reverseIntake();
  }

  public void stopIntake() {
    io.stopIntake();
  }

  // set the speed of the belt
  public void runBelt() {
    io.runBelt();
  }

  public void stopBelt() {
    io.stopBelt();
  }

  public Command RunIntakeCmd() {
    return new StartEndCommand(
        () -> {
          runIntake();
        },
        () -> stopIntake());
  }

  public Command RunBeltCmd() {
    return new StartEndCommand(
        () -> {
          runBelt();
        },
        () -> stopBelt());
  }

  public Command RunClearBeltCmd() {
    return new WaitCommand(.25).andThen(() -> io.runClearBelt());
  }

  public Command RunClearIntakeCmd() {
    return new WaitCommand(0.25)
        .andThen(
            () -> {
              // double rps = SmartDashboard.getNumber("Hopper/SetIntakeClearRps", 0.0);
              // runIntake(rps);
              io.runClearIntake();
            })
        .andThen(new WaitCommand(4).andThen(() -> stopIntake()))
        .handleInterrupt(() -> stopIntake());
  }
}
