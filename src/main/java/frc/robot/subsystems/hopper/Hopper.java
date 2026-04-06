package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged HopperInputs = new HopperIOInputsAutoLogged();

  private double mHopperMotorSetpoint = 0;
  private boolean mHopperExtended = false;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  public void setHopperPosition(double pos) {
    mHopperMotorSetpoint = pos;
    io.setHopperPosition(mHopperMotorSetpoint);
  }

  public void GoToIntakePOS() {
    setHopperPosition(HopperConstants.HOPPER_INTAKE_POSITION);
    mHopperExtended = true;
  }

  public void GoToHomePOS() {
    setHopperPosition(HopperConstants.HOPPER_HOME_POSITION);
    mHopperExtended = false;
  }

  public void runAgitator() {
    io.setAgitator(HopperConstants.kdefaultAgitatorSpeed);
  }

  public void runReverseAgitator() {
    io.setAgitator(-HopperConstants.kdefaultAgitatorSpeed);
  }

  public void stopAgitator() {
    io.setAgitator(0);
  }

  public void runIntake() {
    io.runIntake();
  }

  public void reverseIntake() {
    io.reverseIntake();
  }

  public void stopIntake() {
    io.stopIntake();
  }

  public void runBelt() {
    io.runBelt();
  }

  public void reverseBelt() {
    io.runClearBelt();
  }

  public void stopBelt() {
    io.stopBelt();
  }

  public void runAccelerator() {
    io.runAccelerator();
  }

  public void stopAccelerator() {
    io.stopAccelerator();
  }

  public Command FeederRpsCmd() {
    return new StartEndCommand(
        () -> {
          // double frps = SmartDashboard.getNumber("Shooter/SetFeedRps", 0.0);

          runAccelerator();
        },
        () -> stopAccelerator());
  }

  public Command runFeedBallsToShooterCmd() {
    return Commands.sequence(
            Commands.runOnce(() -> runAccelerator()),
            Commands.runOnce(() -> runBelt()),
            Commands.runOnce(() -> runReverseAgitator()),
            Commands.runOnce(() -> runIntake()),
            new WaitCommand(0.5),
            Commands.runOnce(() -> stopIntake()),
            // Commands.runOnce(() -> io.runClearBelt()),
            new WaitCommand(0.25),
            Commands.runOnce(() -> runIntake()),
            Commands.runOnce(() -> runBelt()),
            Commands.runOnce(() -> runAgitator()),
            new WaitCommand(1.25))
        .repeatedly()
        .finallyDo(
            () -> {
              stopBelt();
              stopIntake();
              stopAgitator();
              stopAccelerator();
            });
  }

  private double getPulse() {
    if (mHopperExtended) {
      return mHopperMotorSetpoint > HopperConstants.HOPPER_PULSE_IN_EXTENDED_POSITION
          ? HopperConstants.HOPPER_PULSE_IN_EXTENDED_POSITION
          : HopperConstants.HOPPER_PULSE_OUT_EXTENDED_POSITION;
    }

    return mHopperMotorSetpoint > HopperConstants.HOPPER_PULSE_IN_POSITION
        ? HopperConstants.HOPPER_PULSE_IN_POSITION
        : HopperConstants.HOPPER_PULSE_OUT_POSITION;
  }

  public Command runHopperPulseCmd() {
    return Commands.sequence(
            new WaitCommand(0.5), Commands.runOnce(() -> setHopperPosition(getPulse())))
        .repeatedly()
        .finallyDo(
            () -> {
              if (mHopperExtended) {
                GoToIntakePOS();
              } else {
                GoToHomePOS();
              }
            });
  }

  @Override
  public void periodic() {
    io.updateInputs(HopperInputs);

    Logger.processInputs(HopperConstants.SUBSYSTEM_NAME, HopperInputs);
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

  // Manual Command
  public void setHopperSpeed(double speed) {
    io.setHopperSpeed(speed);
  }
}
