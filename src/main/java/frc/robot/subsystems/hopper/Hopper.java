package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Hopper extends SubsystemBase {

  private final HopperIO io;

  private final HopperIOInputsAutoLogged HopperInputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // the first step in periodic is to update the inputs from the IO implementation.
    io.updateInputs(HopperInputs);
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

  // extend the hopper
  public void extendHopper() {
    io.extendHopper();
  }

  // retract the hopper
  public void retractHopper() {
    io.retractHopper();
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
        .andThen(new WaitCommand(2.5).andThen(() -> stopIntake()))
        .handleInterrupt(() -> stopIntake());
  }
}
