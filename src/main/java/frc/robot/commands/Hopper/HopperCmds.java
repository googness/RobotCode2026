package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.hopper.*;

public class HopperCmds {

  private HopperCmds() {}

  public static Command runIntake(Hopper hopper) {
    return new InstantCommand(() -> hopper.runIntake());
  }

  public static Command stopIntake(Hopper hopper) {
    return new InstantCommand(() -> hopper.stopIntake());
  }

  public static Command runBelt(Hopper hopper) {
    return new InstantCommand(() -> hopper.runBelt());
  }

  public static Command stopBelt(Hopper hopper) {
    return new InstantCommand(() -> hopper.stopBelt());
  }

  public static Command reverseIntake(Hopper hopper) {
    return new InstantCommand(() -> hopper.reverseIntake());
  }

  public static Command runAccelerator(Hopper hopper) {
    return new InstantCommand(() -> hopper.runAccelerator());
  }

  public static Command stopAccelerator(Hopper hopper) {
    return new InstantCommand(() -> hopper.stopAccelerator());
  }

  // St Cloud Commands
  public static Command GoToIntakePos(Hopper hopper) {
    return new InstantCommand(() -> hopper.GoToIntakePOS());
  }

  public static Command GoToHomePos(Hopper hopper) {
    return new InstantCommand(() -> hopper.GoToHomePOS());
  }

  public static Command SetHopperPOS(double pos, Hopper hopper) {
    return new InstantCommand(() -> hopper.setHopperPosition(pos));
  }

  public static Command autonomousIntake(Hopper hopper) {
    return new InstantCommand(hopper::GoToIntakePOS)
        .alongWith(new WaitCommand(0.15))
        .andThen(hopper::runIntake);
  }
}
