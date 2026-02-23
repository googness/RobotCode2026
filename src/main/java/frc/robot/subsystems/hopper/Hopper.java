package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}
