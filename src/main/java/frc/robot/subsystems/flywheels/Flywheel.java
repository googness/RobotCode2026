package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final InterpolatingDoubleTreeMap distanceToRpsMap = new InterpolatingDoubleTreeMap();

  private double distRemove = 23.5 + 12.5 + 3.5;

  private double lastDistanceRequested = 0.0;

  // 3. Create a method to get the interpolated RPS based on current distance
  public double getTargetRps(double distanceInches) {
    // The .get() method automatically interpolates values between your data points.
    // Note: If distanceInches > 90, it will cap at 49.0. If < 40, it caps at 39.0.

    lastDistanceRequested = distanceInches;
    SmartDashboard.putNumber("distToHubNotRps", lastDistanceRequested);
    return distanceToRpsMap.get(distanceInches - distRemove);
  }

  public Flywheel(FlywheelIO io) {
    this.io = io;

    SmartDashboard.putNumber("distToHubNotRps", 0);

    // distanceToRpsMap.put(40.0, 39.0);
    // distanceToRpsMap.put(45.0, 40.0);
    // distanceToRpsMap.put(50.0, 41.0);
    // distanceToRpsMap.put(55.0, 42.0);
    // distanceToRpsMap.put(60.0, 43.0);
    // distanceToRpsMap.put(65.0, 44.0);
    // distanceToRpsMap.put(70.0, 45.0);
    // distanceToRpsMap.put(75.0, 46.0);
    // distanceToRpsMap.put(80.0, 47.0);
    // distanceToRpsMap.put(90.0, 49.0);

    distanceToRpsMap.put(30.0, 41.0);
    distanceToRpsMap.put(35.0, 42.0);
    distanceToRpsMap.put(40.0, 43.0);
    distanceToRpsMap.put(45.0, 43.5);
    distanceToRpsMap.put(50.0, 44.0);
    distanceToRpsMap.put(55.0, 44.5);
    distanceToRpsMap.put(60.0, 45.0);
    distanceToRpsMap.put(65.0, 45.5);
    distanceToRpsMap.put(70.0, 46.5);
    distanceToRpsMap.put(75.0, 47.0);
    // distanceToRpsMap.put(80.0, 47.5);
    // distanceToRpsMap.put(85.0, 48.5);
    // distanceToRpsMap.put(90.0, 49.0);
  }

  @Override
  public void periodic() {
    // updating the inputs
    io.updateInputs(inputs);

    // putting the inputs into advantagescope
    Logger.processInputs(SUBSYSTEM_NAME, inputs);
  }

  // Set the velocity of the flywheels
  public void setVelocity() {
    io.runVelocity();
  }

  public void setVelocity(double rps) {
    io.runVelocity(rps);
  }

  // stop the flywheels
  public void stopFlywheel() {
    io.stopFlywheel();
  }

  public void runAccelerator() {
    io.runAccelerator();
  }

  public void stopAccelerator() {
    io.stopAccelerator();
  }

  public void testSpeed() {
    io.testSpeed();
  }

  //   public Command ShootRpsCmd() {
  //     return new StartEndCommand(
  //         () -> {
  //             double frps = SmartDashboard.getNumber("Shooter/SetFrontRps", 0.0);
  //             double brps = SmartDashboard.getNumber("Shooter/SetBackRps", 0.0);
  //             runFlywheel(frps,brps);
  //         },
  //         () -> stop());
  // }

  public Command FeederRpsCmd() {
    return new StartEndCommand(
        () -> {
          // double frps = SmartDashboard.getNumber("Shooter/SetFeedRps", 0.0);

          runAccelerator();
        },
        () -> stopAccelerator());
  }
}
