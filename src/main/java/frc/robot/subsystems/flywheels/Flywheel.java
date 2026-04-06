package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.vision.VisionSystem;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private static final LoggedTunableNumber mFlywheelRps =
      new LoggedTunableNumber("Flywheel/FlywheelRps", FlywheelConstants.kDefaultShootingRps);

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

    // SmartDashboard.putNumber("distToHubNotRps", 0);
    // distanceToRpsMap.put(100.0, 41.0);
    // distanceToRpsMap.put(105.0, 42.0);
    // distanceToRpsMap.put(110.0, 43.0);
    // distanceToRpsMap.put(115.0, 43.5);
    // distanceToRpsMap.put(120.0, 44.0);
    // distanceToRpsMap.put(125.0, 44.5);
    // distanceToRpsMap.put(130.0, 45.0);
    // distanceToRpsMap.put(135.0, 45.5);
    // distanceToRpsMap.put(145.0, 46.5); //
    // distanceToRpsMap.put(155.0, 47.0); //

    SmartDashboard.putNumber("distToHubNotRps", 0);
    distanceToRpsMap.put(75.0, 37.0);
    distanceToRpsMap.put(80.0, 37.5);
    distanceToRpsMap.put(85.0, 38.0);
    distanceToRpsMap.put(90.0, 38.5);
    distanceToRpsMap.put(100.0, 41.0);
    distanceToRpsMap.put(105.0, 41.5);
    distanceToRpsMap.put(110.0, 42.0);
    distanceToRpsMap.put(115.0, 42.5);
    distanceToRpsMap.put(120.0, 43.0);
    distanceToRpsMap.put(125.0, 43.5);
    distanceToRpsMap.put(130.0, 44.0);
    distanceToRpsMap.put(135.0, 45.0);
    distanceToRpsMap.put(140.0, 45.5);
    distanceToRpsMap.put(145.0, 46.5);
    distanceToRpsMap.put(150.0, 46.5);
    distanceToRpsMap.put(155.0, 47.0);
    distanceToRpsMap.put(160.0, 48.0); //
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs(SUBSYSTEM_NAME, inputs);
  }

  private double mVelocitySetpoint = 0;
  private double mDistanceTolerance = 3;
  private double mLastDistance = 0;

  public void setVelocity() {
    setVelocity(mFlywheelRps.get());
  }

  public void setVelocity(double rps) {
    io.runVelocity(rps);
    mVelocitySetpoint = rps;
  }

  public void stopFlywheel() {
    io.stopFlywheel();
    mVelocitySetpoint = 0;
  }

  public Command runScoreWithVisionCmd(VisionSystem vision) {
    return Commands.run(
            () -> {
              double currentDistance = Units.metersToInches(vision.distanceToHub());

              Logger.recordOutput("FlywheelStuff/VelocitySetpoint", mVelocitySetpoint);
              Logger.recordOutput("FlywheelStuff/CurrentDistance", currentDistance);
              Logger.recordOutput("FlywheelStuff/LastDistance", mLastDistance);

              if (Math.abs(currentDistance - mLastDistance) > mDistanceTolerance) {
                double newSpeed = distanceToRpsMap.get(currentDistance);

                Logger.recordOutput("FlywheelStuff/newSpeed", newSpeed);

                setVelocity(newSpeed);
                mLastDistance = currentDistance;
              }
            },
            this)
        .finallyDo(
            () -> {
              stopFlywheel();
              mLastDistance = 0;
            });
  }
}
