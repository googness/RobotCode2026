package frc.robot.commands.Vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.*;
import frc.robot.subsystems.vision.VisionSystem;
import java.util.function.DoubleSupplier;

public class ShootingZoneCommand extends Command {

  private final SwerveDrivetrain drivetrain;
  private final VisionSystem vision;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final ProfiledPIDController thetaController;

  // --- BLUE ALLIANCE AIMING TARGETS ---
  private final Translation2d blueLeftTarget = new Translation2d(4.880, 4.016);
  private final Translation2d blueCenterTarget = new Translation2d(4.621, 4.029); // True center
  private final Translation2d blueRightTarget = new Translation2d(4.880, 4.016);

  // --- RED ALLIANCE AIMING TARGETS ---
  private final Translation2d redLeftTarget = new Translation2d(11.919, 4.041);
  private final Translation2d redCenterTarget = new Translation2d(11.919, 4.041); // True center
  private final Translation2d redRightTarget = new Translation2d(11.919, 4.041);

  public ShootingZoneCommand(
      SwerveDrivetrain drivetrain,
      VisionSystem vision,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    this.drivetrain = drivetrain;
    this.vision = vision;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    this.thetaController =
        new ProfiledPIDController(
            RobotConfig.getInstance().getDriveFacingAngleThetaKP(),
            RobotConfig.getInstance().getDriveFacingAngleThetaKI(),
            RobotConfig.getInstance().getDriveFacingAngleThetaKD(),
            new TrapezoidProfile.Constraints(
                3, // speed
                5 // acceleration
                ));

    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.thetaController.setTolerance(0.0355, 1);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.vision.resetOdometryToVisionPose();
  }

  @Override
  public void execute() {

    // 1. Get driver joysticks and apply deadbands
    double rawX = xSupplier.getAsDouble();
    double rawY = ySupplier.getAsDouble();

    double deadbandedX = MathUtil.applyDeadband(rawX, 0.1);
    double deadbandedY = MathUtil.applyDeadband(rawY, 0.1);

    // Allow driver inputs for translating while aiming
    double xVelocity =
        deadbandedX * RobotConfig.getInstance().getRobotMaxVelocity().in(MetersPerSecond);
    double yVelocity =
        deadbandedY * RobotConfig.getInstance().getRobotMaxVelocity().in(MetersPerSecond);

    // 2. Get your perfectly accurate Pose
    Pose2d currentPose = drivetrain.getPose();

    // 3. Find out which ideal shooting spot we are closest to (our "Zone")
    Pose2d nearestSpot = DriveToShoot.getNearestTarget(currentPose);

    // 4. Check what alliance we are on
    boolean isRed = false;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      isRed = true;
    }

    // 5. SWITCH to the specific target coordinates based on our Zone!
    Translation2d targetHub;

    if (nearestSpot.getY() > 5.0) {
      // We are in the Left Zone
      targetHub = isRed ? redLeftTarget : blueLeftTarget;
    } else if (nearestSpot.getY() < 3.0) {
      // We are in the Right Zone
      targetHub = isRed ? redRightTarget : blueRightTarget;
    } else {
      // We are in the Center Zone
      targetHub = isRed ? redCenterTarget : blueCenterTarget;
    }

    // 6. Calculate the exact compass angle to our chosen target Hub
    Rotation2d targetAngle = targetHub.minus(currentPose.getTranslation()).getAngle();

    // 7. Run the PID math to get the spin speed
    double omegaRadiansPerSecond =
        thetaController.calculate(currentPose.getRotation().getRadians(), targetAngle.getRadians());

    // --- FIX: Overcome Carpet Friction (kS) ---
    // If we are not at the goal yet, make sure the speed is high enough to actually move the robot
    if (!thetaController.atGoal() && omegaRadiansPerSecond != 0.0) {

      // Find which direction the PID wants us to spin (1.0 for Left, -1.0 for Right)
      double spinDirection = Math.signum(omegaRadiansPerSecond);

      // TUNE THIS NUMBER: The minimum radians per second needed to make your robot barely creep on
      // the carpet.
      // Start at 0.15. If the robot violently twitches back and forth, lower it to 0.1 or 0.05.
      // If it still gets stuck short of the target, raise it to 0.2.
      double minimumFrictionSpeed = 0.425;

      // Add the minimum speed in the correct direction
      omegaRadiansPerSecond += (spinDirection * minimumFrictionSpeed);
    }

    // Stop spinning if we are perfectly aimed within tolerance
    if (thetaController.atGoal()) {
      omegaRadiansPerSecond = 0.0;
    }

    // 9. Drive the robot
    drivetrain.drive(
        MetersPerSecond.of(xVelocity),
        MetersPerSecond.of(yVelocity),
        RadiansPerSecond.of(omegaRadiansPerSecond),
        true, // field relative
        true // open loop
        );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
