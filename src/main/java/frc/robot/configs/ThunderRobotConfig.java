package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class ThunderRobotConfig extends RobotConfig {

  // : update all CAN IDs and steer offsets
  // private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
  // private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8;
  // private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
  // private static final Angle FRONT_LEFT_MODULE_STEER_OFFSET = Rotations.of(-0.09619140625);

  // private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10;
  // private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11;
  // private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
  // private static final Angle FRONT_RIGHT_MODULE_STEER_OFFSET = Rotations.of(0.4208984375);

  // private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 13;
  // private static final int BACK_LEFT_MODULE_STEER_MOTOR = 14;
  // private static final int BACK_LEFT_MODULE_STEER_ENCODER = 15;
  // private static final Angle BACK_LEFT_MODULE_STEER_OFFSET = Rotations.of(0.489501953125);

  // private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
  // private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 17;
  // private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 18;
  // private static final Angle BACK_RIGHT_MODULE_STEER_OFFSET = Rotations.of(-0.34033203125);

  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 16;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 17;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 18;
  private static final Angle FRONT_LEFT_MODULE_STEER_OFFSET = Rotations.of(0.159912109375);

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 14;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 15;
  private static final Angle FRONT_RIGHT_MODULE_STEER_OFFSET = Rotations.of(-0.008544921875);

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
  private static final Angle BACK_LEFT_MODULE_STEER_OFFSET = Rotations.of(-0.07861328125);

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9;
  private static final Angle BACK_RIGHT_MODULE_STEER_OFFSET = Rotations.of(0.40380859375);

  private static final int GYRO_ID = 19;

  // : update robot dimensions
  private static final Mass MASS =
      Kilograms.of(54.4311); // : update based on measured mass of robot with battery and bumpers
  private static final MomentOfInertia MOI = KilogramSquareMeters.of(6.0); // : measure
  private static final Distance TRACKWIDTH = Meters.of(0.55245); // 21
  private static final Distance WHEELBASE = Meters.of(0.55245);

  // JAY: (Odemetry Issue).  This is probably your odemetry issue.  I will explain later but I dont
  // see any issues with your code, the math or the conversions
  // -- This solves all the riddles.  Wheel is bigger than 4 inches.  Try 0.0515.  Over 20 feet
  // thats over 3 inches further.  Error is increased when turning.
  // -- ** Another issue I found plus this may be causing your aim issue both over time and
  // sparatic.
  private static final Distance WHEEL_RADIUS = Meters.of(0.0508); // Meters.of(0.0508);

  private static final double WHEEL_COEFFICIENT_OF_FRICTION =
      1.2; // : update based on wheel coefficient of friction
  private static final Translation2d FRONT_RIGHT_CORNER_POSITION =
      new Translation2d(0.3429, -0.3429);
  private static final Distance ROBOT_WIDTH_WITH_BUMPERS = Meters.of(0.876);
  private static final Distance ROBOT_LENGTH_WITH_BUMPERS = Meters.of(0.876);

  private static final double COUPLE_RATIO = 3.857142857142857; // : tune

  // : tune PID values for the angle and drive motors for the swerve modules

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 100.0;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.5;

  // : characterize the drivetrain and update these constants
  private static final double ANGLE_KS = 0.1;
  private static final double ANGLE_KV = 2.49; // convert from V/(radians/s) to V/(rotations/s)
  private static final double ANGLE_KA = 0;

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 0.1;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;

  // : characterize the drivetrain and update these constants
  private static final double DRIVE_KS = 0.16217;
  private static final double DRIVE_KV = 0.12269;
  private static final double DRIVE_KA = 0.0;

  // : determine maximum velocities empirically
  private static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(5.12);
  private static final LinearVelocity MAX_COAST_VELOCITY = MetersPerSecond.of(0.05);
  private static final double SLOW_MODE_MULTIPLIER = 0.75;

  // : specify the name of the CANivore CAN FD bus as appropriate (an empty string uses the
  // default CAN bus)
  private static final String CAN_BUS_NAME = "TheCan";
  private static final CANBus CAN_BUS = new CANBus(CAN_BUS_NAME);

  // : specify the name of the camera used for detecting AprilTags
  private static final String FR_CAMERA_NAME = "RightCamera";
  private static final String FL_CAMERA_NAME = "LeftCamera";
  // private static final String BR_CAMERA_NAME = "OV2311";
  // private static final String BL_CAMERA_NAME = "OV2311";

  // // : update this with the actual transform from the robot to the camera
  private static final Transform3d ROBOT_TO_FR_CAMERA =
      new Transform3d(
          new Translation3d(0.2206625, -0.2413, 0.4937125), new Rotation3d(0, 15, 30.0));

  private static final Transform3d ROBOT_TO_FL_CAMERA =
      new Transform3d(
          new Translation3d(0.2206625, 0.2413, 0.4937125), new Rotation3d(0, 15, -30.0));

  // private static final Transform3d ROBOT_TO_BR_CAMERA =
  //     new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  // private static final Transform3d ROBOT_TO_BL_CAMERA =
  //     new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  // : specify the configuration for pneumatics
  public static final int PNEUMATICS_HUB_ID = 0;
  private static final int FLOW_SENSOR_CHANNEL = 0;
  private static final int REV_HIGH_PRESSURE_SENSOR_CHANNEL = 0;
  private static final int REV_LOW_PRESSURE_SENSOR_CHANNEL = 1;

  // : specify maximum velocity and acceleration and tune PID values for auto paths
  private static final double AUTO_DRIVE_P_CONTROLLER = 5.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 5.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // : tune PID values for drive to pose
  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_KP = 2.5;
  private static final double DRIVE_TO_POSE_DRIVE_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP = 18.0;
  private static final double DRIVE_TO_POSE_THETA_KI = 10.0;
  private static final double DRIVE_TO_POSE_THETA_KD = 0.0;
  private static final Distance DRIVE_TO_POSE_DRIVE_TOLERANCE = Meters.of(0.06);
  private static final Angle DRIVE_TO_POSE_THETA_TOLERANCE = Radians.of(0.02);
  private static final LinearVelocity DRIVE_TO_POSE_MAX_VELOCITY = MetersPerSecond.of(1.25);

  private static final LinearVelocity SQUARING_SPEED = MetersPerSecond.of(1.0);

  // : tune PID values for drive facing angle
  // Drive Facing Angle constants
  private static final double DRIVE_FACING_ANGLE_KP = 3.5;
  private static final double DRIVE_FACING_ANGLE_KD = 0.02;
  private static final double DRIVE_FACING_ANGLE_KI = 0;

  // : specify the number of LEDs
  private static final int LED_COUNT = 0;

  @Override
  public boolean getPhoenix6Licensed() {
    // : return true if you have Phoenix 6 Pro license
    return true;
  }

  @Override
  public double getSwerveAngleKP() {
    return ANGLE_KP;
  }

  @Override
  public double getSwerveAngleKI() {
    return ANGLE_KI;
  }

  @Override
  public double getSwerveAngleKD() {
    return ANGLE_KD;
  }

  @Override
  public double getSwerveAngleKS() {
    return ANGLE_KS;
  }

  @Override
  public double getSwerveAngleKV() {
    return ANGLE_KV;
  }

  @Override
  public double getSwerveAngleKA() {
    return ANGLE_KA;
  }

  @Override
  public double getSwerveDriveKP() {
    return DRIVE_KP;
  }

  @Override
  public double getSwerveDriveKI() {
    return DRIVE_KI;
  }

  @Override
  public double getSwerveDriveKD() {
    return DRIVE_KD;
  }

  @Override
  public double getDriveKS() {
    return DRIVE_KS;
  }

  @Override
  public double getDriveKV() {
    return DRIVE_KV;
  }

  @Override
  public double getDriveKA() {
    return DRIVE_KA;
  }

  @Override
  public SwerveConstants getSwerveConstants() {
    // : specify the type of swerve module (MK4, MK4i, MK4n are supported)
    return SwerveConstants.MK5N_R2_CONSTANTS;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_DRIVE_MOTOR,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      BACK_LEFT_MODULE_DRIVE_MOTOR,
      BACK_RIGHT_MODULE_DRIVE_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_MOTOR,
      FRONT_RIGHT_MODULE_STEER_MOTOR,
      BACK_LEFT_MODULE_STEER_MOTOR,
      BACK_RIGHT_MODULE_STEER_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_ENCODER,
      FRONT_RIGHT_MODULE_STEER_ENCODER,
      BACK_LEFT_MODULE_STEER_ENCODER,
      BACK_RIGHT_MODULE_STEER_ENCODER
    };
  }

  @Override
  public Angle[] getSwerveSteerOffsets() {
    return new Angle[] {
      FRONT_LEFT_MODULE_STEER_OFFSET,
      FRONT_RIGHT_MODULE_STEER_OFFSET,
      BACK_LEFT_MODULE_STEER_OFFSET,
      BACK_RIGHT_MODULE_STEER_OFFSET
    };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_ID;
  }

  @Override
  public Distance getTrackwidth() {
    return TRACKWIDTH;
  }

  @Override
  public Distance getWheelbase() {
    return WHEELBASE;
  }

  @Override
  public Distance getWheelRadius() {
    return WHEEL_RADIUS;
  }

  @Override
  public Translation2d getFrontRightCornerPosition() {
    return FRONT_RIGHT_CORNER_POSITION;
  }

  @Override
  public Distance getRobotWidthWithBumpers() {
    return ROBOT_WIDTH_WITH_BUMPERS;
  }

  @Override
  public Distance getRobotLengthWithBumpers() {
    return ROBOT_LENGTH_WITH_BUMPERS;
  }

  @Override
  public LinearVelocity getRobotMaxVelocity() {
    return MAX_VELOCITY;
  }

  @Override
  public double getRobotSlowModeMultiplier() {
    return SLOW_MODE_MULTIPLIER;
  }

  @Override
  public LinearVelocity getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY;
  }

  @Override
  public double getAutoDriveKP() {
    return AUTO_DRIVE_P_CONTROLLER;
  }

  @Override
  public double getAutoDriveKI() {
    return AUTO_DRIVE_I_CONTROLLER;
  }

  @Override
  public double getAutoDriveKD() {
    return AUTO_DRIVE_D_CONTROLLER;
  }

  @Override
  public double getAutoTurnKP() {
    return AUTO_TURN_P_CONTROLLER;
  }

  @Override
  public double getAutoTurnKI() {
    return AUTO_TURN_I_CONTROLLER;
  }

  @Override
  public double getAutoTurnKD() {
    return AUTO_TURN_D_CONTROLLER;
  }

  @Override
  public Mass getMass() {
    return MASS;
  }

  @Override
  public MomentOfInertia getMomentOfInertia() {
    return MOI;
  }

  @Override
  public double getWheelCOF() {
    return WHEEL_COEFFICIENT_OF_FRICTION;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
  }

  @Override
  public CANBus getCANBus() {
    return CAN_BUS;
  }

  @Override
  public CameraConfig[] getCameraConfigs() {
    return new CameraConfig[] {
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_FR_CAMERA)
          .id(FR_CAMERA_NAME)
          .location("FR")
          .width(1280)
          .height(700)
          .stdDevFactor(0.2)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_FL_CAMERA)
          .id(FL_CAMERA_NAME)
          .location("FL")
          .width(1280)
          .height(700)
          .stdDevFactor(0.2)
          .build(),
      //     CameraConfig.builder()
      //         .robotToCameraTransform(ROBOT_TO_BR_CAMERA)
      //         .id(BR_CAMERA_NAME)
      //         .location("BR")
      //         .width(1600)
      //         .height(1200)
      //         .stdDevFactor(1.0)
      //         .build(),
      //     CameraConfig.builder()
      //         .robotToCameraTransform(ROBOT_TO_BL_CAMERA)
      //         .id(BL_CAMERA_NAME)
      //         .location("BL")
      //         .width(1600)
      //         .height(1200)
      //         .stdDevFactor(1.0)
      //         .build(),
    };
  }

  @Override
  public double getDriveToPoseDriveXKP() {
    return DRIVE_TO_POSE_DRIVE_KP;
  }

  @Override
  public double getDriveToPoseDriveYKP() {
    return DRIVE_TO_POSE_DRIVE_KP;
  }

  @Override
  public double getDriveToPoseDriveXKD() {
    return DRIVE_TO_POSE_DRIVE_KD;
  }

  @Override
  public double getDriveToPoseDriveYKD() {
    return DRIVE_TO_POSE_DRIVE_KD;
  }

  @Override
  public double getDriveToPoseThetaKI() {
    return DRIVE_TO_POSE_THETA_KI;
  }

  @Override
  public double getDriveToPoseThetaKP() {
    return DRIVE_TO_POSE_THETA_KP;
  }

  @Override
  public double getDriveToPoseThetaKD() {
    return DRIVE_TO_POSE_THETA_KD;
  }

  @Override
  public LinearVelocity getDriveToPoseDriveMaxVelocity() {
    return DRIVE_TO_POSE_MAX_VELOCITY;
  }

  @Override
  public Distance getDriveToPoseDriveTolerance() {
    return DRIVE_TO_POSE_DRIVE_TOLERANCE;
  }

  @Override
  public Angle getDriveToPoseThetaTolerance() {
    return DRIVE_TO_POSE_THETA_TOLERANCE;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return PNEUMATICS_HUB_ID;
  }

  @Override
  public int getFlowSensorChannel() {
    return FLOW_SENSOR_CHANNEL;
  }

  @Override
  public int getRevHighPressureSensorChannel() {
    return REV_HIGH_PRESSURE_SENSOR_CHANNEL;
  }

  @Override
  public int getRevLowPressureSensorChannel() {
    return REV_LOW_PRESSURE_SENSOR_CHANNEL;
  }

  @Override
  public LinearVelocity getMoveToPathFinalVelocity() {
    return SQUARING_SPEED;
  }

  @Override
  public double getDriveFacingAngleThetaKP() {
    return DRIVE_FACING_ANGLE_KP;
  }

  @Override
  public double getDriveFacingAngleThetaKI() {
    return DRIVE_FACING_ANGLE_KI;
  }

  @Override
  public double getDriveFacingAngleThetaKD() {
    return DRIVE_FACING_ANGLE_KD;
  }

  @Override
  public double getOdometryUpdateFrequency() {
    // : return 250 Hz if using the DrivetrainIOCTRE class
    return 250.0;
  }

  @Override
  public LED_HARDWARE getLEDHardware() {
    return LED_HARDWARE.RIO;
  }

  @Override
  public int getLEDCount() {
    return LED_COUNT;
  }

  @Override
  public SWERVE_CONTROL_MODE getSwerveSteerControlMode() {
    return SWERVE_CONTROL_MODE.VOLTAGE;
  }

  @Override
  public SWERVE_CONTROL_MODE getSwerveDriveControlMode() {
    return SWERVE_CONTROL_MODE.VOLTAGE;
  }

  @Override
  public double getAzimuthSteerCouplingRatio() {
    return COUPLE_RATIO;
  }
}
