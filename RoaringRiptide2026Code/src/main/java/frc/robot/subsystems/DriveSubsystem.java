// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.pathplanner.lib.config.RobotConfig;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.PathPlannerConstants;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 pidgey = new Pigeon2(20, "rio");

  private final Field2d field2d = new Field2d();

  private boolean tooCloseToHub() {
    var tags = LimelightHelpers.getRawFiducials(VisionConstants.kLimelightName);

    if (tags.length == 0) {
      return false;
    }

    double distance = tags[0].distToRobot;
    return distance <= VisionConstants.kMinHubDistanceInches;
  }

  // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry =
  //   new SwerveDriveOdometry(
  //       DriveConstants.kDriveKinematics,
  //       Rotation2d.fromDegrees(pidgey.getYaw().getValueAsDouble()),
  //       new SwerveModulePosition[] {
  //         m_frontLeft.getPosition(),
  //         m_frontRight.getPosition(),
  //         m_rearLeft.getPosition(),
  //         m_rearRight.getPosition()
  //       },
  //       new Pose2d());
  private SwerveDriveOdometry m_Odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // Apply default configuration
    Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
    pidgey.getConfigurator().apply(pigeonConfig);

    // Set signal update rates (important for CAN optimization)
    pidgey.getYaw().setUpdateFrequency(100);              // 100 Hz for odometry
    pidgey.getAngularVelocityZWorld().setUpdateFrequency(100);

    // Optimize bus utilization
    pidgey.optimizeBusUtilization();

    // Zero heading at startup
    pidgey.setYaw(0);

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    // AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //         ),
    //         Constants.PathPlannerConstants.kRobotConfig, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );

m_Odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(pidgey.getYaw().getValueAsDouble()),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    },
    new Pose2d()  // IMPORTANT
);

    AutoBuilder.configure(
    this::getPose,
    this::resetOdometry,
    this::getRobotRelativeSpeeds,
    this::driveRobotRelative,
    new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0)
    ),
    PathPlannerConstants.kRobotConfig,
    () -> DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red,
    this
);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_Odometry.update(
        Rotation2d.fromDegrees(pidgey.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    
    // Adding field map to the smart dashboard
    field2d.setRobotPose(m_Odometry.getPoseMeters());
    SmartDashboard.putData(field2d);

    SmartDashboard.putNumber("FL Turning burnt", m_frontLeft.m_turningSpark.getOutputCurrent());
    SmartDashboard.putNumber("FL Turning healthy", m_frontRight.m_turningSpark.getOutputCurrent());

    SmartDashboard.putBoolean("Too Close To Hub", tooCloseToHub());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
  return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState());
}

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {

    if (pose == null) {
      System.out.println("❌ ERROR: resetOdometry received null pose");
      return;
    }

    if (m_Odometry == null) {
      System.out.println("❌ ERROR: m_odometry is NULL");
      return;
    }

    try {
      m_Odometry.resetPosition(
          Rotation2d.fromDegrees(pidgey.getYaw().getValueAsDouble()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          pose);
    } catch (Exception e) {
      System.out.println("❌ resetOdometry crashed: " + e.getMessage());
    }
  }

    public void resetPose(Pose2d pose) {
      resetOdometry(pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    if (tooCloseToHub() && xSpeed > 0) {
          xSpeed = 0;
    }
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(pidgey.getYaw().getValueAsDouble()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  }

    /**
 * Drives the robot using robot-relative chassis speeds (used by PathPlanner AutoBuilder).
 *
 * @param speeds The desired ChassisSpeeds (robot-relative).
 */
public void driveRobotRelative(ChassisSpeeds speeds) {
  var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_frontRight.setDesiredState(swerveModuleStates[1]);
  m_rearLeft.setDesiredState(swerveModuleStates[2]);
  m_rearRight.setDesiredState(swerveModuleStates[3]);
}
// public ChassisSpeeds getRobotRelativeSpeeds() {
//   return DriveConstants.kDriveKinematics.toChassisSpeeds(
//     m_frontLeft.getState(),
//     m_frontRight.getState(),
//     m_rearLeft.getState(),
//     m_rearRight.getState()
//   );
// }

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> pidgey.setYaw(0));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return pidgey.getYaw().getValueAsDouble();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return pidgey.getAngularVelocityZWorld().getValueAsDouble()
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
}
