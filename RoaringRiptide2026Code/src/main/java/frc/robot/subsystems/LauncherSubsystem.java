// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants.LauncherSubsystemConstants.FeederSetpoints;
import frc.robot.Constants.LauncherSubsystemConstants.FlywheelSetpoints;
import frc.robot.Constants.LauncherSubsystemConstants;

public class LauncherSubsystem extends SubsystemBase {
  
  // Initialize flywheel SPARKs. We will use MAXMotion velocity control for the flywheel, so we also need to
  // initialize the closed loop controllers and encoders.
  private SparkFlex flywheelMotor =
      new SparkFlex(LauncherSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController flywheelController = flywheelMotor.getClosedLoopController();
  private RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

// Initialize feeder SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.

  private SparkFlex feederMotor =
      new SparkFlex(LauncherSubsystemConstants.kFeederMotorCanId, MotorType.kBrushless);
      
  private SparkFlex flywheelFollowerMotor =
      new SparkFlex(LauncherSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);
  private RelativeEncoder feederEncoder = feederMotor.getEncoder();

  

  // Member variables for subsystem state management
  private double flywheelTargetVelocity = 0.0;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    flywheelMotor.configure(
        Configs.LauncherSubsystem.flywheelConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    flywheelFollowerMotor.configure(
        Configs.LauncherSubsystem.flywheelFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    feederMotor.configure(
        Configs.LauncherSubsystem.feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero flywheel encoder on initialization
    flywheelEncoder.setPosition(0);

    System.out.println("---> LauncherSubsystem initialized");
  }

  private boolean isFlywheelAt(double velocity) {
    return MathUtil.isNear(flywheelEncoder.getVelocity(), 
            velocity, FlywheelSetpoints.kVelocityTolerance);
  }

  // Trigger: Is the flywheel spinning at the required velocity?

  public final Trigger isFlywheelSpinningBackwards = new Trigger(
      () -> isFlywheelAt(-FlywheelSetpoints.kLaunchRpm)
  );

  public final Trigger isFlywheelSpinning = new Trigger
  (
    () -> isFlywheelAt(FlywheelSetpoints.kLaunchRpm)
  );
  /** 
   * Trigger: Is the flywheel stopped?
   */
  public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

  /**
   * Drive the flywheels to their set velocity. This will use MAXMotion
   * velocity control which will allow for a smooth acceleration and deceleration to the mechanism's
   * setpoint.
   */
  private void setFlywheelVelocity(double velocity) {
    flywheelController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    flywheelTargetVelocity = velocity;
  }

  /** Set the feeder motor power in the range of [-1, 1]. */
  private void setFeederPower(double power) {
    feederMotor.set(power);
  }
  
  /**
   * Command to run the flywheel motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runFlywheelCommand() {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kLaunchRpm);
        },
        () -> {
          this.setFlywheelVelocity(0.0);
        }).withName("Spinning Up Flywheel");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runFeederCommand() {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kLaunchRpm);
          this.setFeederPower(FeederSetpoints.kFeed);
        }, () -> {
          this.setFlywheelVelocity(0.0);
          this.setFeederPower(0.0);
        }).withName("Feeding");
  }

  /**
   * Meta-command to operate the Launcher. The Flywheel starts spinning up and when it reaches
   * the desired speed it starts the Feeder.
   */
  public Command runLauncherCommand() {
    return this.startEnd(
      () -> this.setFlywheelVelocity(FlywheelSetpoints.kLaunchRpm),
      () -> flywheelMotor.stopMotor()).until(isFlywheelSpinning).andThen(
      this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kLaunchRpm);
          this.setFeederPower(FeederSetpoints.kFeed);
        }, () -> {
          flywheelMotor.stopMotor();
          feederMotor.stopMotor();
        })
    ).withName("Launching");
  }

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Launcher | Feeder | Applied Output", feederMotor.getAppliedOutput());
    SmartDashboard.putNumber("Launcher | Flywheel | Applied Output", flywheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Launcher | Flywheel | Current", flywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Launcher | Flywheel Follower | Applied Output", flywheelFollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Launcher | Flywheel Follower | Current", flywheelFollowerMotor.getOutputCurrent());

    SmartDashboard.putNumber("Launcher | Flywheel | Target Velocity", flywheelTargetVelocity);
    SmartDashboard.putNumber("Launcher | Flywheel | Actual Velocity", flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Feeder | Flywheel | Actual Velocity", feederEncoder.getVelocity());

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
  }

}
