// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDriveSub extends SubsystemBase {
  public SparkMax frontLeft =
      new SparkMax(Constants.ConfigurationConstants.FRONT_LEFT_ID, MotorType.kBrushless);
      
  public SparkMax frontRight =
      new SparkMax(Constants.ConfigurationConstants.FRONT_RIGHT_ID, MotorType.kBrushless);

  public SparkMax rearLeft =
      new SparkMax(Constants.ConfigurationConstants.REAR_LEFT_ID, MotorType.kBrushless);

  public SparkMax rearRight =
    new SparkMax(Constants.ConfigurationConstants.REAR_RIGHT_ID, MotorType.kBrushless);

  // Gyro
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  // Kinematics
  private final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          Constants.DriveConstants.FRONT_LEFT_LOCATION,
          Constants.DriveConstants.FRONT_RIGHT_LOCATION,
          Constants.DriveConstants.BACK_LEFT_LOCATION,
          Constants.DriveConstants.BACK_RIGHT_LOCATION
      );

  // Slew rate limiters (imitan swerve)
  private final SlewRateLimiter xLimiter =
      new SlewRateLimiter(Constants.DriveConstants.X_SLEW_LIMITER);
  private final SlewRateLimiter yLimiter =
      new SlewRateLimiter(Constants.DriveConstants.Y_SLEW_LIMITER);
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(Constants.DriveConstants.ROTATION_SLEW_LIMITER);

  private final SparkMaxConfig configInverted = new SparkMaxConfig();
  private final SparkMaxConfig configNotInverted = new SparkMaxConfig();

  public MecanumDriveSub() {
    configInverted.inverted(true);
    configInverted.idleMode(IdleMode.kBrake);

    configNotInverted.inverted(false);
    configNotInverted.idleMode(IdleMode.kBrake);


    //TODO: Search for a different way of reseting parameters and persisting parameters that is not deprecated
    frontLeft.configure(configNotInverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearLeft.configure(configNotInverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    frontRight.configure(configInverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rearRight.configure(configInverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    gyro.reset();
  }

  

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public void drive(double xInput, double yInput, double zRotationInput, boolean fieldOrientedFunction) {

    // Deadband (Makes nothing moves unless is a reasonable joystick movement)
    xInput = MathUtil.applyDeadband(xInput, Constants.OIConstants.DEADBAND);
    yInput = MathUtil.applyDeadband(yInput, Constants.OIConstants.DEADBAND);
    zRotationInput = MathUtil.applyDeadband(zRotationInput, Constants.OIConstants.DEADBAND);

    // Exponential curves (Makes a smoother movement)
    xInput = Math.copySign(xInput * xInput, xInput);
    yInput = Math.copySign(yInput * yInput, yInput);
    zRotationInput = Math.copySign(zRotationInput * zRotationInput, zRotationInput);

    // Escalated to real Velocities 
    double xSpeed = xLimiter.calculate(
        xInput * Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    double ySpeed = yLimiter.calculate(
        yInput * Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND *
        Constants.DriveConstants.LATERAL_FACTOR_REDUCTION
      );

    double zRotationSpeed = rotationLimiter.calculate(
        zRotationInput * Constants.DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND);

    // Smoother Adjustment of Heading
    double translationalMag = Math.hypot(xSpeed, ySpeed);
    zRotationSpeed *= (1.0 - 0.3 * (translationalMag / Constants.DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND));

    // Field-Oriented Drive
    ChassisSpeeds speeds = fieldOrientedFunction ? ChassisSpeeds.fromRobotRelativeSpeeds(
      xSpeed,
      ySpeed,
      zRotationSpeed,
      getHeading() 
    ) : ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed,
      ySpeed, 
      zRotationSpeed, 
      getHeading()
    );

    // Using Mecanum Kinematics so it can calculate each wheels Speed
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    // Motor Outputs
    frontLeft.set(wheelSpeeds.frontLeftMetersPerSecond / Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontRight.set(wheelSpeeds.frontRightMetersPerSecond / Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    rearLeft.set(wheelSpeeds.rearLeftMetersPerSecond / Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    rearRight.set(wheelSpeeds.rearRightMetersPerSecond / Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND);
  }

  public void brake(){
    frontLeft.set(0);
    frontRight.set(0);
    rearLeft.set(0);
    rearRight.set(0);
  }
}
