// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  WPI_TalonFX frontLeft;
  WPI_TalonFX backLeft;
  WPI_TalonFX frontRight;
  WPI_TalonFX backRight;

  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;

  private static final int kFrontLeftChannel = 0;
  private static final int kRearLeftChannel = 1;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 3;
  private static final int kGyroPort = 0;

  private MecanumDrive mecanumDrive;

  public AnalogGyro gyro = new AnalogGyro(kGyroPort);
  MecanumDriveOdometry odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d());

  public Drive() {

    frontLeft = new WPI_TalonFX(kFrontLeftChannel);
    backLeft = new WPI_TalonFX(kRearLeftChannel);
    frontRight = new WPI_TalonFX(kFrontRightChannel);
    backRight = new WPI_TalonFX(kRearRightChannel);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    backRight.setInverted(true);

    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    gyro.setSensitivity(kVoltsPerDegreePerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
                gyro.getRotation2d(),
                new MecanumDriveWheelSpeeds(
                    getSpeed(frontLeft),
                    getSpeed(backLeft),
                    getSpeed(frontRight),
                    getSpeed(backRight) 
                    )
         ) ;
         report();
  }

  private double getSpeed(WPI_TalonFX motor) {
    // multiplied by 10 because velocity reported as counts per 1/10th second
    double s = motor.getSelectedSensorVelocity() * 10.0 * DriveConstants.Meters_Per_Count;
    return (s);
  }

     /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getGyroHeading() {
      return Math.IEEEremainder(gyro.getAngle(), 360) ;
  }

  private void report() {
    NetworkTableInstance.getDefault().getEntry("drive/gyro_heading").setDouble(getGyroHeading());
    NetworkTableInstance.getDefault().getEntry("drive/odometry/X").setDouble(odometry.getPoseMeters().getX());
    NetworkTableInstance.getDefault().getEntry("drive/odometry/Y").setDouble(odometry.getPoseMeters().getY());
    NetworkTableInstance.getDefault().getEntry("drive/odometry/theta").setDouble(odometry.getPoseMeters().getRotation().getDegrees());
  }

  public void driveCartesian(double d, double x, double z, double angle) {
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }
}
