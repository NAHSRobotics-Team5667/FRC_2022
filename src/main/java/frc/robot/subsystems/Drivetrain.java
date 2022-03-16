// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX m_fL, m_fR, m_rL, m_rR;
  private AHRS m_gyro;

  private double[] odometryValues = new double[4];

  private DifferentialDrive m_drive;
  private DifferentialDriveOdometry m_odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_fL = new WPI_TalonFX(Constants.DriveConstants.kFrontLeftId);
    m_fR = new WPI_TalonFX(Constants.DriveConstants.kFrontRightId);
    m_rL = new WPI_TalonFX(Constants.DriveConstants.kBackLeftId);
    m_rR = new WPI_TalonFX(Constants.DriveConstants.kBackRightId);

    m_fL.setNeutralMode(NeutralMode.Brake);
    m_fR.setNeutralMode(NeutralMode.Brake);
    m_rL.setNeutralMode(NeutralMode.Brake);
    m_rR.setNeutralMode(NeutralMode.Brake);

    m_fR.setInverted(true);
    m_rR.setInverted(true);

    m_rL.follow(m_fL);
    m_rR.follow(m_fR);

    m_gyro = new AHRS(SPI.Port.kMXP);

    m_drive = new DifferentialDrive(m_fL, m_fR);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
  }

  /**
   * Gets the angle of the drivetrain in degrees.
   * 
   * @return angle of the robot in degrees.
   */
  public double getAngle() {
    return -m_gyro.getAngle();
  }

  public void resetGyro() {
    m_gyro.reset();
  }
  
  public void tankDriveVolts(double right, double left) {
    m_fL.setVoltage(left);
    m_rL.setVoltage(left);
    m_fR.setVoltage(-right);
    m_fR.setVoltage(-right);
  }

  /**
   * Updates encoder values.
   * 
   * 0 - front left encoder
   * 1 - front right encoder
   * 2 - rear left encoder
   * 3 - rear right encoder
   */
  private void updateEncoderValues() {
    odometryValues[0] = m_fL.getSelectedSensorPosition();
    odometryValues[1] = -m_fR.getSelectedSensorPosition();
    odometryValues[2] = m_rL.getSelectedSensorPosition();
    odometryValues[3] = -m_rR.getSelectedSensorPosition();
  }

  /**
   * Makes the robot go weeeeeeeeeee
   * 
   * @param xSpeed    forward speed
   * @param zRotation rotation
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    m_drive.arcadeDrive(xSpeed, zRotation);
  }

  public void stopMotors() {
    m_fL.set(0);
    m_fR.set(0);
  }

  /**
   * Converts falcon drivetrain ticks to meters driven.
   * 
   * @param ticks input raw encoder units from Falcon 500
   * @return ticks converted to meters
   */
  private double falconTicksToMeters(double ticks) {
    return (ticks / (2048 * Constants.DriveConstants.kGearRatio)) * Constants.DriveConstants.kWheelDiameter * Math.PI;
  }

  /**
   * Gets the pose of the robot in meters.
   * 
   * @return the position of the robot in meters
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, pose.getRotation());
  }

  public void resetEncoders() {
    m_fL.setSelectedSensorPosition(0);
    m_fR.setSelectedSensorPosition(0);
    m_rL.setSelectedSensorPosition(0);
    m_rR.setSelectedSensorPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      falconTicksToMeters((m_fL.getSelectedSensorVelocity() + m_rL.getSelectedSensorVelocity()) / 2),
      falconTicksToMeters((m_fR.getSelectedSensorVelocity() + m_rR.getSelectedSensorVelocity()) / 2)
    );
  }

  @Override
  public void periodic() {
    updateEncoderValues();
    m_odometry.update(
      Rotation2d.fromDegrees(getAngle()), 
      falconTicksToMeters((odometryValues[0] + odometryValues[2]) / 2), 
      falconTicksToMeters((odometryValues[1] + odometryValues[3]) / 2)
    );

    m_rR.set(ControlMode.PercentOutput, m_fR.get());
    m_rL.set(ControlMode.PercentOutput, m_fL.get());

    SmartDashboard.putNumber("FL Speed", m_fL.get());
    SmartDashboard.putNumber("FR Speed", m_fR.get());
    SmartDashboard.putNumber("BL Speed", m_rL.get());
    SmartDashboard.putNumber("BR Speed", m_rR.get());
  }
}
