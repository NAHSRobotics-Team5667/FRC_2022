// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.Controller;

public class ShooterSubsystem extends SubsystemBase {
  public WPI_TalonFX shootingMotorL, shootingMotorR, hoodMotor;
  public boolean shooterToggle = false;

  private boolean enableShooter = false;

  private double hoodAngle = 0;
  private double initialHoodAngle;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(double initialHoodAngle) {
    shootingMotorL = new WPI_TalonFX(Constants.ShooterConstants.kShooterMotorL);
    shootingMotorR = new WPI_TalonFX(Constants.ShooterConstants.kShooterMotorR);
    hoodMotor = new WPI_TalonFX(Constants.ShooterConstants.kHoodMotor);

    shootingMotorL.setNeutralMode(NeutralMode.Coast);
    shootingMotorR.setNeutralMode(NeutralMode.Coast);
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    shootingMotorL.setInverted(true);
    hoodMotor.setInverted(true);

    this.initialHoodAngle = initialHoodAngle;
  }

  public void setSpeed(double speed){
    shootingMotorL.set(ControlMode.PercentOutput, speed);
    shootingMotorR.set(speed);
  }

  public double getHoodAngle() {
    updateHoodAngle();
    return hoodAngle;
  }

  public void updateHoodAngle() {
    hoodAngle = ((hoodMotor.getSelectedSensorPosition() * 360) / (2048 * (1 / Constants.ShooterConstants.kHoodGearRatio))) + initialHoodAngle;
  }

  public boolean isEnabled() {
    return enableShooter;
  }

  public void toggleShooter() {
    enableShooter = !enableShooter;
  }

  public double limelightToAngle() {
    return 0; // TODO: placeholder
  }
  
  public void setHoodAngle(double target) {
    if (getHoodAngle() < target + 1) {
      hoodMotor.set(0.1);
    } else if (getHoodAngle() > target - 1) {
      hoodMotor.set(-0.1);
    }
  }

  public void setHoodSpeed(double speed) {
    hoodMotor.set(speed);
  }

  @Override
  public void periodic() {
    updateHoodAngle();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putNumber("Hood Output", hoodMotor.get());
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
  }
}
