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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.Controller;

public class ShooterSubsystem extends SubsystemBase {
  public WPI_TalonFX shootingMotor;
  public WPI_TalonFX hoodMotor;
  public boolean shooterToggle = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shootingMotor = new WPI_TalonFX(Constants.ShooterConstants.kShooterMotor);
    hoodMotor = new WPI_TalonFX(Constants.ShooterConstants.kHoodMotor);
  }
  public void shootingSpeed(double speed){
    shootingMotor.set(ControlMode.PercentOutput, speed);
  }
  public double hoodAngle(){
    double angle = hoodMotor.getSelectedSensorPosition(Constants.ShooterConstants.kHoodMotor*Constants.ShooterConstants.kHoodMotor*360/2048);
    return angle;
  }    




  public int rightPressed(int button){
    if (RobotContainer.getController().getRightBumperPressed()){
      button ++;
    }

    return button;
  }


  
  public void shooterProtocol(boolean active){
    //put limelight math here
  }

  @Override
  public void periodic() {
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
        
    // This method will be called once per scheduler run
  }
}
