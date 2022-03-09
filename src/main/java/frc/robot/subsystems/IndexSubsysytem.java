// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsysytem extends SubsystemBase {  
  public Solenoid s_indexPisston;
  public WPI_TalonFX m_indexMotor;

  /** Creates a new IndexSubsysytem. */
  public IndexSubsysytem() {
    
   
    s_indexPisston = new Solenoid(PneumaticsModuleType.REVPH ,Constants.IndexConstants.kSolenoidOfIntake);
    m_indexMotor = new WPI_TalonFX(Constants.IndexConstants.kSpinnyMotorId);
    m_indexMotor.setNeutralMode(NeutralMode.Brake);
    
  } 
    
  public void indexSpeed(double fastness){
    m_indexMotor.set(ControlMode.PercentOutput, fastness);
  }
   
  public void indexGoPew(boolean pisston) {
    s_indexPisston.set(pisston);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }
}
