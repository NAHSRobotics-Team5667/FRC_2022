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

public class IndexSubsystem extends SubsystemBase {  
  public Solenoid s_indexPiston;
  public WPI_TalonFX m_indexMotor;

  /** Creates a new IndexSubsysytem. */
  public IndexSubsystem() {
    m_indexMotor = new WPI_TalonFX(Constants.IndexConstants.kSpinnyMotorId);

    m_indexMotor.setNeutralMode(NeutralMode.Brake);
    // m_indexMotor.setInverted(true);
  }

  public void setSpeed(double fastness) {
    m_indexMotor.set(ControlMode.PercentOutput, fastness);
  }

  @Override
  public void periodic() {
  }
}
