// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonFX m_intake;
  private Solenoid m_piston;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    //Intake Motor
    m_intake = new WPI_TalonFX(Constants.IntakeConstants.kIntakeId);
    m_intake.setNeutralMode(NeutralMode.Coast);
    // m_piston = new Solenoid(PneumaticsModuleType.REVPH, 7);
    //Intake Piston 
    //Is not correct, will fix later ; look at FRC 2021 for piston controls
    // m_piston = new Solenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.kPistonId);
  }
  public void setIntake(double percentOutput){
    m_intake.set(ControlMode.PercentOutput, percentOutput);
  }
 public void setPiston(boolean piston){
   m_piston.set(piston);
 }
  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Piston", m_piston.get());
  }
}
