// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private WPI_TalonFX m_forward, m_backward;
  
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    m_forward = new WPI_TalonFX(Constants.ClimbConstants.kForwardMotorId);
    m_backward = new WPI_TalonFX(Constants.ClimbConstants.kBackwardMotorId);

    m_forward.setNeutralMode(NeutralMode.Brake);
    m_backward.setNeutralMode(NeutralMode.Brake);

    m_backward.follow(m_forward);
    m_backward.setInverted(true);
  }

  /**
   * Moves the vertical component of the climb upwards
   * @param speed - The speed to set the motors to
   */
  public void moveUp(double speed) {
    m_forward.set(speed);
  }

  /**
   * Moves the vertical component of the climb downwards
   * @param speed - The speed to set the motors to
   */
  public void moveDown(double speed) {
    m_forward.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
