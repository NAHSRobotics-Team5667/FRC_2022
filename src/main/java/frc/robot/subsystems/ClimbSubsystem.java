// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  private WPI_TalonFX m_leftVertical, m_rightVertical, m_leftDiagonal, m_rightDiagonal;
  private Solenoid m_hook, m_extend;

  private PIDController m_rightVerticalController, m_leftVerticalController, m_rightDiagonalController, m_leftDiagonalController;

  private double[] verticalPositions = new double[1];
  private double[] diagonalPositions = new double [1];
  
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    m_leftVertical = new WPI_TalonFX(Constants.ClimbConstants.kLeftVerticalId);
    m_rightVertical = new WPI_TalonFX(Constants.ClimbConstants.kRightVerticalId);
    m_leftDiagonal = new WPI_TalonFX(Constants.ClimbConstants.kLeftDiagonalId);
    m_rightDiagonal = new WPI_TalonFX(Constants.ClimbConstants.kRightDiagonalId);

    m_leftVertical.setNeutralMode(NeutralMode.Brake);
    m_rightVertical.setNeutralMode(NeutralMode.Brake);
    m_leftDiagonal.setNeutralMode(NeutralMode.Brake);
    m_rightDiagonal.setNeutralMode(NeutralMode.Brake);

    m_leftVerticalController = new PIDController(ClimbConstants.kPVertical, ClimbConstants.kIVertical, ClimbConstants.kDVertical);
    m_rightVerticalController = new PIDController(ClimbConstants.kPVertical, ClimbConstants.kIVertical, ClimbConstants.kDVertical);
    m_leftDiagonalController = new PIDController(ClimbConstants.kPDiagonal, ClimbConstants.kIDiagonal, ClimbConstants.kDDiagonal);
    m_rightDiagonalController = new PIDController(ClimbConstants.kPDiagonal, ClimbConstants.kIDiagonal, ClimbConstants.kDDiagonal);
  }

  private double ticksToInches(double ticks) {
    return -1; // TODO
  }

  private double inchesToTicks(double inches) {
    return -1; // TODO
  }

  /**
   * Vertical extension in inches.
   * 
   * @return the height of the top of the vertical climb.
   */
  public double getVerticalPositionInches() {
    return ticksToInches((verticalPositions[0] + verticalPositions[1]) / 2);
  }

  /**
   * Diagonal extension in inches.
   * 
   * @return the extension of the diagonal climb in inches.
   */
  public double getDiagonalPositionInches() {
    return ticksToInches((diagonalPositions[0] + diagonalPositions[1]) / 2);
  }

  /**
   * Sets vertical position. Should be called periodically.
   * 
   * @param inches target inches of extension.
   */
  public void setVerticalPosition(double inches) {
    double target = inchesToTicks(inches);
    if (!m_leftVerticalController.atSetpoint() && !m_rightVerticalController.atSetpoint()) {
      m_leftVertical.set(ControlMode.PercentOutput, m_leftVerticalController.calculate(verticalPositions[0], target));
      m_rightVertical.set(ControlMode.PercentOutput, m_rightVerticalController.calculate(verticalPositions[1], target));
    }
  }

  /**
   * Sets diagonal position. Should be called periodically.
   * 
   * @param inches target inches of extension.
   */
  public void setVDiagonalPosition(double inches) {
    double target = inchesToTicks(inches);
    if (!m_leftDiagonalController.atSetpoint() && !m_rightDiagonalController.atSetpoint()) {
      m_leftDiagonal.set(ControlMode.PercentOutput, m_leftDiagonalController.calculate(diagonalPositions[0], target));
      m_rightDiagonal.set(ControlMode.PercentOutput, m_rightDiagonalController.calculate(diagonalPositions[1], target));
    }
  }

  @Override
  public void periodic() {
    verticalPositions[0] = m_leftVertical.getSelectedSensorPosition();
    verticalPositions[1] = -m_rightVertical.getSelectedSensorPosition();
    diagonalPositions[0] = m_leftDiagonal.getSelectedSensorPosition();
    diagonalPositions[1] = -m_rightDiagonal.getSelectedSensorPosition();
  }
}
