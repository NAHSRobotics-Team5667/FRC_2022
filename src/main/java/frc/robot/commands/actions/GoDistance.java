// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class GoDistance extends CommandBase {
  private Drivetrain m_drive;
  private double distance;

  private PIDController leftController, rightController;
  /** Creates a new GoDistance. */
  public GoDistance(Drivetrain m_drive, double distance) {
    this.m_drive = m_drive;
    this.distance = distance;

    leftController = new PIDController(0.7, 0, 0);
    rightController = new PIDController(0.7, 0, 0);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setDrivetrainSpeed(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double leftOutput = MathUtil.clamp(leftController.calculate(m_drive.getLeftDistance(), distance), -0.5, 0.5);
    // double rightOutput = MathUtil.clamp(rightController.calculate(m_drive.getRightDistance(), distance), -0.5, 0.5);

    // m_drive.setDrivetrainSpeed(leftOutput, rightOutput);
    if (distance > 0) {
      m_drive.setDrivetrainSpeed(0.3, 0.3);
    } else if (distance < 0) {
      m_drive.setDrivetrainSpeed(-0.3, -0.3);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setDrivetrainSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getStraightDistance()) >= Math.abs(distance);
  }
}
