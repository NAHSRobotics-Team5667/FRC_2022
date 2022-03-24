// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GoDistance extends CommandBase {
  private Drivetrain m_drive;
  private double distance;
  /** Creates a new GoDistance. */
  public GoDistance(Drivetrain m_drive, double distance) {
    this.m_drive = m_drive;
    this.distance = distance;
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
    if (distance > 0) {
      m_drive.setDrivetrainSpeed(0.4, 0.4);
    } else if (distance < 0) {
      m_drive.setDrivetrainSpeed(-0.4, -0.4);
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
