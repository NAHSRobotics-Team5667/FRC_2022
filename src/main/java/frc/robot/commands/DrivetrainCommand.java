// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainCommand extends CommandBase {
  private Drivetrain m_drive;
  public boolean slowmode = false;
  
  /** Creates a new DrivetrainCommand. */
  public DrivetrainCommand(Drivetrain m_drive) {
    this.m_drive = m_drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_drive.resetGyro();
    m_drive.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Map<String, Double> sticks = RobotContainer.getController().getSticks();
    if (slowmode) {
      m_drive.arcadeDrive(0.5 * sticks.get("LSY"), 0.5 * -sticks.get("RSX"));
    } else {
      m_drive.arcadeDrive(0.8 * sticks.get("LSY"), 0.5 * -sticks.get("RSX"));
    }

    if (RobotContainer.getController().getLeftStickButtonPressed()) slowmode = !slowmode;

    if (RobotContainer.getController().getAButtonPressed()) {
      m_drive.resetEncoders();
    } else if (RobotContainer.getController().getBButtonPressed()) {
      m_drive.resetGyro();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
