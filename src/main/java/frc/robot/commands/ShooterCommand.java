// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem m_shooter) {
    this.m_shooter = m_shooter;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setSpeed(0);
    m_shooter.setHoodSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.getController().getRightBumperPressed()) {
      m_shooter.toggleShooter();
    }

    if (m_shooter.isEnabled()) {
      m_shooter.setSpeed(0.8);
      // m_shooter.setHoodAngle(m_shooter.limelightToAngle());
    } else {
      m_shooter.setSpeed(0);
    }

    if (RobotContainer.getController().getLeftTrigger() > 0 || RobotContainer.getController().getLeftBumper()) {
      m_shooter.setSpeed(-0.2);
    }

    if (RobotContainer.getController().getDPad() == 90) {
      m_shooter.setHoodSpeed(0.27); // go up
    } else if (RobotContainer.getController().getDPad() == 270) {
      m_shooter.setHoodSpeed(-0.27); // go down
    } else {
      m_shooter.setHoodSpeed(0);
    }

    SmartDashboard.putBoolean("Shooter Enabled", m_shooter.isEnabled());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setSpeed(0);
    m_shooter.setHoodSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
