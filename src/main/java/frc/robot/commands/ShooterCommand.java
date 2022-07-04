// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;

public class ShooterCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  private double shooterSpeed = 0;

  private boolean lowerHubMode = false;

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

    if (RobotContainer.getController().getRightStickButtonPressed()) {
      lowerHubMode = !lowerHubMode;
    }

    if (m_shooter.isEnabled()) {
      m_shooter.setNeutralMode(NeutralMode.Coast);
      // m_shooter.setSpeed(shooterSpeed);
      // m_shooter.setShooterSpeedPolynomial(Limelight.getInstance().getYAngle());
      if (!lowerHubMode) {
        m_shooter.setSpeed(0.6); // TODO: CHANGE THIS VALUE DEPENDING ON BENJAMIN'S RESULTS
      } else {
        m_shooter.setSpeed(0.45);
      }
      // m_shooter.setHoodAngle(m_shooter.limelightToAngle());
    } else if (RobotContainer.getBeastMode()) {
      m_shooter.setNeutralMode(NeutralMode.Brake);
      if (RobotContainer.getController().getRightTrigger() > 0) {
        m_shooter.setSpeed(0.2);
      } else {
        m_shooter.setSpeed(0);
      }
    } else {
      m_shooter.setNeutralMode(NeutralMode.Coast);
      m_shooter.setSpeed(0);
    }

    // if (RobotContainer.getController().getLeftTrigger() > 0 || RobotContainer.getController().getLeftBumper()) {
    //   m_shooter.setSpeed(-0.2);
    // }

    // if (RobotContainer.getController().getDPad() == 90) {
    //   shooterSpeed += 0.01;
    // } else if (RobotContainer.getController().getDPad() == 270) {
    //   shooterSpeed -= 0.01;
    // }

    if (RobotContainer.getController().getXButton()) {
      m_shooter.resetHoodEncoder();
    }

    SmartDashboard.putBoolean("Shooter Enabled", m_shooter.isEnabled());
    SmartDashboard.putBoolean("Lower Hub", lowerHubMode);
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
