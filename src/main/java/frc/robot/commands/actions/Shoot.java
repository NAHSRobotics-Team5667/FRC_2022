// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;

public class Shoot extends CommandBase {
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  private IndexSubsystem m_index;

  private double initialTime;
  /** Creates a new Shoot. */
  public Shoot(ShooterSubsystem m_shooter, IntakeSubsystem m_intake, IndexSubsystem m_index) {
    this.m_shooter = m_shooter;
    this.m_intake = m_intake;
    this.m_index = m_index;
    addRequirements(m_shooter, m_intake, m_index);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setSpeed(0);
    m_intake.setIntake(0);
    m_index.setSpeed(0);
    initialTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - initialTime > 2) {
      m_intake.setIntake(0.7);
      m_index.setSpeed(0.3);
    } else {
      m_shooter.setShooterSpeedLinear(Limelight.getInstance().getYAngle());
      m_index.setSpeed(-0.2);
      m_intake.setIntake(0.7);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setSpeed(0);
    m_intake.setIntake(0);
    m_index.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
