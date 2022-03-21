// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunIntake extends CommandBase {
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;

  private double runtime;
  private double initialTime = 0;
  /** Creates a new Intake. */
  public RunIntake(IntakeSubsystem m_intake, ShooterSubsystem m_shooter, double runtime) {
    this.m_intake = m_intake;
    this.m_shooter = m_shooter;
    this.runtime = runtime;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setSpeed(0);
    m_intake.setIntake(0);
    initialTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntake(0.6);
    m_shooter.setSpeed(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntake(0);
    m_shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - initialTime > runtime;
  }
}
