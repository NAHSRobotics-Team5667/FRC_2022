// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexSubsystem;

public class IndexCommand extends CommandBase {
  private IndexSubsystem index;

  /** Creates a new IndexCommand. */
  public IndexCommand(IndexSubsystem index) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.index = index;
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index.setSpeed(0.1);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.controller.getRightTrigger() > 0) {
      index.setSpeed(.1);
    } else {
      index.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
