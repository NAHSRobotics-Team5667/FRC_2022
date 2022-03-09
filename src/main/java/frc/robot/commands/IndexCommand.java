// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.utils.Controller;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexSubsysytem;

public class IndexCommand extends CommandBase {
  private IndexSubsysytem index;

  /** Creates a new IndexCommand. */
  public IndexCommand(IndexSubsysytem index) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.index = index;
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.controller.getRightTrigger() > 0) {
      index.indexSpeed(.31);
      index.indexGoPew(false);
    } else if (RobotContainer.controller.getLeftTrigger() > 0)
      index.indexSpeed(.31);
      index.indexGoPew(true);
    } else{
      index.indexSpeed(0);
      index.indexGoPew(true);
    }

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.indexSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
