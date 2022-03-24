// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem intake;
  private boolean pistonExtend = false;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // intake.setPiston(false);
    intake.setIntake(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.getController().getLeftTrigger() > 0) {
      intake.setIntake(0.7);
      // intake.setPiston(true);
    } else if (RobotContainer.getController().getRightTrigger() > 0) {
      intake.setIntake(0.7);
      // intake.setPiston(false);
    } else {
      intake.setIntake(0);
      // intake.setPiston(false);
    }

    if (RobotContainer.getController().getYButtonPressed()) {
      pistonExtend = !pistonExtend;
    }

    intake.setPiston(pistonExtend);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // intake.setPiston(false);
    intake.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
