// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Limelight;

public class AlignCommand extends CommandBase {
  private Drivetrain m_drive;
  private PIDController angleController;

  public boolean alignToggle;

  /** Creates a new AlignCommand. */
  public AlignCommand(Drivetrain m_drive) {
    this.m_drive = m_drive;

    angleController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.getInstance().turnLightOn();
    m_drive.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.getController().getRightBumperPressed()) alignToggle = !alignToggle;

    if (Limelight.getInstance().hasValidTarget() && alignToggle) {
      if (Limelight.getInstance().getXAngle() != 0) {
        double output = angleController.calculate(Limelight.getInstance().getXAngle());
        output = (output > 0) ? Math.max(0.3, output) : Math.min(-0.3, output);
        m_drive.setDrivetrainSpeed(output, -output);
      }
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
