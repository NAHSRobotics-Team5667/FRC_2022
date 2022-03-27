// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainCommand extends CommandBase {
  private Drivetrain m_drive;
  public boolean slowmode = false;
  public boolean beastmode = false;

  private PIDController throttleController, rotationController;

  private double previousThrottle = 0;
  private double previousRotation = 0;
  
  /** Creates a new DrivetrainCommand. */
  public DrivetrainCommand(Drivetrain m_drive) {
    this.m_drive = m_drive;
    throttleController = new PIDController(0.05, 0, 0);
    rotationController = new PIDController(0.05, 0, 0);
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

    double throttle = previousThrottle + throttleController.calculate(previousThrottle, sticks.get("LSY"));
    double rotation = previousRotation + rotationController.calculate(previousRotation, -sticks.get("RSX"));

    if (slowmode) {
      m_drive.arcadeDrive(
        MathUtil.clamp(sticks.get("LSY"), -0.2, 0.2), 
        MathUtil.clamp(-sticks.get("RSX"), -0.2, 0.2));
    } else if (RobotContainer.getBeastMode()) {
      m_drive.arcadeDrive(
        MathUtil.clamp(throttle, -1, 1), 
        MathUtil.clamp(-sticks.get("RSX"), -0.4, 0.4));
    } else {
      m_drive.arcadeDrive(
        MathUtil.clamp(throttle, -0.8, 0.8), 
        MathUtil.clamp(-sticks.get("RSX"), -0.4, 0.4));
    }

    // if (slowmode) {
    //   m_drive.arcadeDrive(0.5 * sticks.get("LSY"), 0.5 * -sticks.get("RSX"));
    // } else {
    //   m_drive.arcadeDrive(0.8 * sticks.get("LSY"), 0.5 * -sticks.get("RSX"));
    // }

    if (RobotContainer.getController().getLeftStickButtonPressed()) slowmode = !slowmode;
    // if (RobotContainer.getController().getAButtonPressed()) beastmode = !beastmode;

    // if (RobotContainer.getController().getAButtonPressed()) {
    //   m_drive.resetEncoders();
    // } else if (RobotContainer.getController().getBButtonPressed()) {
    //   m_drive.resetGyro();
    // }

    previousThrottle = throttle;
    previousRotation = rotation;

    SmartDashboard.putBoolean("Slowmode", slowmode);
    // SmartDashboard.putBoolean("Beastmode", beastmode);

    m_drive.feed();
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
