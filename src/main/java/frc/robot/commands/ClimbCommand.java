// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
  ClimbSubsystem m_climbSubsystem;
  
  /** Creates a new ClimbCommand. 
   * 
   *  @param subsystem - The subsystem to be used by this command.
  */
  public ClimbCommand(ClimbSubsystem subsystem) {
    m_climbSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSubsystem.setVerticalSpeed(0.0);
    //m_climbSubsystem.setDiagonalSpeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Moves the vertical climb depending on the dpad button pressed
    if (RobotContainer.getController().getDPad() == 0) {
      m_climbSubsystem.setVerticalSpeed(0.4);
    } else if (RobotContainer.getController().getDPad() == 180) {
      m_climbSubsystem.setVerticalSpeed(-0.4);
    } else {
      m_climbSubsystem.setVerticalSpeed(0.0);
    }

    //Moves the diagonal climb depending on the dpad button pressed
    /*if (RobotContainer.getController().getDPad() == 90) {
      //Extend
      m_climbSubsystem.setDiagonalSpeed(0.2);
    } else if (RobotContainer.getController().getDPad() == 270) {
      //Retract
      m_climbSubsystem.setDiagonalSpeed(-0.2);
    } else {
      m_climbSubsystem.setDiagonalSpeed(0.0);
    }*/

    //Releases the vertical climb when X is pressed
<<<<<<< Updated upstream
    if (RobotContainer.getController().getXButton()) {
      m_climbSubsystem.setVerticalPosition(Constants.ClimbConstants.kMaxVerticalHeight);
    }
=======
    // if (RobotContainer.getController().getXButton()) {
    //   m_climbSubsystem.setVerticalPosition(0.0);
    // }
>>>>>>> Stashed changes

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setVerticalSpeed(0.0);
    //m_climbSubsystem.setDiagonalSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
