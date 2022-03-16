// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.PathWeaver;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.auto.TrajectoryFollower;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IndexSubsysytem;
import frc.robot.utils.Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static Controller controller = new Controller(0);
  
  private Drivetrain m_drive;
  private ClimbSubsystem m_climb;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drive = new Drivetrain();
    m_climb = new ClimbSubsystem();

    m_drive.setDefaultCommand(new DrivetrainCommand(m_drive));
    m_climb.setDefaultCommand(new ClimbCommand(m_climb));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public static Controller getController() {
    return controller;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return getAuto(0);
  }

  public SequentialCommandGroup getAuto(int index) {
    switch(index) {
      case 0:
        return new SequentialCommandGroup(
          TrajectoryFollower.getRamseteCommand(PathWeaver.getTrajectory("forward"), m_drive),
          TrajectoryFollower.getRamseteCommand(PathWeaver.getTrajectory("backward"), m_drive),
          TrajectoryFollower.getRamseteCommand(PathWeaver.getTrajectory("onward"), m_drive)
        );
      default:
        return null;
    }
  }
}
