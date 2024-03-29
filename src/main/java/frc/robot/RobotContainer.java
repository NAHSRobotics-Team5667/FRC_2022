// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.PathWeaver;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.actions.AlignCommand;
import frc.robot.commands.actions.GoDistance;
import frc.robot.commands.actions.RunIntake;
import frc.robot.commands.actions.SetPiston;
import frc.robot.commands.actions.Shoot;
import frc.robot.commands.actions.Wait;
import frc.robot.commands.auto.TrajectoryFollower;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private IntakeSubsystem m_intake;
  private IndexSubsystem m_index;
  private ShooterSubsystem m_shooter;

  public static boolean beastmode = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drive = new Drivetrain();
    m_climb = new ClimbSubsystem();
    m_intake = new IntakeSubsystem();
    m_index = new IndexSubsystem();
    m_shooter = new ShooterSubsystem(0);

    m_drive.setDefaultCommand(new DrivetrainCommand(m_drive));
    m_climb.setDefaultCommand(new ClimbCommand(m_climb));
    m_intake.setDefaultCommand(new IntakeCommand(m_intake));
    m_index.setDefaultCommand(new IndexCommand(m_index));
    m_shooter.setDefaultCommand(new ShooterCommand(m_shooter));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton rightBumper = new JoystickButton(getController(), Constants.ControllerConstants.BUMPER_RIGHT_PORT);
    JoystickButton a = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_A_PORT);

    rightBumper.whenPressed(new AlignCommand(m_drive));
    a.whenPressed(() -> beastmode = !beastmode);
  }

  public static Controller getController() {
    return controller;
  }

  public static boolean getBeastMode() {
    return beastmode;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return getAuto(1);
    // return new GoDistance(m_drive, 3);
  }

  public SequentialCommandGroup getAuto(int index) {
    switch (index) {
      case 0:
        return new SequentialCommandGroup(
          new GoDistance(m_drive, -1.5),
          new AlignCommand(m_drive),
          new Shoot(m_shooter, m_intake, m_index)
        );
      case 1:
        return new SequentialCommandGroup(
          new SetPiston(m_intake, true),
          new ParallelCommandGroup(
            new RunIntake(m_intake, m_index, 5),
            new GoDistance(m_drive, -2)
          ),
          new GoDistance(m_drive, 1.5),
          new AlignCommand(m_drive),
          new Shoot(m_shooter, m_intake, m_index)
        );
      default:
        return null;
    }
  }
}
