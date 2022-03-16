package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;

public class TrajectoryFollower {
    public static Command getRamseteCommand(Trajectory path, Drivetrain m_drive) {
        RamseteCommand auto = new RamseteCommand(
            path, 
            m_drive::getPose, 
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta), 
            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics, 
            m_drive::getWheelSpeeds, 
            new PIDController(DriveConstants.kPDriveVel, 0, 0), 
            new PIDController(DriveConstants.kPDriveVel, 0, 0), 
            m_drive::tankDriveVolts, 
            m_drive);

        m_drive.resetOdometry(path.getInitialPose());

        return auto.andThen(() -> m_drive.tankDriveVolts(0,0));
    }
}