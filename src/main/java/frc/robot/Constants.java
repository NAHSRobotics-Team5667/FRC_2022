// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public final static int kFrontLeftId = -1; // TODO: placeholder
        public final static int kFrontRightId = -1; // TODO: placeholder
        public final static int kBackLeftId = -1; // TODO: placeholder
        public final static int kBackRightId = -1; // TODO: placeholder
        
        public final static double kGearRatio = 8 / 62;
        public final static double kWheelDiameter = Units.inchesToMeters(4);
    }

    public static final class IndexConstants {
        public final static int kSpinnyMotorId = 2;
        public final static double kGearRatio = -1; // TODO: placeholder
        public final static int kSolenoidOfIntake = -1; // TODO: placeholder
        
    }

    public static final class ClimbConstants {
        public final static int kLeftVerticalId = -1; // TODO: placeholder
        public final static int kRightVerticalId = -1; // TODO: placeholder
        public final static int kLeftDiagonalId = -1; // TODO: placeholder
        public final static int kRightDiagonalId = -1; // TODO: placeholder

        public final static double kPVertical = 0.01; // TODO: tweak with experimentation
        public final static double kIVertical = 0; // TODO: tweak with experimentation
        public final static double kDVertical = 0; // TODO: tweak with experimentation

        public final static double kPDiagonal = 0.01; // TODO: tweak with experimentation
        public final static double kIDiagonal = 0; // TODO: tweak with experimentation
        public final static double kDDiagonal = 0; // TODO: tweak with experimentation
    }

    public final static class ControllerConstants {
        public static final int CONTROLLER_PORT = 0; // Controller port

        // Sticks
        public static final int S_RIGHT_X_PORT = 4; // Right stick x
        public static final int S_RIGHT_Y_PORT = 5; // Right stick y
        public static final int S_LEFT_X_PORT = 0; // Left stick x
        public static final int S_LEFT_Y_PORT = 1; // Left stick y

        public static final int S_LEFT = 9; // Left stick button
        public static final int S_RIGHT = 10; // Right stick button

        // Triggers
        public static final int TRIGGER_RIGHT_PORT = 3; // Right trigger
        public static final int TRIGGER_LEFT_PORT = 2; // Left trigger

        // Bumpers
        public static final int BUMPER_RIGHT_PORT = 6; // Right bumper
        public static final int BUMPER_LEFT_PORT = 5; // Left bumper

        // Buttons
        public static final int BUTTON_A_PORT = 1; // A Button
        public static final int BUTTON_B_PORT = 2; // B Button
        public static final int BUTTON_X_PORT = 3; // X Button
        public static final int BUTTON_Y_PORT = 4; // Y Button

        // Special buttons
        public static final int BUTTON_MENU_PORT = 8; // Menu Button
        public static final int BUTTON_START_PORT = 7; // Start button
    }
}
   
