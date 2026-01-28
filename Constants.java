// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ConfigurationConstants{
        public static final int FRONT_LEFT_ID = 15;
        public static final int FRONT_RIGHT_ID = 12;
        public static final int REAR_LEFT_ID = 13;
        public static final int REAR_RIGHT_ID = 14;
    }

    public static class OperatorConstants{
        public static final XboxController driverController = new XboxController(0);
    }

    public static class DriveConstants{
        public static final double TRACK_WIDTH = 0.55;
        public static final double WHEEL_BASE = 0.55;

        // Velocidades máximas
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.0; // m/s
        public static final double MAX_ANGULAR_SPEED_RADS_PER_SECOND = Math.PI * 2; // rad/s

        // Slew rates (Limitan la aceleracion en todos los ejes)
        public static final double X_SLEW_LIMITER = 3.0;
        public static final double Y_SLEW_LIMITER = 3.0;
        public static final double ROTATION_SLEW_LIMITER = 6.0;

        // Penalización lateral (muy importante)
        public static final double LATERAL_FACTOR_REDUCTION = 0.65;

        // Posición de ruedas
        public static final Translation2d FRONT_LEFT_LOCATION =
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);

        public static final Translation2d FRONT_RIGHT_LOCATION =
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);

        public static final Translation2d BACK_LEFT_LOCATION =
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);

        public static final Translation2d BACK_RIGHT_LOCATION =
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DEADBAND = 0.05;
    }
}
