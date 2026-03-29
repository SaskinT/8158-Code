package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;

public final class Constants {
    public static final class FuelConstants {
        public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 5;//12
        public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 15;//2
        public static final int INDEXER_MOTOR_ID = 16;//13
        public static final int INDEXER_MOTOR_CURRENT_LIMIT = 80;
        public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 80;
        public static final double INDEXER_INTAKING_PERCENT = 0.8;
        public static final double INDEXER_LAUNCHING_PERCENT = -0.6;
        public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;
        public static final double INTAKE_INTAKING_PERCENT = 0.6;
        public static final double LAUNCHING_LAUNCHER_PERCENT = 0.85;
        public static final double INTAKE_EJECT_PERCENT = -0.8;
        public static final double SPIN_UP_SECONDS = 0.75;
    }

    public static final class ClimbConstants {
        public static final int CLIMBER_MOTOR_ID = 4;//3
        public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;
        public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.8;
        public static final double CLIMBER_MOTOR_UP_PERCENT = 0.8;
    }

    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
    
    public static final class VisionConstants {
        public static final PhotonCamera camera = new PhotonCamera("OV2311");
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(-0.250, 0.225, 0.62),
            new Rotation3d(0, Math.toRadians(-28), Math.toRadians(180))
);

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}