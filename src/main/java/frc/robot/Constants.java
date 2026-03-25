package frc.robot;

public final class Constants {
    public static final class FuelConstants {
        public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 12;
        public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 2;
        public static final int INDEXER_MOTOR_ID = 13;
        public static final int INDEXER_MOTOR_CURRENT_LIMIT = 80;
        public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 80;
        public static final double INDEXER_INTAKING_PERCENT = -0.8;
        public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
        public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;
        public static final double INTAKE_INTAKING_PERCENT = 0.6;
        public static final double LAUNCHING_LAUNCHER_PERCENT = 0.85;
        public static final double INTAKE_EJECT_PERCENT = -0.8;
        public static final double SPIN_UP_SECONDS = 0.75;
    }

    public static final class ClimbConstants {
        public static final int CLIMBER_MOTOR_ID = 3;
        public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;
        public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.8;
        public static final double CLIMBER_MOTOR_UP_PERCENT = 0.8;
    }

    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}