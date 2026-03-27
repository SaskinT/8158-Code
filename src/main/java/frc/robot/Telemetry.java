package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {

    
    private final double m_maxSpeed;

    
    private final double m_maxSpeedTimesTwo;

    
    private static final int MODULE_COUNT = 4;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        m_maxSpeed = maxSpeed;
        m_maxSpeedTimesTwo = 2 * maxSpeed;
        SignalLogger.start();

        
        for (int i = 0; i < MODULE_COUNT; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    
    private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();

    
    private final NetworkTable m_driveStateTable = m_inst.getTable("DriveState");

    private final StructPublisher<Pose2d> m_drivePose = m_driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> m_driveSpeeds = m_driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> m_driveModuleStates = m_driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> m_driveModuleTargets = m_driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> m_driveModulePositions = m_driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

    private final DoublePublisher m_driveTimestamp = m_driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher m_driveOdometryFrequency = m_driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    
    private final NetworkTable m_table = m_inst.getTable("Pose");

    private final DoubleArrayPublisher m_fieldPub = m_table.getDoubleArrayTopic("robotPose").publish();
    
    private final StringPublisher m_fieldTypePub = m_table.getStringTopic(".type").publish();

    
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };


    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };

    
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    
    public void telemeterize(SwerveDriveState state) {
        
        m_drivePose.set(state.Pose);
        m_driveSpeeds.set(state.Speeds);
        m_driveModuleStates.set(state.ModuleStates);
        m_driveModuleTargets.set(state.ModuleTargets);
        m_driveModulePositions.set(state.ModulePositions);
        m_driveTimestamp.set(state.Timestamp);
        m_driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        
        m_fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        m_fieldPub.set(m_poseArray);

        
        for (int i = 0; i < MODULE_COUNT; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / m_maxSpeedTimesTwo);
        }
    }
}