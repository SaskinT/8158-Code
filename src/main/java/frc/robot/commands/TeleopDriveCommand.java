package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;



public class TeleopDriveCommand extends Command {

    private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Align PID — yaw'ı sıfırlamaya çalışır
    private static final double ALIGN_KP = 0.05;
    private static final double ALIGN_TOLERANCE_DEG = 2.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final CommandXboxController joystick;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final PIDController alignPID = new PIDController(ALIGN_KP, 0, 0.001);

    public TeleopDriveCommand(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        CommandXboxController joystick
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.joystick = joystick;

        alignPID.setTolerance(ALIGN_TOLERANCE_DEG);
        alignPID.setSetpoint(0.0); // Hedef: yaw = 0 (tag tam önde)

        addRequirements(drivetrain);
        // VisionSubsystem'i sadece okuyoruz, addRequirements'a eklemiyoruz
    }
    
    @Override
    public void execute() {
    double velocityY = -joystick.getLeftX() * MaxSpeed;
    double velocityX = -joystick.getLeftY() * MaxSpeed;
    double rotationalRate;
    
            if (vision.isTargetVisible()) {
                rotationalRate = alignPID.calculate(vision.getTargetYaw());
                rotationalRate = MathUtil.clamp(rotationalRate, -MaxAngularRate, MaxAngularRate);
            } else {
        // Diğer her durumda normal joystick
        rotationalRate = -joystick.getRightX() * MaxAngularRate;
    }
    
    drivetrain.setControl(
        drive
        .withVelocityX(velocityX)
        .withVelocityY(velocityY)
        .withRotationalRate(rotationalRate)
        );
    }
    
    @Override
    public boolean isFinished() {
        return false; // Teleop boyunca sürekli çalışır
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}