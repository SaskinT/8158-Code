// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.TeleopDriveCommand;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final FuelSubsystem fuelsubsystem = new FuelSubsystem();
    private final ClimbSubsystem climbsubsystem = new ClimbSubsystem();
    
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final CommandXboxController joystick = new CommandXboxController(DRIVER_CONTROLLER_PORT);

    private SendableChooser<Command> autoChooser;

    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    
    private double SlowSpeed = MaxSpeed * 0.35;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final VisionSubsystem camera = new VisionSubsystem(VisionConstants.camera);

    public RobotContainer() {

        // /*
        NamedCommands.registerCommand("Intake", (new Intake(fuelsubsystem)));


 /*Denencek
        NamedCommands.getCommand("Intake");
        NamedCommands.registerCommand("Intake", getAutonomousCommand());
*/
     /* Denecek
        NamedCommands.registerCommand("Intake", Commands.runOnce(()->{System.out.println("iceri alanzi");}));
        NamedCommands.registerCommand("ClimbUP", Commands.runOnce(()->{System.out.println("climbdown");}));
        NamedCommands.registerCommand("ClimbDown", Commands.runOnce(()->{System.out.println("Climbup");}));
        NamedCommands.registerCommand("LaunchSequence", Commands.runOnce(()->{System.out.println("Atis");}));
 */
       /*Denenecek
        new EventTrigger("Intake").onTrue(Commands.runOnce(()->{System.out.println("icerialanzi");}));

        */

        configureBindings();

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.setControl(
                    new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain
            );
            autoChooser = AutoBuilder.buildAutoChooser();
        } catch (Exception e) {
            e.printStackTrace();
            
            autoChooser = null;
        }

        SmartDashboard.putData("Auto Chooser", autoChooser != null ? autoChooser : new SendableChooser<>());
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        
        joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * SlowSpeed)
                .withVelocityY(-joystick.getLeftX() * SlowSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
        ));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        
        joystick.b().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.leftTrigger().onTrue(new TeleopDriveCommand(drivetrain, camera, joystick));

        drivetrain.registerTelemetry(logger::telemeterize);

        


        joystick.leftBumper().whileTrue(new Intake(fuelsubsystem));
        joystick.rightBumper().whileTrue(new LaunchSequence(fuelsubsystem));
        joystick.a().whileTrue(new Eject(fuelsubsystem));

        joystick.povDown().whileTrue(new ClimbDown(climbsubsystem));
        joystick.povUp().whileTrue(new ClimbUp(climbsubsystem));

        fuelsubsystem.setDefaultCommand(Commands.run(fuelsubsystem::stop, fuelsubsystem));
        climbsubsystem.setDefaultCommand(Commands.run(climbsubsystem::stop, climbsubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser != null ? autoChooser.getSelected() : Commands.none();
    }
}