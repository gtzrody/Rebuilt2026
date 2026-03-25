// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveTeleop;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoLineUp;
import frc.robot.commands.DriveToHub;
import frc.robot.subsystems.HubAlignmentPID;
import frc.robot.subsystems.autoalignhood.Shootercalculations;
import frc.robot.subsystems.camera.AprilTagCamera;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.auto.LeftDriveAuto;
import frc.robot.auto.RightDriveAuto;
import frc.robot.constants.Constants.CameraConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.autoalignhood.Shootercalculations;
import java.util.Set;
import frc.robot.commands.FeedCommand;


public class RobotContainer {
    
    // public SendableChooser<Command> autoSelector;

    //Constants
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    //Subsystems
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController op_joystick = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Hood m_hood = new Hood();
    private final Shooter m_Shooter = new Shooter();
    private final Intake m_intake = new Intake();
    private final Indexer m_index = new Indexer();
    private final Hopper m_hopper = new Hopper();
    private final HubAlignmentPID m_hubPID = new HubAlignmentPID(drivetrain);
    private final Shootercalculations m_shooterCalc = new Shootercalculations();
    // private final Command swerveTeleop = new SwerveTeleop(drivetrain, joystick);

    //Camera 
    public AprilTagCamera rightcamera1 = new AprilTagCamera(Constants.CameraConstants.CAMERA_1_NAME, Constants.CameraConstants.CAMERA_1_POS, drivetrain);
    public AprilTagCamera leftcamera2 = new AprilTagCamera(Constants.CameraConstants.CAMERA_2_NAME, Constants.CameraConstants.CAMERA_2_POS, drivetrain);

    //Commands
    private final Command leftHubAutoDrive = new AutoLineUp(drivetrain, 0);
    private final Command rightHubAutoDrive = new AutoLineUp(drivetrain, 1);
    

    private final Telemetry logger = new Telemetry(MaxSpeed);


     private void configureDefaultCommands() {
        // drivetrain.setDefaultCommand(swerveTeleop);
        m_intake.setDefaultCommand(m_intake.set(0));
        m_index.setDefaultCommand(m_index.set(0));
        m_hopper.setDefaultCommand(m_hopper.set(0));
        drivetrain.registerTelemetry(logger::telemeterize);
        m_Shooter.setDefaultCommand(m_Shooter.set(0));
        m_hood.setDefaultCommand(m_hood.set(0));
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }

    private void configureBindings() {
 


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //Swerve/Vision Bindings
        joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.leftBumper().whileTrue(leftHubAutoDrive);
        // joystick.rightBumper().whileTrue(rightHubAutoDrive);

        //Scoring Bindings
        joystick.leftTrigger().whileTrue(m_intake.set(0.80));
        joystick.rightBumper().whileTrue(m_hopper.set(0.60));
        joystick.rightTrigger().whileTrue(m_Shooter.setVelocity(RPM.of(1000)));
        joystick.rightBumper().whileTrue(m_index.set(0.60));
        joystick.leftBumper().whileTrue(
            Commands.defer(
                () -> new ShootCommand(
                    drivetrain,
                    m_hubPID,
                    m_Shooter,
                    m_hood,
                    m_shooterCalc
                ),
                Set.of(drivetrain, m_Shooter, m_hood)
            )
        );  

        joystick.b().whileTrue(
            new FeedCommand(drivetrain, m_Shooter, m_hood)
        );
        //Hood Bindings
        joystick.povUp().whileTrue(m_hood.set(0.05));
        joystick.povDown().whileTrue(m_hood.set(-0.05));
        // joystick.povDown().whileTrue(m_hood.setAngle(Degrees.of(10)));

        
        drivetrain.registerTelemetry(logger::telemeterize);

    }


   

     private void initializeAutoCommands() {
        // final Command LeftDisrupter = new LeftDriveAuto(drivetrain);
    //     final Command RightDisrupter = new RightDriveAuto(drivetrain);


    // autoSelector = new SendableChooser<>();
    // autoSelector.setDefaultOption("No Auto", new InstantCommand(() -> drivetrain.resetPose(drivetrain.getState().Pose), drivetrain));
    // autoSelector.addOption("LeftDisrupter", LeftDisrupter);
    // autoSelector.addOption("RightDisrupter", RightDisrupter);

    
    // autoSelector.close();
    // SmartDashboard.putData("Auto Selector", autoSelector);
    }

    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
        initializeAutoCommands();
    }

     public Command getAutonomousCommand() {
         return null;
    
}
}