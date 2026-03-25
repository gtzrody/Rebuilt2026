package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
 
import java.util.function.DoubleSupplier;
 
import com.ctre.phoenix6.swerve.SwerveRequest;
 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
 

public class FeedCommand extends SequentialCommandGroup {
 

    private static final double PASS_RPM         = 2500.0;
 
    private static final double PASS_HOOD_ANGLE  = -15.0; // degrees
 
    private static final double RPM_TOLERANCE    = 100.0;
 
    private static final double HEADING_TOLERANCE = 0.05; // radians (~3 degrees)
 
    private static final double BLUE_PASS_HEADING_RAD = Math.PI;
 
    private static final double RED_PASS_HEADING_RAD  = 0.0;
 
    private final SwerveRequest.FieldCentric     m_fieldCentric = new SwerveRequest.FieldCentric();
    private final SwerveRequest.SwerveDriveBrake m_brake        = new SwerveRequest.SwerveDriveBrake();
 
    public FeedCommand(
            CommandSwerveDrivetrain drivetrain,
            Shooter                 shooter,
            Hood                    hood 
            ) {
 
        addRequirements(drivetrain, shooter, hood);

        PIDController headingPID = new PIDController(
            frc.robot.constants.Constants.SwerveConstants.HUB_ROTATION_PID_KP,
            frc.robot.constants.Constants.SwerveConstants.HUB_ROTATION_PID_KI,
            frc.robot.constants.Constants.SwerveConstants.HUB_ROTATION_PID_KD
        );
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(HEADING_TOLERANCE);
 
        DoubleSupplier passHeadingSupplier = () ->
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? RED_PASS_HEADING_RAD
                : BLUE_PASS_HEADING_RAD;
 
        DoubleSupplier rotationRateSupplier = () -> {
            headingPID.setSetpoint(passHeadingSupplier.getAsDouble());
            return headingPID.calculate(drivetrain.getState().Pose.getRotation().getRadians());
        };
 
        addCommands(
 
          
            new ParallelCommandGroup(
 
                new FunctionalCommand(
                    () -> headingPID.reset(),
                    () -> drivetrain.setControl(
                        m_fieldCentric
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(rotationRateSupplier.getAsDouble())
                    ),
                    interrupted -> {},
                    () -> headingPID.atSetpoint(),
                    drivetrain
                ),
 
                Commands.run(() -> shooter.setVelocitySetpoint(RPM.of(PASS_RPM)), shooter)
                    .until(() -> Math.abs(shooter.getVelocity().in(RPM) - PASS_RPM) < RPM_TOLERANCE)
            ),
 
            new ParallelCommandGroup(
 
                drivetrain.applyRequest(() -> m_brake),
 
                shooter.setVelocity(RPM.of(PASS_RPM)),
 
                Commands.run(
                    () -> hood.setAngleSetpoint(Degrees.of(PASS_HOOD_ANGLE)),
                    hood
                )
            )
        );
    }
}

