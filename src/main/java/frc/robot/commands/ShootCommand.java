package frc.robot.commands;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HubAlignmentPID;
import frc.robot.subsystems.autoalignhood.Shootercalculations;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;


public class ShootCommand extends SequentialCommandGroup {

private static final double SHOOTER_RPM = 3000.0;

private static final double RPM_TOLERANCE = 100.0;

private final SwerveRequest.FieldCentric m_fieldCentric  = new SwerveRequest.FieldCentric();
private final SwerveRequest.SwerveDriveBrake m_brake     = new SwerveRequest.SwerveDriveBrake();

public ShootCommand(
        CommandSwerveDrivetrain drivetrain,
        HubAlignmentPID         hubPID,
        Shooter                 shooter,
        Hood                    hood,
        Shootercalculations     shooterCalc) {


    double distance        = drivetrain.getDistanceToHub();
    double targetHoodAngle = shooterCalc.getHoodAngle(distance);

    addCommands(


        new ParallelCommandGroup(

            new FunctionalCommand(
                () -> {},   
                () -> drivetrain.setControl(
                    m_fieldCentric
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(hubPID.getPIDCalculation())
                ),
                interrupted -> {}, // end
                () -> hubPID.getController().atSetpoint(),
                drivetrain
            ),

            Commands.run(() -> shooter.setVelocitySetpoint(RPM.of(SHOOTER_RPM)), shooter)
                .until(() -> Math.abs(shooter.getVelocity().in(RPM) - SHOOTER_RPM) < RPM_TOLERANCE)
        ),


        new ParallelCommandGroup(

            drivetrain.applyRequest(() -> m_brake),

            shooter.setVelocity(RPM.of(SHOOTER_RPM)),

            hood.setAngle(Degrees.of(targetHoodAngle))
        )
    );
}
}