// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.hopper;

// import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
// import static edu.wpi.first.units.Units.Feet;
// import static edu.wpi.first.units.Units.Pounds;
// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Volts;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import yams.gearing.GearBox;
// import yams.gearing.MechanismGearing;
// import yams.mechanisms.SmartMechanism;
// import yams.mechanisms.config.ArmConfig;
// import yams.mechanisms.positional.Arm;
// import yams.motorcontrollers.SmartMotorController;
// import yams.motorcontrollers.SmartMotorControllerConfig;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
// import yams.motorcontrollers.local.SparkWrapper;
// import yams.motorcontrollers.remote.TalonFXWrapper;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// public class Hopper extends SubsystemBase {

//   private SmartMotorControllerConfig HopperConfig = new SmartMotorControllerConfig(this)
//   .withControlMode(ControlMode.CLOSED_LOOP)
//   .withClosedLoopController(10, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
//   .withSimClosedLoopController(10, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
//   .withFeedforward(new ArmFeedforward(0, 0, 0))
//   .withSimFeedforward(new ArmFeedforward(0, 0, 0))
//   .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
//   // Gearing from the motor rotor to final shaft.
//   // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
//   .withGearing(new MechanismGearing(GearBox.fromReductionStages(36, 25)))
//   // Motor properties to prevent over currenting.
//   .withMotorInverted(false)
//   .withIdleMode(MotorMode.BRAKE)    
//   .withStatorCurrentLimit(Amps.of(90))
//   .withClosedLoopRampRate(Seconds.of(0.25))
//   .withOpenLoopRampRate(Seconds.of(0.25));

//   // Vendor motor controller object
//     TalonFX HopperFx = new TalonFX(16, new CANBus("canivore"));


//   // Create our SmartMotorController from our Spark and config with the NEO.
//   SmartMotorController HopperController = new TalonFXWrapper(indexFx, DCMotor.getKrakenX60(1), IndexConfig);


//   private ArmConfig HopperCfg = new ArmConfig(HopperController)
//   // Soft limit is applied to the SmartMotorControllers PID
//   .withSoftLimits(Degrees.of(-360), Degrees.of(360))
//   // Hard limit is applied to the simulation.
//   .withHardLimit(Degrees.of(-720), Degrees.of(720))
//   // Starting position is where your arm starts
//   .withStartingPosition(Degrees.of(0))
//   // Length and mass of your arm for sim.
//   .withLength(Feet.of(3))
//   .withMass(Pounds.of(1))
//   // Telemetry name and verbosity for the arm.
//   .withTelemetry("Arm", TelemetryVerbosity.HIGH);

//   // Arm Mechanism
//   private Arm hopper = new Arm(HopperCfg);


//   /**
//    * Move the arm up and down.
//    * @param cycle [-1, 1] speed to set the arm too.
//    */
//   public Command set(double cycle) { return hopper.set(cycle);}


//   /** Creates a new ExampleSubsystem. */
//   public Hopper() {}

//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     index.updateTelemetry();
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//     index.simIterate();
//   }
// }
