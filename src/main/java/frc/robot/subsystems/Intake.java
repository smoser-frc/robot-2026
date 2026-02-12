package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

// initializes intake arm and roller motors
public class Intake extends SubsystemBase {
  private final TalonFX liftMotor = new TalonFX(Constants.Intake.LIFT_MOTOR_ID);
  private final TalonFX rollerMotor = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);

  private final DutyCycleOut rollerDutyCycle = new DutyCycleOut(0);

  private SmartMotorControllerConfig liftConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("LiftMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(32, SparkLowLevel.MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(1), liftConfig);

  // Create our SmartMotorController from our Spark and config with the NEO.
  // private SmartMotorController liftSmartMotorController =
  //    new TalonFXWrapper(liftMotor, DCMotor.getKrakenX60(1), liftConfig);

  private ArmConfig liftCfg =
      new ArmConfig(sparkSmartMotorController)
          // Soft limit is applied to the SmartMotorControllers PID
          .withSoftLimits(Degrees.of(-20), Degrees.of(10))
          // Hard limit is applied to the simulation.
          .withHardLimit(Degrees.of(-30), Degrees.of(40))
          // Starting position is where your arm starts
          .withStartingPosition(Degrees.of(-5))
          // Length and mass of your arm for sim.
          .withLength(Feet.of(3))
          .withMass(Pounds.of(1))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm lift = new Arm(liftCfg);

  public Intake() {}

  public Command setAngle(Double angle) {
    return lift.run(Degrees.of(angle));
  }

  public Command setAngleAndStop(Double angle) {
    return lift.runTo(Degrees.of(angle), Angle.ofBaseUnits(3, Degrees));
  }

  public void setAngleSetpoint(Double angle) {
    lift.setMechanismPositionSetpoint(Degrees.of(angle));
  }

  public Command set(double dutycycle) {
    return lift.set(dutycycle);
  }

  public Command sysId() {
    return lift.sysId(Volts.of(7), Volts.of(2).per(Seconds), Seconds.of(4));
  }

  public void setArmSpeed(double speed) {}

  public void setRollerSpeed(double speed) {
    this.rollerMotor.setControl(rollerDutyCycle.withOutput(speed));
  }

  private void runRoller() {
    setRollerSpeed(Constants.Intake.ROLLER_SPEED);
  }

  private void stopRoller() {
    setRollerSpeed(0);
  }

  public Command activateRoller() {
    return startEnd(() -> runRoller(), () -> stopRoller());
  }

  public Command runCommand(DoubleSupplier speedSupplier) {
    return runEnd(
        () -> {
          double speed = speedSupplier.getAsDouble();
          setRollerSpeed(speed);
        },
        () -> stopRoller());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lift.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    lift.simIterate();
  }
}
