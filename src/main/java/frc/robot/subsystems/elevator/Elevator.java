package frc.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TunableNumber;
import frc.robot.subsystems.elevator.ElevatorConstants.State;

public class Elevator extends SubsystemBase {
    private final TalonFXConfiguration config;

    private final TalonFX leftMotor, rightMotor;
    private final MotionMagicVoltage motionMagicVoltage;
    private final DutyCycleOut dutyCycle;
    private final VoltageOut voltageOut;
    private final TunableNumber volts
        = new TunableNumber("Elevator/Volts", 0.0);

    private ElevatorConstants.State desiredState;
    private double voltage = 0.0;
    private boolean isManual = false;

    public Elevator() {
        leftMotor = new TalonFX(ElevatorConstants.LEFT_ID, ElevatorConstants.CAN_BUS);
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_ID, ElevatorConstants.CAN_BUS);
        config = new TalonFXConfiguration();

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true)); // right motor follows all commands on leftMotor

        motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);
        dutyCycle = new DutyCycleOut(0.0);
        voltageOut = new VoltageOut(0.0);
        this.desiredState = State.HOME;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0 = new Slot0Configs()
            .withKP(ElevatorConstants.kP)
            .withKI(ElevatorConstants.kI)
            .withKD(ElevatorConstants.kD)
            .withKS(ElevatorConstants.kS)
            .withKG(ElevatorConstants.kG)
            .withKV(ElevatorConstants.kV)
            .withGravityType(GravityTypeValue.Elevator_Static); // Other is Arm_Cosine
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.REDUCTION;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY_ROT_PER_SEC;
        config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MAX_ACCELERATION_ROT_PER_SEC_2;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config, 0.25));
        zeroEncoders();
    }

    public void zeroEncoders() {
        new Thread(() -> {
            leftMotor.setPosition(0.0);
            rightMotor.setPosition(0.0);
        }).start();
    }

    public void stop() {
        leftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/PositionRot", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/velocity", leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/acceleration", leftMotor.getAcceleration().getValueAsDouble());
        if (!isManual) {
            leftMotor.setControl(
                motionMagicVoltage.withPosition(desiredState.getAngle().getRotations()));
        }
    }

    public Command setDesiredState(ElevatorConstants.State desiredState) {
        return Commands.runOnce(() -> {
            isManual = false;
            this.desiredState = desiredState;
        }, this);
    }

    public Command applyVolts() {
        return Commands.runOnce(() -> isManual = true)
            .andThen(
                Commands.run(() -> leftMotor.setControl(voltageOut.withOutput(volts.get())), this)
            .finallyDo(this::stop));
    }

    public Command homeCommand() {
        return setDesiredState(State.HOME);
    }

    public void runElevator(double volts) {
        leftMotor.setControl(
            voltageOut.withOutput(volts));
    }

    public void runCharacterizer() {
        runElevator(volts.get());
    }

    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }
}
