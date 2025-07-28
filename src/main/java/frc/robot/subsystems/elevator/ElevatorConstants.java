package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
     public enum State {
        L4(Units.radiansToRotations(23.299634187195004)),
        L3(2.146240234375),
        L2(1.1181640625),
        L1(Units.radiansToRotations(4.947855031325136)),
        HOME(0.0);

        private final double omegaRotations;

        State(double omega) {
            this.omegaRotations = omega;
        }

        public Rotation2d getAngle() {
            return Rotation2d.fromRotations(omegaRotations);
        }
    }

    public static final CANBus CAN_BUS = new CANBus("canivore");

    public static final int LEFT_ID = 13;
    public static final int RIGHT_ID = 14;

    public static final double REDUCTION = 25.0 / 1.0; // 25 rotations of motor one rotation of mechanism

    public static final Double MAX_VELOCITY_ROT_PER_SEC = 15.0;
    public static final Double MAX_ACCELERATION_ROT_PER_SEC_2 = 20.0;

    public static final double kP = 18.67;
    public static final double kI = 0;
    public static final double kD = 0.0;

    public static final double kS = 0.03;
    public static final double kG = 0.25;
    public static final double kV = 2.63;
}
