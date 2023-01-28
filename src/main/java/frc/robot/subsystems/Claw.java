package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    CANSparkMax leftController;
    CANSparkMax rightController;

    DoubleSolenoid solenoid;

    Position currentPosition;

    public Claw(CANSparkMax leftController, CANSparkMax rightController, DoubleSolenoid solenoid) {
        this.leftController = leftController;
        this.rightController = rightController;
        this.solenoid = solenoid;
    }

    public enum Position {
        OPEN(DoubleSolenoid.Value.kForward),
        CLOSED(DoubleSolenoid.Value.kReverse),
        OFF(DoubleSolenoid.Value.kOff);

        private DoubleSolenoid.Value value;

        private Position(DoubleSolenoid.Value value) {
            this.value = value;
        }

        public DoubleSolenoid.Value getValue() {
            return value;
        }
    }

    // Motors
    public void setSpeed(double speed) {
        leftController.set(speed);
        rightController.set(speed);
    }

    // Solenoids
    public void setPosition(Position position) {
        currentPosition = position;
        solenoid.set(position.value);
    }

    public Position getPosition() {
        return currentPosition;
    }
}
