package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    CANSparkMax leadController;

    DoubleSolenoid rightSolenoid;
    DoubleSolenoid leftSolenoid;

    Position currentPosition;

    SparkMaxLimitSwitch clawLimitSwitch;

    public Claw(CANSparkMax leadController, DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid) {
        this.leadController = leadController;
        this.rightSolenoid = rightSolenoid;
        this.leftSolenoid = leftSolenoid;

        this.clawLimitSwitch = leadController.getForwardLimitSwitch(Type.kNormallyOpen);
        clawLimitSwitch.enableLimitSwitch(false);

        
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
        if (speed < 0) {
            // half speed intake direction 
            speed = speed / 2;
        }
        leadController.set(speed);
    }

    // Solenoids
    public void setPosition(Position position) {
        currentPosition = position;
        rightSolenoid.set(position.value);
        leftSolenoid.set(position.value);
    }

    public Position getPosition() {
        return currentPosition;
    }

    public boolean GetLimitPressed() {
        return clawLimitSwitch.isPressed();
    }
}
