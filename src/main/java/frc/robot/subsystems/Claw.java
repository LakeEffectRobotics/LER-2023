package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    CANSparkMax leadController;

    DoubleSolenoid rightSolenoid;
    DoubleSolenoid leftSolenoid;

    Position currentPosition;

    SparkMaxLimitSwitch clawLimitSwitch;

    // very large shuffleboard entry for limit swicth pressed
    private ShuffleboardTab tab = Shuffleboard.getTab("my favourite tab");
    public GenericEntry limitswitchShuffle = tab
        .add("limit switch!", false)
        .withSize(3, 3)
        .withPosition(7, 0)
        .getEntry();

        
    public Claw(CANSparkMax leadController, DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid) {
        this.leadController = leadController;
        this.rightSolenoid = rightSolenoid;
        this.leftSolenoid = leftSolenoid;

        this.clawLimitSwitch = leadController.getReverseLimitSwitch(Type.kNormallyOpen);
        clawLimitSwitch.enableLimitSwitch(true);

        
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
    @Override
    public void periodic() {
        // show on shuffleboard if limit switch pressed
        if (GetLimitPressed()) {
            limitswitchShuffle.setBoolean(true);
        } else {
            limitswitchShuffle.setBoolean(false);
        }
    } 
    
    public boolean GetLimitPressed() {
        return clawLimitSwitch.isPressed();
    }
}
