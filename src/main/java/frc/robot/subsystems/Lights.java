package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
     * First relay has power and the red line; second relay has green and blue red
     * green and blue lines must be LOW to turn ON.
 * 
 * //Red: RobotMap.leftLED_PR.set(Relay.Value.kForward); //12 0
 * RobotMap.leftLED_GB.set(Relay.Value.kOn); //12 12 //Green:
 * RobotMap.leftLED_PR.set(Relay.Value.kOn); //12 12
 * RobotMap.leftLED_GB.set(Relay.Value.kReverse); //0 12 //Blue:
 * RobotMap.leftLED_PR.set(Relay.Value.kOn); //12 12
 * RobotMap.leftLED_GB.set(Relay.Value.kForward); //12 0 //Purple:
 * RobotMap.leftLED_PR.set(Relay.Value.kForward); //12 0
 * RobotMap.leftLED_GB.set(Relay.Value.kForward); //12 0 //White:
 * RobotMap.leftLED_PR.set(Relay.Value.kForward); //12 0
 * RobotMap.leftLED_GB.set(Relay.Value.kOff); //0 0
 * 
 */
public class Lights extends SubsystemBase {
  public enum Colour {
    RED(new boolean[] { true, false, false }), GREEN(new boolean[] { false, true, false }),
    BLUE(new boolean[] { false, false, true }), PURPLE(new boolean[] { true, false, true }),
    WHITE(new boolean[] { true, true, true }), YELLOW(new boolean[] { true, true, false }),
    OFF(new boolean[] { false, false, false });

    private boolean[] value;

    private Colour(boolean[] value) {
      this.value = value;
    }

    public boolean[] getValue() {
      return this.value;
    }
  }

  public static int LEFT = 0;
  public static int RIGHT = 1;

  public void setColour(int side, Colour c) {
    setColour(side, c.getValue()[0], c.getValue()[1], c.getValue()[2]);
  }

  public void setBoth(Colour c) {
    setColour(LEFT, c);
    setColour(RIGHT, c);
  }

  public void setColour(int side, boolean r, boolean g, boolean b) {
    Relay pb;
    Relay gr;

    if (side == LEFT) {
      pb = RobotMap.leftLED_PB;
      gr = RobotMap.leftLED_GR;
    } else if (side == RIGHT) {
      pb = RobotMap.rightLED_PB;
      gr = RobotMap.rightLED_GR;
    } else {
      return;
    }

    if (!(r || g || b)) {
      pb.set(Value.kReverse);
      gr.set(Value.kOn);
      return;
    }
    if (b) {
      pb.set(Value.kForward);
    } else {
      pb.set(Value.kOn);
    }

    if (g && r) {
      gr.set(Value.kOff);
    } else if (g) {
      gr.set(Value.kReverse);
    } else if (r) {
      gr.set(Value.kForward);
    } else {
      gr.set(Value.kOn);
    }

  }

  public void setBoth(boolean r, boolean g, boolean b) {
    setColour(LEFT, r, g, b);
    setColour(RIGHT, r, g, b);
  }

}