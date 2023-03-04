package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lights;

public class DiscoCommand extends CommandBase {

  byte leftColour = 0b001;
  byte rightColour = 0b001;
  final byte RED = 0b100;
  final byte BLUE = 0b010;
  final byte GREEN = 0b001;
  int delay = 1;
  Lights lights;
  
  public DiscoCommand(Lights lights) {
    addRequirements(lights);
    this.lights = lights;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Robot.lights.setColour(Lights.LEFT, (leftColour & RED) == RED, (leftColour & GREEN) == GREEN,
        (leftColour & BLUE) == BLUE);
    Robot.lights.setColour(Lights.RIGHT, (rightColour & RED) == RED, (rightColour & GREEN) == GREEN,
        (rightColour & BLUE) == BLUE);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (delay < 7) {
      delay++;
      return;
    }
    delay = 0;
    leftColour += 0b001;
    if (leftColour > 0b111)
      leftColour = 0b001;
    rightColour -= 0b001;
    if (rightColour == 0b000)
      rightColour = 0b111;

    Robot.lights.setColour(Lights.LEFT, (leftColour & RED) == RED, (leftColour & GREEN) == GREEN,
        (leftColour & BLUE) == BLUE);
    Robot.lights.setColour(Lights.RIGHT, (rightColour & RED) == RED, (rightColour & GREEN) == GREEN,
        (rightColour & BLUE) == BLUE);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }
}