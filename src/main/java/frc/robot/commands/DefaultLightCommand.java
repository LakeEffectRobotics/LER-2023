package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Lights.Colour;
import frc.robot.subsystems.TargetSelection.Type;

public class DefaultLightCommand extends CommandBase {
    Lights lights;
    TargetSelection targetSelection;
    Claw claw;

    public DefaultLightCommand(Lights lights, TargetSelection targetSelection, Claw claw) {
        this.lights = lights;
        this.targetSelection = targetSelection;
        this.claw = claw;
        addRequirements(lights, targetSelection);
    }

   @Override
   public void execute() {
    // if claw limit switch is pressed, green to show successfully picked up piece
    if (claw.GetLimitPressed()) {
        lights.setBoth(Colour.GREEN);
    } else if (targetSelection.getSelectedNode().getType() == Type.CONE) {
        // otherwise, set to target selection color
        lights.setBoth(Colour.YELLOW);
    } else {
        lights.setBoth(Colour.PURPLE);
    }
   }
    
}
