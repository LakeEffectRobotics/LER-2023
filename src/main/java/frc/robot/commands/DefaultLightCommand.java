package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.TargetSelection;
import frc.robot.subsystems.Lights.Colour;

public class DefaultLightCommand extends CommandBase {
    Lights lights;
    TargetSelection targetSelection;

    public DefaultLightCommand(Lights lights, TargetSelection targetSelection) {
        this.lights = lights;
        this.targetSelection = targetSelection;
        addRequirements(lights, targetSelection);
    }

   @Override
   public void execute() {
    if (targetSelection.getSelectedNode().getType().name() == "CUBE") {
        lights.setBoth(Colour.PURPLE);
    } else {
        lights.setBoth(Colour.YELLOW);
    }
   }
    
}
