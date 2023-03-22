package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.TargetSelection;

public class LimelightAimCommand extends CommandBase {
    Limelight limelight;
    Drivetrain drivetrain;
    TargetSelection targetSelection;
    Claw claw;

    private PIDController pidController = new PIDController(0.01, 0.0, 0.0);

    public LimelightAimCommand(Limelight limelight, Drivetrain drivetrain, TargetSelection targetSelection, Claw claw) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.targetSelection = targetSelection;
        this.claw = claw;
    }

    @Override
    public void initialize() {
        pidController.setSetpoint(0);
        
        // if has limit switch pressed, assume we have a game piece so target scoring
        // TODO: add manual override for limit switch condition in case limit switch breaks
        if (claw.GetLimitPressed()) {
            // TODO: make pipelines for each scoring node using apriltag 3d thing ?
            limelight.setPipeline(Limelight.Pipeline.APRILTAG);
        } else {
            if (targetSelection.getSelectedNode().getType().name() == "CUBE") {
                limelight.setPipeline(Limelight.Pipeline.CUBE);
            } else {
                limelight.setPipeline(Limelight.Pipeline.CONE);
            }
        }


    }

    @Override
    public void execute() {
        double heading_error = limelight.getX();
        double steering_adjust = pidController.calculate(heading_error);
        drivetrain.tankDrive(-steering_adjust, steering_adjust);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        // avoid blinding everyone
        limelight.setPipeline(Limelight.Pipeline.APRILTAG);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
