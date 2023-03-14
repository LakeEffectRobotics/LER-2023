package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class GyroDriveStraightCommand extends CommandBase {
    Drivetrain drivetrain;
    Gyro gyro;
    DoubleSupplier rightSupplier;

    public GyroDriveStraightCommand(Drivetrain d, Gyro g, DoubleSupplier ds) {
        this.drivetrain = d;
        this.gyro = g;
        this.rightSupplier = ds;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
    	double[] straight_gyro_output = gyro.getStraightOutput(rightSupplier.getAsDouble(), rightSupplier.getAsDouble(), gyro.getAngle());
    	
        final double leftSpeed = straight_gyro_output[0] * drivetrain.MAX_SPEED;
        final double rightSpeed = straight_gyro_output[1] * drivetrain.MAX_SPEED;

    	drivetrain.velocityTankDrive(leftSpeed, rightSpeed);
    }

}
