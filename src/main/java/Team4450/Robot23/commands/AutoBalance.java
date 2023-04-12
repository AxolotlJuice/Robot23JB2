package Team4450.Robot23.commands;

import Team4450.Robot23.RobotContainer;
import Team4450.Robot23.subsystems.DriveBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class AutoBalance extends PIDCommand
{
    // NEEDS TUNING
    private static double kP = 1.2, kI = .15, kD = 0, kToleranceDegrees = 10;

    private DriveBase driveBase;

    public AutoBalance(DriveBase driveBase)
    {
        super(
                new PIDController(kP, kI, kD),
                () -> RobotContainer.navx.getAHRS().getPitch(),
                0.0,
                (output) -> driveBase.drive(output, 0, 0
        ));

        this.driveBase = driveBase;
    }

    @Override
    public boolean isFinished()
    {
        return (Math.abs(RobotContainer.navx.getAHRS().getPitch())) <= kToleranceDegrees;
    }

    @Override
    public void end(boolean interrupted)
    {
        driveBase.stop();
    }
}