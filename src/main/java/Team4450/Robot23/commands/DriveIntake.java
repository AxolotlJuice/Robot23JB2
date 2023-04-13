package Team4450.Robot23.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Intake;

import static Team4450.Robot23.Constants.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveIntake extends CommandBase 
{
    private final Intake            intake;

    private final DoubleSupplier    intakeSupplier;

    public DriveIntake(Intake intake, DoubleSupplier intakeSupplier)
    {
        Util.consoleLog();

        this.intake = intake;
        this.intakeSupplier = intakeSupplier;

        addRequirements(intake);
    }

    @Override
    public void execute()
    {
        double power = deadband(intakeSupplier.getAsDouble(), THROTTLE_DEADBAND);

        power = Util.squareInput(power);

        intake.setPower(power);
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);
    }

    private static double deadband(double value, double deadband) 
    {
        return Math.abs(value) > deadband ? value : 0.0;
    }
}
