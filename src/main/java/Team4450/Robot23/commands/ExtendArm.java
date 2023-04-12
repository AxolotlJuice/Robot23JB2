package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves the arm to a target position.
 */
public class ExtendArm extends CommandBase 
{
    private final Arm       arm;
    private double          targetPostion = 0;    // Revolutions of motor.
    //private SynchronousPID  controller = new SynchronousPID(.10, .001, .025);
    //private SynchronousPID  controller = new SynchronousPID(.10, .005, .05);
    private SynchronousPID  controller = new SynchronousPID(.28, .015, .10);
    //private final double    tolerance = 2, maxPower = .40;
    private final double    tolerance = 2, maxPower = .75;
    private double          lastTimeCalled;

    /**
     * Move arm to target position.
     * @param arm Arm subsystem.
     * @param targetPosition Target position in arm motor revolutions.
     */
    public ExtendArm(Arm arm, double targetPosition)
    {
        Util.consoleLog();

        this.arm = arm;

        this.targetPostion = targetPosition;

        addRequirements(arm);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        controller.reset();

        controller.setSetpoint(targetPostion);

        controller.setOutputRange(-maxPower, maxPower);

        SmartDashboard.putBoolean("ExtendArm", true);

        lastTimeCalled = Util.timeStamp();
    }

    @Override
    public void execute()
    {
        double time = Util.getElaspedTime(lastTimeCalled);

        lastTimeCalled = Util.timeStamp();

        double power = controller.calculate(arm.getPosition(), time);

        // Invert power because calculation above returns + result but arm
        // wants - power to extend.

        arm.setPower(-power);
    }

    @Override
    public boolean isFinished()
    {
        return controller.onTarget(tolerance); 
    }

    @Override
    public void end(boolean interrupted) 
    {
        arm.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("ExtendArm", false);
    }
}

