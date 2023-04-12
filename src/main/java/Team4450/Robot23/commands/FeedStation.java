package Team4450.Robot23.commands;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Claw;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Lower arm from fully up position to correct height for feeder station,
 * open the claw and extend the arms to grab position in prep for pickup.
 */
public class FeedStation extends CommandBase 
{
    private final Winch             winch;
    private final Arm               arm;
    private final Claw              claw;
    private ParallelCommandGroup    pCommands; 
    private SequentialCommandGroup	commands;

    public FeedStation(Winch winch, Arm arm, Claw claw)
    {
        Util.consoleLog();

        this.winch = winch;
        this.arm = arm;
        this.claw = claw;

        addRequirements(winch, arm, claw);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();
		
		commands = new SequentialCommandGroup();

		// Some commands will be run in parallel.

		pCommands = new ParallelCommandGroup();

		// First action is to lower the arm.

		Command command = new LowerArm(winch, -37);

		pCommands.addCommands(command);
        
        // Next action is to open the claw.

        command = new OpenClaw(claw);

		pCommands.addCommands(command);

        // Next action is to extend arms.

        command = new ExtendArm(arm, 35.5);

		pCommands.addCommands(command);
        
        // Add parallel commands to sequential commands.

        commands.addCommands(pCommands);

        // Now hold winch position.

        command = new InstantCommand(winch::toggleHoldPosition);

		commands.addCommands(command);

        // Run the commands, only if winch mostly up.

        //if (winch.getUpperSwitch()) commands.schedule();
        if (winch.getPosition() > -5) commands.schedule();

        SmartDashboard.putBoolean("FeedStation", true);
    }

    @Override
    public boolean isFinished()
    {
		// Note: commands.isFinished() will not work to detect the end of the command list
		// due to how FIRST coded the SquentialCommandGroup class. 
		
		return !commands.isScheduled();
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);

        if (interrupted) commands.cancel();

        SmartDashboard.putBoolean("FeedStation", false);
    }
}

