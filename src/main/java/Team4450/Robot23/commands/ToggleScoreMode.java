package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Claw;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import Team4450.Robot23.Constants;
import Team4450.Robot23.RobotContainer;
import Team4450.Robot23.Constants.*;

/**
 * Moves the Claw to a target position.
 */
public class ToggleScoreMode extends CommandBase 
{

    /**
     * Move claw to target position (closing it).
     * @param claw Claw subsystem.
     * @param targetPosition Target position in encoder tick counts.
     */
    public ToggleScoreMode()
    {
        Util.consoleLog();
        
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        if(Constants.scoreMode == ScoreMode.CONE)
            Constants.scoreMode = ScoreMode.CUBE;
		else
            Constants.scoreMode = ScoreMode.CONE;
            
    }

    @Override
    public void execute()
    {
        
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
