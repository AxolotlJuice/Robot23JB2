package Team4450.Robot23.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class Intake extends SubsystemBase
{
    private WPI_TalonFX     motor = new WPI_TalonFX(CLAW_MOTOR);
    private FXEncoder       encoder = new FXEncoder(motor);

    public Intake()
    {
        Util.consoleLog();

        motor.setInverted(true);

        encoder.setInverted(true);
        
    }

    /**
     * Sets intake motor power.
     * @param power (+) means intake cube, (-) means intake cone.
     */
    public void setPower(double power)
    {

        power = Util.clampValue(power, .20);
        
        motor.set(power);
    }

    public void stop()
    {
        motor.stopMotor();
    }

    /**
     * Return encoder tick count.
     * @return The current tick count.
     */
    public int getPosition()
    {
        return encoder.get();
    }

    /**
     * Reset Intake encoder to zero.
     */
    public void resetPosition()
    {
        encoder.reset();
    }

    public void updateDS()
    {

    }
}