package Team4450.Robot23.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class Arm extends SubsystemBase
{
    private Winch           winch;
    private CANSparkMax     motor = new CANSparkMax(ARM_MOTOR, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private DigitalInput    limitSwitch = new DigitalInput(ARM_SWITCH);

    //private final double    ARM_MAX = 102;      // Revolutions of motor, not spool due to gearbox.
    private final double    ARM_MAX = 245;      // Revolutions of motor, not spool due to gearbox.
    //private final double    ARM_LIMIT = 50;
    private final double    ARM_LIMIT = 120;
    private final double    WINCH_LIMIT = -55;

    public Arm(Winch winch)
    {
        Util.consoleLog();

        this.winch = winch;

        // Arm will start all the way retracted and that is encoder zero.
        // Encoder max will represent arm fully extended.

        resetPosition();
    }

    /**
     * Set Arm motor power.
     * @param power -1..+1, + is retract arm, - is extend arm.
     */
    public void setPower(double power)
    {
        // Limit extension if winch too low.

        double armMax = ARM_MAX;

        //if (winch.getPosition() < WINCH_LIMIT) armMax = ARM_LIMIT;

        // If power positive, which means retract, check encoder stop if 1 or less.
        // 1 instead of zero for safety.
        // If power negative, which means extend, check encoder for max extend, stop if there.

        //if ((power > 0 && limitSwitch.get()) || (power < 0 && encoder.getPosition() >= ARM_MAX)) power = 0;

        if ((power > 0 && getPosition() <= 1) || (power < 0 && getPosition() >= armMax)) power = 0;
        
        //if ((power < 0 && encoder.getPosition() >= ARM_MAX)) power = 0;

        //if (limitSwitch.get()) resetPosition();

        power = Util.squareInput(power);

        power = Util.clampValue(power, .75);
        
        // Invert since they put the rope on the spools the wrong way...

        motor.set(power);
    }

    /**
     * Set Arm motor power.
     * @param power -1..+1, + is retract arm, - is extend arm.
     */
    public void setPowerNoLimit(double power)
    {
        power = Util.squareInput(power);

        power = Util.clampValue(power, .75);
        
        // Invert since they put the rope on the spools the wrong way...

        motor.set(power);
    }

    public void stop()
    {
        motor.stopMotor();
    }

    /**
     * Return Arm encoder position.
     * @return Position in revolutions.
     */
    public double getPosition()
    {
        return -encoder.getPosition();
    }

    /**
     * Reset Arm encoder to zero.
     */
    public void resetPosition()
    {
        encoder.setPosition(0);
    }

    /**
     * Return Arm retracted limit switch state. Not currently implemented.
     * @return True if switch contacted (arm fully retracted).
     */
    public boolean getSwitch()
    {
        return limitSwitch.get();
    }

    public void updateDS()
    {

    }

    public RelativeEncoder getEncoder(){
        return encoder;
    }

    public CANSparkMax getMotor(){
        return motor;
    }
}
