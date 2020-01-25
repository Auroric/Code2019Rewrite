package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ComponentConstants;
import frc.robot.Constants.HatchConstants;

public class HatchEffector implements Subsystem {

    public static DoubleSolenoid eject = new DoubleSolenoid(ComponentConstants.kPCM_ID, HatchConstants.kEjectForward, HatchConstants.kEjectReverse);
    public static DoubleSolenoid retainer = new DoubleSolenoid(ComponentConstants.kPCM_ID, HatchConstants.kRetainerForward, HatchConstants.kRetainerReverse);

    private static HatchEffector instance = null;
    public static HatchEffector getInstance() {
        if (instance == null)
            instance = new HatchEffector();
        return instance;
    }

    private HatchEffector() {
        
    }

    /* Hatch ejector methods */
    public void extend() {
        eject.set(Value.kForward);
    }

    public void retract() {
        eject.set(Value.kReverse);
    }

    public void alternate_ejector() {
        if (eject.get() == Value.kReverse) {
            extend();
        } else {
            retract();
        }
    }

    /* Retaining clamp methods */
   
    public void retain() {
        retainer.set(Value.kReverse);
    }

    public void release() {
        retainer.set(Value.kForward);
    }

    public void alternate_retainer() {
        if (retainer.get() == Value.kReverse) {
            release();
        } else {
            retain();
        }
    }
}
