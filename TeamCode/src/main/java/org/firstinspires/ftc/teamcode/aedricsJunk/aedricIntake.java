package org.firstinspires.ftc.teamcode.aedricsJunk;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class aedricIntake
{
    private DcMotor intakeMotor;

    public void init(HardwareMap hwMap)
    {
        intakeMotor = hwMap.get(DcMotor.class, "intakeM");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Intake(boolean buttonIsPressed)
    {
        if (buttonIsPressed)
        {
            intakeMotor.setPower(100);
        }
    }
}
