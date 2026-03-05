package org.firstinspires.ftc.teamcode.aedricsJunk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class aedricTransfer
{
    private DcMotor transferMotor;

    private void init(HardwareMap hwMap)
    {
        transferMotor = hwMap.get(DcMotor.class, "transferMotor");
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Transfer(boolean buttonIsPressed)
    {
        if (buttonIsPressed)
        {
            transferMotor.setPower(100);
        }
    }
}
