package org.firstinspires.ftc.teamcode.aedricsJunk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class aedricLauncher
{
    private DcMotor LauncherLeft;
    private DcMotor LauncherRight;

    public void init(HardwareMap hwMap)
    {
        LauncherLeft = hwMap.get(DcMotor.class, "LauncherLeft");
        LauncherLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LauncherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LauncherRight = hwMap.get(DcMotor.class, "LauncherRight");
        LauncherRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LauncherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LauncherRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Launch(double launchPower)
    {
        LauncherLeft.setPower(launchPower);
        LauncherRight.setPower(launchPower);
    }
}
