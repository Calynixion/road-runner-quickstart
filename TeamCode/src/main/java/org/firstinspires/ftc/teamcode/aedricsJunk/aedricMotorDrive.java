package org.firstinspires.ftc.teamcode.aedricsJunk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class aedricMotorDrive
{
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;

    public void init(HardwareMap hwMap)
    {
        flMotor = hwMap.get(DcMotor.class, "flMotor");
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frMotor = hwMap.get(DcMotor.class, "frMotor");
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blMotor = hwMap.get(DcMotor.class, "blMotor");
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        brMotor = hwMap.get(DcMotor.class, "brMotor");
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    double flPower;
    double frPower;
    double blPower;
    double brPower;

    public boolean wasReversed = false;
    public boolean isReversed;

    public void drive(double forward, double strafe, double rotate, boolean reverse, double powMod)
    {
        if (reverse && !wasReversed)
        {
            isReversed = true;
        }

        if (isReversed)
        {
            rotate *= -1;
        }

        flPower = forward - strafe - rotate;
        frPower = forward + strafe + rotate;
        blPower = forward + strafe - rotate;
        brPower = forward - strafe + rotate;

        if (isReversed)
        {
            flPower *= -1;
            frPower *= -1;
            blPower *= -1;
            brPower *= -1;
        }

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(flPower));
        maxPower = Math.max(maxPower, Math.abs(frPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));

        flMotor.setPower((maxSpeed * (flPower / maxPower)) * powMod);
        frMotor.setPower((maxSpeed * (frPower / maxPower)) * powMod);
        blMotor.setPower((maxSpeed * (flPower / maxPower)) * powMod);
        brMotor.setPower((maxSpeed * (flPower / maxPower)) * powMod);

        boolean wasReversed = reverse;
    }
}
