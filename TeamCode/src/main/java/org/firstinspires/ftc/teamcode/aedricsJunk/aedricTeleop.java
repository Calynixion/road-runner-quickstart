package org.firstinspires.ftc.teamcode.aedricsJunk;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class aedricTeleop extends OpMode
{
    double driveSpeed;

    aedricMotorDrive motors = new aedricMotorDrive();
    aedricIntake Intake = new aedricIntake();
    aedricTransfer Transfer = new aedricTransfer();
    aedricLauncher Launcher = new aedricLauncher();

    @Override
    public void init()
    {
        motors.init(hardwareMap);

        driveSpeed = 100;
    }

    boolean dpadRightPressedLastCycle;
    boolean dpadLeftPressedLastCycle;

    @Override
    public void init_loop()
    {
        if (gamepad1.dpad_right && driveSpeed <= 95 && !dpadRightPressedLastCycle)
        {
            driveSpeed += 5;
        }

        if (gamepad1.dpad_left && driveSpeed >= 5 && !dpadLeftPressedLastCycle)
        {
            driveSpeed -= 5;
        }

        dpadRightPressedLastCycle = gamepad1.dpad_right;
        dpadLeftPressedLastCycle = gamepad1.dpad_left;
    }

    @Override
    public void start()
    {
        driveSpeed /= 100;
    }

    @Override
    public void loop()
    {
        telemetry.addData("Reversed?", motors.isReversed);
        telemetry.addData("Drive Speed", driveSpeed);

        motors.drive(gamepad1.left_stick_x, gamepad1.left_stick_x,
                     gamepad1.right_stick_x, gamepad1.dpad_down, driveSpeed);

        Intake.Intake(gamepad1.left_bumper);
        Transfer.Transfer(gamepad1.left_bumper);
        Launcher.Launch(gamepad1.right_trigger);
    }
}
