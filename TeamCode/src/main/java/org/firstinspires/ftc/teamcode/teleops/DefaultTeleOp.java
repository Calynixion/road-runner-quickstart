package org.firstinspires.ftc.teamcode.teleops;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.commands.setTrigger;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Bot_Trigger;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.vision.EOCVAprilTagPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.seattlesolvers.solverslib.command.button.Trigger;


import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;


//OpMode is actual code that is initialized and ran, calls commands which call methods of subsystems
@TeleOp(name="DefaultTeleOp")
public class DefaultTeleOp extends CommandOpMode {
    //initialize variables
    public double detectedX;
    public double detectedY;
    Drive drivetrain;
    RobotCentricDrive r_drive;
    GamepadEx controller1;
    Intake intake;
    Shooter shooter;
    Bot_Trigger trigger;
    Blocker blocker;
    Trigger RT;
    setTrigger triggerCmd;
    int i = 0;
    EOCVAprilTagPipeline aprilTagDetectionPipeline;


    public double naught;

    //initialize function runs when init is pressed on the driver station with this teleop selected
    @Override
    public void initialize(){
        //assign variables to their respective objects
        controller1 = new GamepadEx(gamepad1);
        drivetrain = new Drive(hardwareMap, telemetry);
        r_drive = new RobotCentricDrive(drivetrain,controller1::getLeftX,controller1::getLeftY,controller1::getRightX);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        trigger = new Bot_Trigger(hardwareMap, telemetry );
        blocker = new Blocker(hardwareMap, telemetry);
        triggerCmd = new setTrigger(trigger,0);
        RT = new Trigger() {
            public boolean get() {
                return gamepad1.right_trigger > 0.4;
            }
        };

        //register the subsystems to the command scheduler
        register(drivetrain,intake,shooter,trigger,blocker);

        //sets the drivetrain subsystem to run the robot centric command continuously
        drivetrain.setDefaultCommand(r_drive);
        trigger.setDefaultCommand(triggerCmd);



        // Setup

        //example of instant commands to call directly from subsystems without a custom command
        controller1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(intake::spin))
                .whenReleased(new InstantCommand(intake::stop));

        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(intake::reverseSpin))
                .whenReleased(new InstantCommand(intake::stop));

        controller1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenReleased(new InstantCommand(drivetrain::changeDirection));

        controller1.getGamepadButton(GamepadKeys.Button.B)
                .whenReleased(new InstantCommand(blocker::positionZero));
        /*
        controller1.getGamepadButton(GamepadKeys.Button.A)
                .whenReleased(new InstantCommand(blocker::positionBlock));
        */

        controller1.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new InstantCommand(trigger::reverseShoot))
                .whenReleased(new InstantCommand(trigger::stop));
        controller1.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new InstantCommand(trigger::shoot))
                .whenReleased(new InstantCommand(trigger::stop));


        controller1.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.spin(gamepad1.right_trigger)),
                        new InstantCommand(trigger::reverseShoot),
                        new InstantCommand(blocker::positionBlock),
                        new WaitCommand(1000),
                        new InstantCommand(blocker::positionDefault),
                        new InstantCommand(trigger::shoot),
                    new InstantCommand(intake::spin))))
                .whenReleased(new ParallelCommandGroup(
                        new InstantCommand(shooter::stop),
                        new InstantCommand(trigger::stop),
                        new InstantCommand(intake::stop)
                ));



        RT.whileActiveContinuous(
                new InstantCommand(() -> shooter.spin(1), shooter)
        );

        RT.whenInactive(
                new InstantCommand(shooter::stop, shooter)
        );
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new EOCVAprilTagPipeline(telemetry);
        aprilTagDetectionPipeline.setDecimation(3);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });




    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Blocker Angle",blocker.get_angle());
        telemetry.addLine();
        telemetry.addData("Blocker Pos",blocker.get_position());
        telemetry.addLine();
        ArrayList<AprilTagDetection> detectedTags = aprilTagDetectionPipeline.getLatestDetections();
        if (!detectedTags.isEmpty()) {
            for (AprilTagDetection detectedTag : detectedTags) {
                telemetry.addData("Pipeline", "Tag " + detectedTag.id + " found" );
                telemetry.addData("Distance", detectedTag.pose.x);
            }
        }   else {
            telemetry.addData("Pipeline","No tags found");
        }
        telemetry.update();
    }


}
