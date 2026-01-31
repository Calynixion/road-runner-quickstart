package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.EOCVAprilTagPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class AprilTagDetectionTest extends LinearOpMode {
    EOCVAprilTagPipeline aprilTagDetectionPipeline;
    Drive drivetrain;
    public int amount_found = 0;
    /*
    double fx = 679.2888908044871;
    double fy = 679.0590608430991;
    double cx = 399.04720194230583;
    double cy = 301.4138740002473;
    double tagsize = 0.173;
     */
    @Override
    public void runOpMode() throws InterruptedException {
        //drivetrain = new Drive(hardwareMap, telemetry);
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

        //FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detectedTags = aprilTagDetectionPipeline.getLatestDetections();
            if (!detectedTags.isEmpty()) {
                for (AprilTagDetection detectedTag : detectedTags) {
                    telemetry.addData("Pipeline", "Tag " + detectedTag.id + " found");
                    amount_found+=1;
                }
            }   else {
                telemetry.addData("Pipeline","No tags found");
            }
            telemetry.update();
        }

    }


}




