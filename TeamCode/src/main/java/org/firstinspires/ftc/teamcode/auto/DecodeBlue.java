package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.shoot_3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Bot_Trigger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.vision.EOCVAprilTagPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "AutoBlue", group = "Autos")
public class DecodeBlue extends OpMode {
    EOCVAprilTagPipeline aprilTagDetectionPipeline;

    private static Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private TelemetryManager telemetryM;
    private int pathState;

    private Shooter shooter;
    private Intake intake;
    private Bot_Trigger trigger;

    private Blocker blocker;

    Paths paths;
    private shoot_3 shootCmd;
    public Timer shoot_timer;
    public double shoot_power = 1;
    public double intake_power = 1;
    public static int tag;
    public boolean looking=true;

    public static final Pose shootPoseBlue = new Pose(48,96,Math.toRadians(315));
    public static final Pose cameraPoseBlue = new Pose(48,96,Math.toRadians(230));

    public static final Pose startPoseBlue = new Pose(20.6,122.7,Math.toRadians(321.46));


    @Override
    public void init() {
        tag=0;
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
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPoseBlue);
        paths = new Paths(follower);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap,telemetry);
        shootCmd = new shoot_3(shooter,1);
        shoot_timer = new Timer();
        intake = new Intake(hardwareMap, telemetry);
        trigger = new Bot_Trigger(hardwareMap, telemetry);
        blocker = new Blocker(hardwareMap,telemetry);



    }

    @Override
    public void loop() {
        /*
        ArrayList<AprilTagDetection> detectedTags = aprilTagDetectionPipeline.getLatestDetections();
        if (!detectedTags.isEmpty()) {
            for (AprilTagDetection detectedTag : detectedTags) {
                if((detectedTag.id<=23) && (detectedTag.id>=21) && looking){
                    tag=detectedTag.id;
                    looking=false;
                    paths = new Paths(follower);
                }
            }
        }

         */
        tag=21;
        looking=false;
        paths = new Paths(follower);
        telemetry.update();
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State: ",pathState);
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.debug("shoot_timer: " + shoot_timer.getElapsedTime());
        telemetryM.debug("auto_tag" + tag);
        telemetryM.update(telemetry);
        telemetry.update();
        draw();
    }
    public static void draw() {
        DrawingInAuto2.drawDebug(follower);
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        shoot_timer.resetTimer();
        setPathState(-1);
    }

    public static class Paths{

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path67;


        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPoseBlue,cameraPoseBlue)
                    )
                    .setConstantHeadingInterpolation(cameraPoseBlue.getHeading())
                    .build();
            Path67 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startPoseBlue, shootPoseBlue)
                    )
                    .setLinearHeadingInterpolation(startPoseBlue.getHeading(), shootPoseBlue.getHeading())
                    .build();
            if (tag==21) {
                Path2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 35.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                        .build();

                Path3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(48.000, 35.500), new Pose(14.000, 35.500))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                Path4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(14.000, 35.500),
                                        new Pose(51.500, 30.000),
                                        new Pose(48.000, 96.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                        .build();
            } else if (tag==22) {
                Path2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 59.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                        .build();

                Path3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(48.000, 59.500), new Pose(19.000, 59.500))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                Path4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(19.000, 59.500),
                                        new Pose(51.500, 59.500),
                                        new Pose(48.000, 96.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                        .build();
            } else if (tag==23) {
                Path2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 83.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                        .build();

                Path3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(48.000, 83.500), new Pose(19.000, 83.500))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                Path4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(19.000, 83.500),
                                        new Pose(51.500, 83.500),
                                        shootPoseBlue
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), shootPoseBlue.getHeading())
                        .build();

            }
        }
    }

    public void autonomousPathUpdate(){
        switch (pathState) {
            case -1:
                follower.followPath(paths.Path67);
                setPathState(-2);
                break;
            case -2:
                if (!follower.isBusy()){
                    shoot_timer.resetTimer();
                    setPathState(0);
                }
                break;
            case 0:
                //shooting code here, move on when done
                /*
                if (shoot_timer.getElapsedTime() <= 2000) {
                    shooter.spin(shoot_power);
                } else {
                    shooter.spin(0);
                    follower.followPath(paths.Path1, true);
                    setPathState(1);
                    shoot_timer.resetTimer();
                    follower.deactivateAllPIDFs();
                    follower.activateHeading();
                }
                 */
                if (shoot_3(shoot_timer)){
                    follower.followPath(paths.Path1, true);
                    setPathState(1);
                    shoot_timer.resetTimer();
                    follower.deactivateAllPIDFs();
                    follower.activateHeading();
                }

                break;
            case 1:
                if (shoot_timer.getElapsedTime()>=3000) {
                    follower.activateAllPIDFs();
                    if (!(tag==0)) {
                        follower.followPath(paths.Path2);
                        setPathState(2);
                    } else {
                        setPathState(-100);
                    }
                }
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;
            case 3:
                intake.spin();
                if(!follower.isBusy()){
                    intake.stop();
                    follower.followPath(paths.Path4,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    shoot_timer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                /*
                if (shoot_timer.getElapsedTime()<=2000) {
                    shooter.spin(shoot_power);
                }else{
                    shooter.spin(0);
                    tag=0;
                    setPathState(-100);
                }

                 */
                if (shoot_3(shoot_timer)){
                    tag=0;
                    setPathState(-100);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public boolean shoot_3(Timer timer){
        if (timer.getElapsedTime()<=1000) {
            shooter.spin(shoot_power);
            telemetry.addData("Shooting:","spinning up");
            return false;
        } else if (timer.getElapsedTime()>1000 && timer.getElapsedTime()<=4000) {
            shooter.spin(shoot_power);
            intake.spin();
            trigger.shoot();
            telemetry.addData("Shooting:","firing");
            return false;
        } else {
            shooter.spin(0);
            intake.stop();
            trigger.stop();
            telemetry.addData("Shooting:","stopping");
            return true;
        }
    }

}
class DrawingInAuto2 {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}
