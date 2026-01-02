package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Red Goal Pedro 9", group = "Autonomous")
@Configurable
public class redGoalPedro9 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private DcMotorEx intakeMotor;
    private DcMotorEx outtakeMotorLeft;
    private DcMotorEx outtakeMotorRight;
    private DcMotorEx transferMotor;

    private int pathState = 0;
    private boolean pathStarted = false;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int detectedTagId = -1;
    private long visionStartTime;
    private static final long VISION_TIMEOUT_MS = 1000;

    private static final long Wait1 = 1000;
    private static final long Wait2 = 2000;
    private static final long Wait3 = 5000;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outtake_motor_left");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outtake_motor_right");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor_left");

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Tag", detectedTagId);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: 
                startPath(paths.Path1); 
                break;
            case 1: 
                runShooter(); 
                break;
            case 2: 
                startPath(paths.Path2); 
                break;
            case 3: 
                runVision(); 
                break;
            case 4: 
                startPath(paths.Path3); 
                break;
            case 5: 
                runPath4WithIntake(); 
                break;
            case 6: 
                startPath(paths.Path5); 
                break;
            case 7: 
                runShooter(); 
                break;
            case 8: 
                startPath(paths.Path6); 
                break;
            case 9: 
                runPath7WithIntake(); 
                break;
            case 10: 
                startPath(paths.Path8); 
                break;
            case 11: 
                runShooter(); 
                break;
            case 12: 
                startPath(paths.Path9); 
                break;
        }
    }

    private void startPath(PathChain path) {
        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
        }
        if (!follower.isBusy()) {
            pathStarted = false;
            pathState++;
        }
    }

    private void runPath4WithIntake() {
        if (!pathStarted) {
            follower.followPath(paths.Path4);
            intakeMotor.setPower(-1);
            pathStarted = true;
        }

        // Wait simulation for intake
        if (!follower.isBusy()) {
            intakeMotor.setPower(1);
            sleepMillis(Wait1);
            intakeMotor.setPower(0);
            pathStarted = false;
            pathState++;
        }
    }

    private void runPath7WithIntake() {
        if (!pathStarted) {
            follower.followPath(paths.Path7);
            intakeMotor.setPower(-1);
            pathStarted = true;
        }

        // Sequence: -1 -> +1 -> -1
        if (!follower.isBusy()) {
            intakeMotor.setPower(1);
            sleepMillis(Wait2);
            intakeMotor.setPower(-1);
            sleepMillis(Wait1);
            intakeMotor.setPower(0);
            pathStarted = false;
            pathState++;
        }
    }

    private void runShooter() {
        if (detectedTagId == 21) { // GPP
            outtakeMotorLeft.setPower(1);
            sleepMillis(Wait1);
            outtakeMotorLeft.setPower(0);
            outtakeMotorRight.setPower(1);
            sleepMillis(Wait2);
            outtakeMotorRight.setPower(0);
        } else if (detectedTagId == 22) { // PGP
            outtakeMotorRight.setPower(1);
            sleepMillis(Wait1);
            outtakeMotorRight.setPower(0);
            outtakeMotorLeft.setPower(1);
            sleepMillis(Wait1);
            outtakeMotorLeft.setPower(0);
            outtakeMotorRight.setPower(1);
            sleepMillis(Wait1);
            outtakeMotorRight.setPower(0);
        } else if (detectedTagId == 23) { // PPG
            outtakeMotorRight.setPower(1);
            sleepMillis(Wait2);
            outtakeMotorRight.setPower(0);
            outtakeMotorLeft.setPower(1);
            sleepMillis(Wait1);
            outtakeMotorLeft.setPower(0);
        } else { //fallback
            outtakeMotorLeft.setPower(1);
            outtakeMotorRight.setPower(1);
            sleepMillis(Wait2);
            outtakeMotorLeft.setPower(0);
            outtakeMotorRight.setPower(0);
        }
        
        pathState++;
    }

    private void runVision() {
        if (visionPortal == null) {
            aprilTag = new AprilTagProcessor.Builder().build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
            visionStartTime = System.currentTimeMillis();
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (!detections.isEmpty()) {
            detectedTagId = detections.get(0).id;
            visionPortal.close();
            visionPortal = null;
            pathState++;
        } else if (System.currentTimeMillis() - visionStartTime > VISION_TIMEOUT_MS) {
            visionPortal.close();
            visionPortal = null;
            pathState++;
        }
    }

    private void sleepMillis(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(123.046, 122.315), new Pose(112.528, 111.289)))
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(45))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(112.528, 111.289), new Pose(71.914, 119.904)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(71.914, 119.904), new Pose(80.579, 80.551), new Pose(102.066, 83.543)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(102.066, 83.543), new Pose(125.208, 83.447)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(125.208, 83.447), new Pose(93.477, 93.322), new Pose(112.426, 111.482)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(112.426, 111.482), new Pose(75.553, 55.457), new Pose(101.198, 59.228)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(101.198, 59.228), new Pose(125.193, 59.381)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(125.193, 59.381), new Pose(85.744, 77.688), new Pose(112.416, 111.254)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(112.416, 111.254), new Pose(120.579, 94.731)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(270))
                    .build();
        }
    }
}
