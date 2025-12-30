package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "BLUE GOAL PEDRO", group = "Autonomous")
@Configurable
public class PedroAutoBlueGoalTemp extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    private DcMotorEx intakeMotor;
    private DcMotorEx outtakeMotorLeft;
    private DcMotorEx outtakeMotorRight;
    private DcMotorEc transferMotor;

    private int pathState = 0;
    private boolean pathStarted = false;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int detectedTagId = -1;
    private long visionStartTime;
    private static final long VISION_TIMEOUT_MS = 1000;

    private int intakeState = 0;
    private long intakeTimer = 0;
    private int shooterState = 0;
    private long shooterTimer = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(-55, -55, Math.toRadians(45))); //CHANGE 

        paths = new Paths(follower);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outtake_motor_left");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outtake_motor_right");

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("Tag", detectedTagId);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0: 
startPath(paths.Path1); 
break;

            case 1: 
runShooter(); 
pathState++;
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
            intakeState = 0;
            pathStarted = true;
        }

        runPath4Intake();

        if (!follower.isBusy()) {
            intakeMotor.setPower(0);
            pathStarted = false;
            pathState++;
        }
    }

    private void runPath4Intake() {
        switch (intakeState) {
            case 0:
                intakeMotor.setPower(-1);
                intakeTimer = System.currentTimeMillis();
                intakeState++;
                break;
            case 1:
                if (System.currentTimeMillis() - intakeTimer > 5000) {
                    intakeMotor.setPower(1);
                    intakeTimer = System.currentTimeMillis();
                    intakeState++;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - intakeTimer > 1000) {
                    intakeMotor.setPower(0);
                    intakeState++;
                }
                break;
        }
    }

    private void runPath7WithIntake() {
        if (!pathStarted) {
            follower.followPath(paths.Path7);
            intakeState = 0;
            pathStarted = true;
        }

        runPath7Intake();

        if (!follower.isBusy()) {
            intakeMotor.setPower(0);
            pathStarted = false;
            pathState++;
        }
    }

    private void runPath7Intake() {
        switch (intakeState) {
            case 0:
                intakeMotor.setPower(-1);
                intakeTimer = System.currentTimeMillis();
                intakeState++;
                break;
            case 1:
                if (System.currentTimeMillis() - intakeTimer > 2000) {
                    intakeMotor.setPower(1);
                    intakeTimer = System.currentTimeMillis();
                    intakeState++;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - intakeTimer > 2000) {
                    intakeMotor.setPower(-1);
                    intakeTimer = System.currentTimeMillis();
                    intakeState++;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - intakeTimer > 1000) {
                    intakeMotor.setPower(0);
                    intakeState++;
                }
                break;
        }
    }

    private void runShooter() {
        if (detectedTagId == 21) {
            switch (shooterState) {
                case 0:
                    outtakeMotorLeft.setPower(1);
                    shooterTimer = System.currentTimeMillis();
                    shooterState++;
                    break;
                case 1:
                    if (System.currentTimeMillis() - shooterTimer > 1000) {
                        outtakeMotorLeft.setPower(0);
                        outtakeMotorRight.setPower(1);
                        shooterTimer = System.currentTimeMillis();
                        shooterState++;
                    }
                    break;
                case 2:
                    if (System.currentTimeMillis() - shooterTimer > 2000) {
                        outtakeMotorRight.setPower(0);
                        shooterState = 0;
                        pathState++;
                    }
                    break;
            }
        } else {
            outtakeMotorLeft.setPower(1);
            outtakeMotorRight.setPower(1);
            shooterTimer = System.currentTimeMillis();
            pathState++;
        }
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

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower) {
            Path1 = line(follower, 20,122, 29.726,113.299, 145,135);
            Path2 = line(follower, 29.726,113.299, 70.416,119.391, 135,180);
            Path3 = line(follower, 70.416,119.391, 42.396,84.305, 180,180);
            Path4 = line(follower, 42.396,84.305, 18.518,83.817, 180,180);
            Path5 = line(follower, 18.518,83.817, 30.213,113.299, 180,135);
            Path6 = line(follower, 30.213,113.299, 44.102,59.939, 135,180);
            Path7 = line(follower, 44.102,59.939, 19.249,59.939, 180,180);
            Path8 = line(follower, 19.249,59.939, 30.213,113.299, 180,135);
            Path9 = line(follower, 30.213,113.299, 19.980,71.147, 135,90);
        }

        private static PathChain line(Follower f, double x1, double y1, double x2, double y2, double h1, double h2) {
            return f.pathBuilder()
                    .addPath(new BezierLine(new Pose(x1,y1), new Pose(x2,y2)))
                    .setLinearHeadingInterpolation(Math.toRadians(h1), Math.toRadians(h2))
                    .build();
        }
    }
}
