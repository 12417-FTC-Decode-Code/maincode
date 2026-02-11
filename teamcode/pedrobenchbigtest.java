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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Goal Pedro Test Mode", group = "Test")
@Configurable
public class pedrobenchbigtest extends OpMode {

    // --- TEST MODE CONFIGURATION ---
    public static final boolean IS_TEST_MODE = true;
    // 0.1 means the robot moves 10% of the real distance.
    // If the robot moves too little, try 0.3 or 0.5
    public static final double SCALE = IS_TEST_MODE ? 0.2 : 1.0;
    // -------------------------------

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private DcMotorEx intakeMotor, transferMotor, outtakeMotorLeft, outtakeMotorRight;
    private Servo rampServo, lever;

    private int pathState = 0;

    // Timer variables
    private long actionStartTime = 0;
    private int shooterSubState = 0;
    private int intakeSubState = 0;
    private boolean pathStarted = false;

    // Timing constants
    private static final long WAIT_1000 = 1000;
    private static final long WAIT_1500 = 1500;
    private static final long WAIT_2000 = 2000;

    @Override
    public void init() {
        try { panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry(); } catch (Exception e) {}

        follower = Constants.createFollower(hardwareMap);

        // Scale the starting position
        // Real Start: 123.046, 122.315
        double startX = 123.046 * SCALE;
        double startY = 122.315 * SCALE;

        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(40)));

        paths = new Paths(follower, SCALE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_m");
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outake_l");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outake_r");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_m");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rampServo = hardwareMap.get(Servo.class, "ramp_servo");
        lever = hardwareMap.get(Servo.class, "lever_servo");

        lever.setPosition(0);
        rampServo.setPosition(1);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("TEST MODE", IS_TEST_MODE);
        telemetry.addData("Scale Factor", SCALE);
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start Path 1
                if(startPath(paths.Path1)) setPathState(1);
                break;
            case 1: // Run Shooter Action
                if (runShooter()) setPathState(2);
                break;
            case 2: // Start Path 2
                if(startPath(paths.Path2)) setPathState(3);
                break;
            case 3: // Start Path 3
                if(startPath(paths.Path3)) setPathState(4);
                break;
            case 4: // Run Path 4 with Intake logic
                runPath4WithIntake();
                break;
            case 5: // Start Path 5
                if(startPath(paths.Path5)) setPathState(6);
                break;
            case 6: // Run Shooter Action
                if (runShooter()) setPathState(7);
                break;
            case 7: // Start Path 6
                if(startPath(paths.Path6)) setPathState(8);
                break;
            case 8: // Run Path 7 with Intake logic
                runPath7WithIntake();
                break;
            case 9: // Start Path 8
                if(startPath(paths.Path8)) setPathState(10);
                break;
            case 10: // Run Shooter Action
                if (runShooter()) setPathState(11);
                break;
            case 11: // Start Path 9 (Park)
                if(startPath(paths.Path9)) setPathState(12);
                break;
            case 12: // Done
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        actionStartTime = System.currentTimeMillis();
        shooterSubState = 0;
        intakeSubState = 0;
        pathStarted = false;
    }

    private boolean startPath(PathChain path) {
        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
            return false;
        }
        if (!follower.isBusy()) {
            return true;
        }
        return false;
    }

    private boolean runShooter() {
        switch (shooterSubState) {
            case 0:
                outtakeMotorLeft.setPower(0.7);
                outtakeMotorRight.setPower(0.7);
                actionStartTime = System.currentTimeMillis();
                shooterSubState = 1;
                break;
            case 1:
                if (System.currentTimeMillis() - actionStartTime >= WAIT_2000) {
                    lever.setPosition(1);
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1500) {
                    lever.setPosition(0);
                    intakeMotor.setPower(1);
                    transferMotor.setPower(1);
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 3;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1000) {
                    intakeMotor.setPower(0);
                    transferMotor.setPower(0);
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 4;
                }
                break;
            case 4:
                if (System.currentTimeMillis() - actionStartTime >= WAIT_2000) {
                    lever.setPosition(1);
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 5;
                }
                break;
            case 5:
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1500) {
                    lever.setPosition(0);
                    outtakeMotorLeft.setPower(0);
                    outtakeMotorRight.setPower(0);
                    return true;
                }
                break;
        }
        return false;
    }

    private void runPath4WithIntake() {
        switch (intakeSubState) {
            case 0:
                if (!pathStarted) {
                    follower.followPath(paths.Path4);
                    intakeMotor.setPower(-1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    actionStartTime = System.currentTimeMillis();
                    intakeSubState = 1;
                }
                break;
            case 1:
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1000) {
                    intakeMotor.setPower(0);
                    setPathState(5);
                }
                break;
        }
    }

    private void runPath7WithIntake() {
        switch (intakeSubState) {
            case 0:
                if (!pathStarted) {
                    follower.followPath(paths.Path7);
                    intakeMotor.setPower(-1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    actionStartTime = System.currentTimeMillis();
                    intakeSubState = 1;
                }
                break;
            case 1:
                if (System.currentTimeMillis() - actionStartTime >= WAIT_2000) {
                    intakeMotor.setPower(-1);
                    actionStartTime = System.currentTimeMillis();
                    intakeSubState = 2;
                }
                break;
            case 2:
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1000) {
                    intakeMotor.setPower(0);
                    setPathState(9);
                }
                break;
        }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower, double s) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(123.046 * s, 122.315 * s), new Pose(112.528 * s, 111.289 * s)))
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(225))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(112.528 * s, 111.289 * s), new Pose(71.914 * s, 119.904 * s)))
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(71.914 * s, 119.904 * s), new Pose(80.579 * s, 80.551 * s), new Pose(102.066 * s, 83.543 * s)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(102.066 * s, 83.543 * s), new Pose(125.208 * s, 83.447 * s)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(125.208 * s, 83.447 * s), new Pose(93.477 * s, 93.322 * s), new Pose(112.426 * s, 111.482 * s)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(112.426 * s, 111.482 * s), new Pose(75.553 * s, 55.457 * s), new Pose(101.198 * s, 59.228 * s)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(101.198 * s, 59.228 * s), new Pose(125.193 * s, 59.381 * s)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(125.193 * s, 59.381 * s), new Pose(85.744 * s, 77.688 * s), new Pose(112.416 * s, 111.254 * s)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(112.416 * s, 111.254 * s), new Pose(120.579 * s, 94.731 * s)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(270))
                    .build();
        }
    }
}