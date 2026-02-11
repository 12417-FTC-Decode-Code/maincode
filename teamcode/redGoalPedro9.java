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

@Autonomous(name = "Red Goal Pedro 9", group = "Autonomous")
@Configurable
public class redGoalPedro9 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx outtakeMotorLeft;
    private DcMotorEx outtakeMotorRight;
    private Servo rampServo;
    private Servo lever;

    private int pathState = 0;
    private boolean pathStarted = false;

    // Timer variables for non-blocking waits
    private long actionStartTime = 0;
    private int shooterSubState = 0;
    private int intakeSubState = 0;

    // Timing constants
    private static final long WAIT_1000 = 1000;
    private static final long WAIT_1500 = 1500;
    private static final long WAIT_2000 = 2000;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize follower with CORRECT starting pose matching Path1
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123.046, 122.315, Math.toRadians(40)));

        paths = new Paths(follower);

        // Initialize motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_m");
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outake_l");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outake_r");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_m");

        // Set motor brake behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servos
        rampServo = hardwareMap.get(Servo.class, "ramp_servo");
        lever = hardwareMap.get(Servo.class, "lever_servo");

        // Set initial servo positions
        lever.setPosition(0);
        rampServo.setPosition(1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Update telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
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
                startPath(paths.Path3);
                break;
            case 4:
                runPath4WithIntake();
                break;
            case 5:
                startPath(paths.Path5);
                break;
            case 6:
                runShooter();
                break;
            case 7:
                startPath(paths.Path6);
                break;
            case 8:
                runPath7WithIntake();
                break;
            case 9:
                startPath(paths.Path8);
                break;
            case 10:
                runShooter();
                break;
            case 11:
                startPath(paths.Path9);
                break;
            case 12:
                // Autonomous complete
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

    // NON-BLOCKING shooter sequence using state machine
    private void runShooter() {
        switch (shooterSubState) {
            case 0: // Start first outtake motors
                outtakeMotorLeft.setPower(0.7);
                outtakeMotorRight.setPower(0.7);
                actionStartTime = System.currentTimeMillis();
                shooterSubState = 1;
                break;

            case 1: // Wait for motors to spin up
                if (System.currentTimeMillis() - actionStartTime >= WAIT_2000) {
                    lever.setPosition(1);
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 2;
                }
                break;

            case 2: // Wait for servo to move
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1500) {
                    lever.setPosition(0);
                    intakeMotor.setPower(1);
                    transferMotor.setPower(-1);
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 3;
                }
                break;

            case 3: // Wait for transfer
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1000) {
                    intakeMotor.setPower(0);
                    transferMotor.setPower(0);
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 4;
                }
                break;

            case 4: // Wait before second shot
                if (System.currentTimeMillis() - actionStartTime >= WAIT_2000) {
                    lever.setPosition(1);
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 5;
                }
                break;

            case 5: // Final servo wait
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1500) {
                    lever.setPosition(0);
                    outtakeMotorLeft.setPower(0);
                    outtakeMotorRight.setPower(0);
                    shooterSubState = 0; // Reset for next use
                    pathState++;
                }
                break;
        }
    }

    // NON-BLOCKING intake with Path4
    private void runPath4WithIntake() {
        switch (intakeSubState) {
            case 0: // Start path and intake
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

            case 1: // Reverse intake briefly
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1000) {
                    intakeMotor.setPower(0);
                    pathStarted = false;
                    intakeSubState = 0; // Reset for next use
                    pathState++;
                }
                break;
        }
    }

    // NON-BLOCKING intake with Path7
    private void runPath7WithIntake() {
        switch (intakeSubState) {
            case 0: // Start path and intake
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

            case 1: // Reverse intake
                if (System.currentTimeMillis() - actionStartTime >= WAIT_2000) {
                    intakeMotor.setPower(-1);
                    actionStartTime = System.currentTimeMillis();
                    intakeSubState = 2;
                }
                break;

            case 2: // Final intake run
                if (System.currentTimeMillis() - actionStartTime >= WAIT_1000) {
                    intakeMotor.setPower(0);
                    pathStarted = false;
                    intakeSubState = 0; // Reset for next use
                    pathState++;
                }
                break;
        }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(123.046, 122.315), new Pose(112.528, 111.289)))
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(225))
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