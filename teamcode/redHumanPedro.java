package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Human Pedro UPDATED", group = "Autonomous")
@Configurable
public class redHumanPedro extends OpMode {

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
    private static final long WAIT_launch = 1000;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize follower with CORRECT starting pose matching Path1 Start
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(86.706, 8.000, Math.toRadians(90)));

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
        rampServo.setPosition(0);

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
            case 0: // 1. Run Path 1 + INTAKE ONLY
                runPathWithIntakeOnly(paths.Path1, false);
                break;
            case 1: // 2. Run Outtake mechanism
                runShooter();
                break;
            case 2: // 3. Run Path 2 (Move only)
                startPath(paths.Path2, false);
                break;
            case 3: // 4. Run Path 3 + SLOW + INTAKE & TRANSFER
                runPathWithFullIntake(paths.Path3, true);
                break;
            case 4: // 5. Run Path 4 + INTAKE ONLY
                runPathWithIntakeOnly(paths.Path4, false);
                break;
            case 5: // 6. Run Outtake mechanism
                runShooter();
                break;
            case 6: // 7. Run Path 5 (Move only)
                startPath(paths.Path5, false);
                break;
            case 7: // 8. Run Path 6 + INTAKE & TRANSFER
                runPathWithFullIntake(paths.Path6, false);
                break;
            case 8: // 9. Run Path 7 + INTAKE & TRANSFER
                runPathWithFullIntake(paths.Path7, false);
                break;
            case 9: // 10. Run Path 8 (Move only)
                startPath(paths.Path8, false);
                break;
            case 10: // 11. Run Outtake mechanism
                runShooter();
                break;
            case 11: // 12. Run Path 9 (Move only)
                startPath(paths.Path9, false);
                break;
            case 12: // 13. Done
                lever.setPosition(0);
                break;
        }
    }

    // Standard path execution with optional slow down
    private void startPath(PathChain path, boolean slow) {
        if (!pathStarted) {
            if (slow) {
                follower.setMaxPower(0.4); // Reduce max power to 40%
            } else {
                follower.setMaxPower(1.0); // Reset to full speed
            }
            follower.followPath(path);
            pathStarted = true;
        }
        if (!follower.isBusy()) {
            follower.setMaxPower(1.0); // Always reset when done
            pathStarted = false;
            pathState++;
        }
    }

    // Standard NON-BLOCKING shooter sequence
    private void runShooter() {
        switch (shooterSubState) {
            case 0: // Start flywheel motors
                outtakeMotorLeft.setPower(0.61);
                outtakeMotorRight.setPower(-0.61);
                actionStartTime = System.currentTimeMillis();
                shooterSubState = 1;
                break;

            case 1: // Wait for motors to spin up
                if (System.currentTimeMillis() - actionStartTime >= WAIT_launch) {
                    lever.setPosition(1); // Load
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 2;
                }
                break;

            case 2: // Wait for servo to move
                if (System.currentTimeMillis() - actionStartTime >= 750) {
                    lever.setPosition(0); // Reset lever
                    intakeMotor.setPower(1); // Ensure everything feeds
                    transferMotor.setPower(-0.7);
                    outtakeMotorLeft.setPower(0.61);
                    outtakeMotorRight.setPower(-0.61);
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 4;
                }
                break;

            case 4: // Wait before second shot (if applicable) or finish
                if (System.currentTimeMillis() - actionStartTime >= WAIT_launch) {
                    lever.setPosition(1); // Fire again?
                    actionStartTime = System.currentTimeMillis();
                    shooterSubState = 5;
                }
                break;

            case 5: // Final cleanup
                if (System.currentTimeMillis() - actionStartTime >= 500) {
                    lever.setPosition(0);
                    outtakeMotorLeft.setPower(0);
                    outtakeMotorRight.setPower(0);
                    shooterSubState = 0; // Reset for next use
                    pathState++;
                }
                break;
        }
    }

    // Runs Path AND Intake AND Transfer simultaneously
    private void runPathWithFullIntake(PathChain path, boolean slow) {
        switch (intakeSubState) {
            case 0:
                if (!pathStarted) {
                    if (slow) follower.setMaxPower(0.4);
                    follower.followPath(path);
                    intakeMotor.setPower(1);
                    transferMotor.setPower(-0.7);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    actionStartTime = System.currentTimeMillis();
                    intakeSubState = 1;
                }
                break;

            case 1:
                if (System.currentTimeMillis() - actionStartTime >= 500) {
                    follower.setMaxPower(1.0); // Reset speed
                    intakeMotor.setPower(1); // Stop both
                    transferMotor.setPower(0);
                    pathStarted = false;
                    intakeSubState = 0;
                    pathState++;
                }
                break;
        }
    }

    // Runs Path AND Intake ONLY (No Transfer)
    private void runPathWithIntakeOnly(PathChain path, boolean slow) {
        switch (intakeSubState) {
            case 0:
                if (!pathStarted) {
                    if (slow) follower.setMaxPower(0.4);
                    follower.followPath(path);
                    intakeMotor.setPower(1); // INTAKE ONLY
                    transferMotor.setPower(1); // Ensure transfer is off
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    actionStartTime = System.currentTimeMillis();
                    intakeSubState = 1;
                }
                break;

            case 1:
                if (System.currentTimeMillis() - actionStartTime >= 500) {
                    follower.setMaxPower(1.0);
                    intakeMotor.setPower(1); // Turn intake off
                    pathStarted = false;
                    intakeSubState = 0;
                    pathState++;
                }
                break;
        }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.706, 8.000),
                                    new Pose(83.529, 13.235)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(247))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.529, 13.235),
                                    new Pose(96.206, 35.441)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(247), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.206, 35.441),
                                    new Pose(119.824, 35.353)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.824, 35.353),
                                    new Pose(83.559, 13.441)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(247))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.559, 13.441),
                                    new Pose(120.324, 15.059)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(247), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.324, 15.059),
                                    new Pose(135.294, 15.206)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.294, 15.206),
                                    new Pose(134.853, 11.882)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.853, 11.882),
                                    new Pose(83.706, 13.059)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(247))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.706, 13.059),
                                    new Pose(110.382, 13.529)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(247), Math.toRadians(0))
                    .build();
        }
    }
}