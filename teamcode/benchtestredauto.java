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

@Autonomous(name = "Pedro Auto Test", group = "Autonomous")
@Configurable
public class benchtestredauto extends OpMode {

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
    private int actionSubState = 0;

    // Timing constants
    private static final long WAIT_SPINUP = 500;
    private static final long WAIT_DUMP = 1000;
    private static final long WAIT_RESET = 500;

    private static final long WAIT_SHOOTING = 3000;

    // Poses for easy adjustment
    private final Pose startPose = new Pose(123.046, 122.315, Math.toRadians(225));
    private final Pose scorePose = new Pose(112.528, 111.289, Math.toRadians(225));
    private final Pose alignPose = new Pose(110.0, 111.289, Math.toRadians(0)); // Moves back slightly, turns to 0
    private final Pose grabPose  = new Pose(118.0, 111.289, Math.toRadians(0)); // Drives forward to grab

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


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
        rampServo.setPosition(0.5); // Assuming 0 is "UP" or default

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Update telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Sub State", actionSubState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Path 1: Go to Score Position
                startPath(paths.Path1);
                break;

            case 1: // Action: Run Outtake & Servo
                runScoringSequence();
                break;

            case 2: // Path 2: Straight couple inches & Heading 0
                startPath(paths.Path2);
                break;

            case 3: // Path 3: Run Intake/Transfer & Move to Grab
                if(pathState == 3 && !pathStarted) {
                    // Start intake before moving
                    intakeMotor.setPower(1);
                    transferMotor.setPower(1);
                }
                startPath(paths.Path3);
                break;

            case 4: // Path 4: Return to Score Position
                // Stop intake before returning
                if(pathState == 4 && !pathStarted) {
                    intakeMotor.setPower(0);
                    transferMotor.setPower(0);
                }
                startPath(paths.Path4);
                break;

            case 5: // Action: Run Outtake & Servo (Repeat of step 1)
                runScoringSequence();
                break;

            case 6: // Done
                requestOpModeStop();
                break;
        }
    }

    private void startPath(PathChain path) {
        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
        }
        // If follower is done with path, move to next state
        if (!follower.isBusy()) {
            pathStarted = false;
            pathState++;
        }
    }

    // NON-BLOCKING Scoring Sequence
    private void runScoringSequence() {
        switch (actionSubState) {
            case 0: // Start Outtake Motors
                outtakeMotorLeft.setPower(0.7);
                outtakeMotorRight.setPower(0.7);
                actionStartTime = System.currentTimeMillis();
                actionSubState = 1;
                break;

            case 1: // Wait for Spin Up
                if (System.currentTimeMillis() - actionStartTime >= WAIT_SPINUP) {
                    rampServo.setPosition(0.5); // Place servo down at 0.5
                    actionStartTime = System.currentTimeMillis();
                    actionSubState = 2;
                }
                break;

            case 2: // Wait for Dump
                if (System.currentTimeMillis() - actionStartTime >= WAIT_DUMP) {
                    rampServo.setPosition(0); // Reset servo (assuming 0 is reset)
                    outtakeMotorLeft.setPower(0);
                    outtakeMotorRight.setPower(0);
                    actionStartTime = System.currentTimeMillis();
                    actionSubState = 3;
                }
                break;

            case 3: // Sequence Complete
                if (System.currentTimeMillis() - actionStartTime >= WAIT_RESET) {
                    actionSubState = 0; // Reset for next time
                    pathState++;       // Move main state machine forward
                }
                break;
        }
    }

    public class Paths {
        public PathChain Path1, Path2, Path3, Path4;

        public Paths(Follower follower) {
            // Path 1: Start -> Score
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();

            // Path 2: Score -> Align (Straight back/side, turn to Heading 0)
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, alignPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), alignPose.getHeading())
                    .build();

            // Path 3: Align -> Grab (Drive straight forward to grab sample)
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(alignPose, grabPose))
                    .setLinearHeadingInterpolation(alignPose.getHeading(), grabPose.getHeading())
                    .build();

            // Path 4: Grab -> Score (Return to Path 1 location)
            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(grabPose, scorePose))
                    .setLinearHeadingInterpolation(grabPose.getHeading(), scorePose.getHeading())
                    .build();
        }
    }
}