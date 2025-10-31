import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "auto", group = "Autonomous")
public class auto extends LinearOpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();    
        if (opModeIsActive()) {
            // Drive forward for 1 second
            leftFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightFront.setPower(0.5);
            rightBack.setPower(0.5);

            sleep(1800);   

            // Stop motors
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }
}