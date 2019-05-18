package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//Gyro References
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="GunOperatorOpMode", group="Tele-Op")

public class GunOperatorOpMode extends LinearOpMode{
    //Initializes the robot hardware variables
    OmniGunHardware robot = new OmniGunHardware();
    public void runOpMode() {


        /*
        Driver:
        -Drivetrain
        -Mascot
        -Latch
        -Vaccuum

        Killer:
        -extension
        -arm
        -vertical lift
         */
        //Retrieves the mappings from runtime
        robot.init(this, hardwareMap, telemetry);

        double sensitivity = 0.3;
        double power = 0.4;

        //Drive train vector
        double left_y, left_x;
        //Rotation
        double left_t, right_t;
        //Absolute heading vector
        double abs_x, abs_y;

        //Sensor variables
        //Gyroscope angle
        double g_angle;

        //Whipper booleans
        boolean upPrev = false;
        boolean downPrev = false;

        //Telemetry initialized message
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        //Gyro calibrating
        robot.modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(robot.timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        //Finalized gyro calibration.
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
        //double servoPosition = 0.0;
        //Wait until op-mode start
        waitForStart();
        //Resets timer
        robot.timer.reset();
        //While loop for robot operation
        while (opModeIsActive()) {
            //Drive Train Control
            left_y = -gamepad1.left_stick_y;
            //Receives x
            left_x = gamepad1.left_stick_x;
            abs_x = left_x;
            abs_y = left_y;
            //Gamepad's left and right trigger values
            left_t = gamepad1.left_trigger;
            right_t = gamepad1.right_trigger;

            //Power variable (0,1), average drive train motor speed
            //x component of the direction vector
            //Handles left and right motion
            robot.frontDrive.setPower(0.55 * (abs_x - left_t + right_t));
            robot.backDrive.setPower(0.55 * (abs_x + left_t - right_t));

            //y component of the direction vector
            //Handles forwards and backwards motion
            robot.leftDrive.setPower(0.55 * (abs_y - left_t + right_t));
            robot.rightDrive.setPower(0.55 * (abs_y + left_t - right_t));

            //gunAxisMotor
            robot.gunAxisMotor.setPower(-0.25*gamepad1.right_stick_y);

            if (gamepad1.b) {
                robot.acceleratingMotor.setPower(1);
            }

            //All telemetry
            telemetry.addLine()
                    .addData("Power Sensitivity:", (power+sensitivity));
            telemetry.addLine()
                    .addData("D-Pad Up", gamepad2.dpad_up);
            telemetry.addLine()
                    .addData("D-Pad Down", gamepad2.dpad_down);
            telemetry.update();
        }
    }
}
