package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//Gyro References
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="", group="Tele-Op")

public class MatthewDriverOperatorOpMode extends LinearOpMode{
    //Initializes the robot hardware variables
    ArmHardwareOmni robot = new ArmHardwareOmni();
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
        robot.init(hardwareMap);
        double sensitivity = 0.3;
        double power = 0.2;

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
        //Ensures the latch is clear
        robot.latch.setPosition(1.0);
        //While loop for robot operation
        while (opModeIsActive()) {
            //Driver
            //Drive train
            //Gamepad's left stick x and y values
            //Inverts y's sign

            //Operator
            //Extension
            //Arm
            //Vertical Lift
            left_y = -gamepad1.left_stick_y;
            //Receives x
            left_x = gamepad1.left_stick_x;
            abs_x = left_x;
            abs_y = left_y;
            //Gamepad's left and right trigger values
            left_t = gamepad1.left_trigger;
            right_t = gamepad1.right_trigger;

            /*
            //Robot Heading Unit Vector
            g_angle = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addLine()
                    .addData("Angle:", g_angle);
            g_angle *= Math.PI / 180;
            abs_x = (left_x * Math.cos(-g_angle) - left_y * Math.sin(-g_angle));
            abs_y = (left_x * Math.sin(-g_angle) + left_y * Math.cos(-g_angle));
            */

            //Power variable (0,1), average drive train motor speed
            //x component of the direction vector
            //Handles left and right motion
            robot.frontDrive.setPower(power * (abs_x + left_t - right_t));
            robot.backDrive.setPower(power * (abs_x + left_t - right_t));

            //y component of the direction vector
            //Handles forwards and backwards motion
            robot.leftDrive.setPower(power * (abs_y + left_t - right_t));
            robot.rightDrive.setPower(power * (-abs_y + left_t - right_t));

            //Vacuum Toggle
            if (gamepad1.dpad_up) {
                robot.vacuum.setPower(-1);
            } else if (gamepad1.dpad_down) {
                robot.vacuum.setPower(1);
            } else {
                robot.vacuum.setPower(0);
            }

            //Drop Mascot
            if (gamepad1.a) {
                robot.mascot.setPosition(1);
            } else {
                robot.mascot.setPosition(0);
            }

            //Operator
            //Arm
            robot.arm.setPower(-power*gamepad2.left_stick_y);

            //Extension
            robot.leftTape.setPower(power*gamepad2.right_stick_x);
            robot.rightTape.setPower(-power*gamepad2.right_stick_x);

            //Vertical Lift
            if (gamepad2.dpad_up) {
                robot.verticalLift.setPower(0.8);
            } else if (gamepad2.dpad_down) {
                robot.verticalLift.setPower(-0.5);
            } else {
                robot.verticalLift.setPower(0);
            }


            //Latch
            if (gamepad1.left_bumper)
            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;

            //All telemetry
            telemetry.addLine()
                    .addData("Left Stick X:", left_x)
                    .addData("Left Stick Y:", left_y);
            telemetry.update();
        }
    }
}
