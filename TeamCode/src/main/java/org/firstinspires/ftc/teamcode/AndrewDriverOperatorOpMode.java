package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//Gyro References
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="", group="Tele-Op")

public class AndrewDriverOperatorOpMode extends LinearOpMode{
    //Initializes the robot hardware variables
    ArmHardwareOmni robot = new ArmHardwareOmni();
    public void runOpMode() {
        //Retrieves the mappings from runtime
        robot.init(hardwareMap);
        double power = 0.3;

        boolean sensitivityControl = false;

        //Drive train vector
        double left_y, left_x;
        //Rotation
        double left_t, right_t;
        //Rotated drive train vectors
        double abs_x, abs_y;


        //Sensor variables
        //Gyroscope angle
        double g_angle;



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
        robot.latch.setPosition(1.0);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //While loop for robot operation
        while (opModeIsActive()) {

            //Sensitivity Control
            if(gamepad2.x){
                sensitivityControl = true;
            }
            else if(gamepad2.y){
                sensitivityControl = false;
            }

            //Forklift
            if(sensitivityControl) {
                robot.forklift.setPower(0.3 * gamepad2.left_stick_y);
            }
            else {
                robot.forklift.setPower(0.8 * gamepad2.left_stick_y);
            }

            //Tape Extensions
            if(gamepad2.right_trigger > 0) {
                robot.leftExtension.setPower(0.8 * gamepad2.right_trigger);
                robot.rightExtension.setPower(0.8 * gamepad2.right_trigger);
            }
            else{
                robot.leftExtension.setPower(0.8 * gamepad2.left_trigger);
                robot.rightExtension.setPower(0.8 * gamepad2.left_trigger);
            }

            if (gamepad1.x) {
                robot.mascot.setPosition(0);
            } else {
                robot.mascot.setPosition(1);
            }

            if (gamepad2.a) {
                robot.latch.setPosition(0);
            } else if(gamepad2.b){
                robot.latch.setPosition(1.0);
            }

            /*
            //Light switch control for color sensor
            bCurrState = gamepad1.b;
            //Checks for a different state
            if (bCurrState != bPrevState) {
                //Checks if the button is enabled
                if (bCurrState) {
                    //Checks if the robot has a switchable light
                    if (robot.colorSensor instanceof SwitchableLight) {
                        //Toggles light
                        SwitchableLight light = (SwitchableLight) robot.colorSensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }

            //Updates bPrevState
            bPrevState = bCurrState;
            */

            //Whipper switch


            if (gamepad1.a) {
                robot.vacuum.setPower(-1); //suck
            } else if (gamepad1.b) {
                robot.vacuum.setPower(1); //spit
            } else{
                robot.vacuum.setPower(0);
            }

            //Vertical Extension
            if(gamepad1.dpad_up){
                robot.verticalLift.setPower(0.8);
            }
            else if(gamepad1.dpad_down){
                robot.verticalLift.setPower(-0.5);
            }
            else{
                robot.verticalLift.setPower(0);
            }


            //Chariot Lift
            //robot.motor7.setPower(0.8*gamepad1.right_stick_y);


            //Gamepad's left stick x and y values
            //Inverts y's sign
            left_y = -gamepad1.left_stick_y;
            //Receives x
            left_x = gamepad1.left_stick_x;

            //Gamepad's left and right trigger values
            left_t = gamepad1.left_trigger;
            right_t = gamepad1.right_trigger;

            //Robot Heading Unit Vector

            //Boolean for distance reset
            g_angle = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addLine()
                    .addData("Angle:", g_angle);
            g_angle *= Math.PI / 180;
            abs_x = (left_x * Math.cos(-g_angle) - left_y * Math.sin(-g_angle));
            abs_y = (left_x * Math.sin(-g_angle) + left_y * Math.cos(-g_angle));

            telemetry.addLine()
                    .addData("Left Stick X:", left_x)
                    .addData("Left Stick Y:", left_y);
            //Power variable (0,1), average drive train motor speed

            //x component vector
            //motor 2
            robot.frontDrive.setPower(power * (-abs_x + left_t - right_t));
            //motor4
            robot.backDrive.setPower(power * (abs_x + left_t - right_t));

            //y vector
            //motor1
            robot.leftDrive.setPower(power * (abs_y + left_t - right_t));
            //motor3
            robot.rightDrive.setPower(power * (-abs_y + left_t - right_t));
            telemetry.update();
        }
    }
}
