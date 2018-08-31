package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
//Gyro
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

@TeleOp(name="Double Math King Gyro", group="Basic OP Mode")

public class VProjControl extends LinearOpMode{
    //Initializes hardware
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    public ElapsedTime timer = new ElapsedTime();
    public void runOpMode(){
        double power = 0.1;
        //Telemetry initialized message
        telemetry.addData(  "Status",   "Initialized");
        telemetry.update();

        //Hardware definitions
        motor1 = hardwareMap.get(DcMotor.class,"motor1");
        motor2 = hardwareMap.get(DcMotor.class,"motor2");
        motor3 = hardwareMap.get(DcMotor.class,"motor3");
        motor4 = hardwareMap.get(DcMotor.class,"motor4");

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();


        //Wait until phone interrupt
        waitForStart();
        //While loop for robot operation
        while (opModeIsActive()){
            //Gamepad's left stick x and y values
            double left_y = -gamepad1.left_stick_y;
            double left_x = gamepad1.left_stick_x;

            //Gamepad's left and right trigger values
            double left_t = gamepad1.left_trigger;
            double right_t = gamepad1.right_trigger;

            //Robot Heading Unit Vector

            //Boolean for distance reset
            double g_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double []Robot_y = {Math.cos(g_angle), Math.sin(g_angle)};
            double []Robot_x = {Robot_y[1], -Robot_y[0]};
            //Magnitude is already 1.
            //double Robot_yMag = Math.sqrt(Robot_y[0]*Robot_y[0]+Robot_y[1]*Robot_y[1]);
            //double Robot_xMag = Math.sqrt(Robot_x[0]*Robot_x[0]+Robot_x[1]*Robot_x[1]);;
            double compy = (left_x*Robot_y[0]+left_y*Robot_y[1]);
            double compx = (left_x*Robot_x[1]+left_y*Robot_x[1]);

            //Power variable (0,1), average drive train motor speed

            //x component vector
            //motor 2
            motor2.setPower(power*(compx+left_t-right_t));
            //motor4
            motor4.setPower(power*(compx+left_t-right_t));

            //y vector
            //motor1
            motor1.setPower(power*(compy+left_t-right_t));
            //motor3
            motor3.setPower(power*(compy+left_t-right_t));

            //More telemetry. Adds left stick values and trigger values
            telemetry.addLine()
                    .addData("right_y", left_y)
                    .addData("left_x", left_x );
            telemetry.addLine()
                    .addData("left trigger", left_t)
                    .addData("right trigger", right_t);
            telemetry.addLine()
                    .addData("angle", g_angle);

            telemetry.update();
        }
    }
}
