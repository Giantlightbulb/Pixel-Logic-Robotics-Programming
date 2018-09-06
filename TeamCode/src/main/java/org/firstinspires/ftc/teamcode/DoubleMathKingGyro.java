package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
//Gyro
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.GyroSensor;

//@TeleOp(name="Double Math King Gyro", group="Basic OP Mode")

public class DoubleMathKingGyro extends LinearOpMode{
    //Initializes motor variables
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    private ElapsedTime timer = new ElapsedTime();
    private double sum_error = 0;
    private double delta_error = 0;

    private double PID(double y, double r, long deltat){
        double u = 0;
        double kp = 0.6;
        double ki = 0.4;
        double kd = 0.75;
        double e = y - r;
        delta_error = (e - delta_error)/deltat;
        sum_error += delta_error;
        u = kp*e + ki*sum_error + kd*delta_error;
        //u(t) = K_p * e(t) + K_i * integral(0, t)(e(t')dt) + K_d * d/dt(e(t))
        return u;
    }

    private static double sigmoid(long time,
                                  boolean derivative,
                                  boolean integral,
                                  boolean inverse,
                                  double a, double b, double c){
        //Sigmoid function is NOT log base (*)
        double fzero = Math.log((b/a)/(1-(b/a)));
        double y = a/(1+Math.exp(-c*time-fzero));
        if (integral){
            if (inverse){
                y = (Math.log(Math.exp((c*time)/a)-1)-fzero)/c;
            } else {
                y = a*Math.log(1+Math.exp(c*time+fzero))/c;
            }
        } else if (derivative){
            if (inverse){
               //Not accounted for
            } else {
                y = c*(y*(1-y/a));
            }
        } else {
            if (inverse){
                y = (Math.log(time/(a-time))-fzero)/c;
            }
        }
        return y;
    }


    public void runOpMode(){
        double power = 0.1;
        //Telemetry initialized message
        telemetry.addData(  "Status",   "Initialized");
        telemetry.update();

        //Hardware definitions
        //Motors
        motor1 = hardwareMap.get(DcMotor.class,"motor1");
        motor2 = hardwareMap.get(DcMotor.class,"motor2");
        motor3 = hardwareMap.get(DcMotor.class,"motor3");
        motor4 = hardwareMap.get(DcMotor.class,"motor4");
        //Gyro
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

        //Motor List
        DcMotor[] motors = {motor1, motor3, motor2, motor4};

        //Base angle/Zero position angle
        double base_angle = 0;

        //Array zero position
        int base = 0;

        //Wait until phone interrupt
        waitForStart();
        //While loop for robot operation
        while (opModeIsActive()){
            //Game Pad's left stick x and y values
            double left_y = -gamepad1.left_stick_y;
            double left_x = gamepad1.left_stick_x;

            //Game Pad's left and right trigger values
            double left_t = gamepad1.left_trigger;
            double right_t = gamepad1.right_trigger;

            //Boolean for distance reset
            double g_angle = Math.abs(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            if ((Math.abs(g_angle-base_angle)-90) > 0){
                base_angle = g_angle;
                base += 2;
            }

            //Power variable (0,1), average drive train motor speed
            //long deltat = (timer.nanoseconds()-base_time)/timer.SECOND_IN_NANO;
                /*
                power = sigmoid((timer.nanoseconds()- base_time)/timer.SECOND_IN_NANO,
                        false, false, false, 0.5, 0.1, 2);
                */
            //PID(power, 0, deltat);
                /*
                The motors are paired and power based on being the x or y component
                of the vector.

                The - on the left_x or left_y ensures that the "paired" motors run in
                tandem.

                The difference left_t-right_t calculates the delta between the right
                and left triggers. They are not multiplied as the motors are supposed
                to run in a circle.

                The sum of the left_(x/y) and the trigger difference allows for movement
                on the x y plane with added rotation. Think drifting.

                All of this is multiplied by the power variable allowing fine power
                control.
                */
            /*
            The motors are paired and power based on being the x or y component
            of the vector.

            The - on the left_x or left_y ensures that the "paired" motors run in
            tandem.

            The difference left_t-right_t calculates the delta between the right
            and left triggers. They are not multiplied as the motors are supposed
            to run in a circle.

            The sum of the left_(x/y) and the trigger difference allows for movement
            on the x y plane with added rotation. Think drifting.

            All of this is multiplied by the power variable allowing fine power
            control.
            */

            //x component vector
            //motor 2
            motors[(2+base)%4].setPower(power*(-left_x+left_t-right_t));
            //motor4
            motors[(3+base)%4].setPower(power*(left_x+left_t-right_t));

            //y vector
            //motor1
            motors[(base)%4].setPower(power*(left_y+left_t-right_t));
            //motor3
            motors[(1+base)%4].setPower(power*(-left_y+left_t-right_t));

            //More telemetry. Adds left stick values and trigger values
            telemetry.addLine()
                    .addData("right_y", left_y)
                    .addData("left_x", left_x );
            telemetry.addLine()
                    .addData("left trigger", left_t)
                    .addData("right trigger", right_t);
            telemetry.addLine()
                    .addData("angle", g_angle);
            telemetry.addLine()
                    .addData("List position", base);

            telemetry.update();
        }
    }
}
