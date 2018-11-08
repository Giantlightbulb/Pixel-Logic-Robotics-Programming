package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Sensors
//   Gyro
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
//  ODS
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
//  Color Sensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

//Range Sensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

//Compass Sensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;

//Gyro References
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//Android App Control
import android.app.Activity;
import android.graphics.Color;
import android.view.View;


public class HardwareOmni {
    /* local OpMode members. */
    HardwareMap hwMap =  null;
    public ElapsedTime timer  = new ElapsedTime();

    //Initializes hardware variables
    //Drive train motors
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;

    //Extension Motor
    public DcMotor motor5;
    //Telescoping lift motor
    public DcMotor motor6;

    //Chariot Motors
    //Arm
    public DcMotor motor7;
    //Whipper - Defunct
    //public DcMotor motor8;

    //Servos
    public Servo servo1;
    public Servo servo2;
    public CRServo servo3;

    //Initializes Sensors
    //  Gyro
    public IntegratingGyroscope gyro;
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    //Axes
    public AxesReference aRefInt = AxesReference.INTRINSIC;
    public AxesOrder aOrderXYZ = AxesOrder.XYZ;
    public AngleUnit aUnit = AngleUnit.DEGREES;
    NormalizedRGBA colors = new NormalizedRGBA();
    //  ODS
    public OpticalDistanceSensor ods;
    //  Color Sensor
    NormalizedColorSensor colorSensor;
    //  Compass Sensor
    ModernRoboticsI2cCompassSensor compass;
    //  Range Sensor
    ModernRoboticsI2cRangeSensor rangeSensor;
    //  Initializes layout
    View relativeLayout;

    //Constructor (Currently empty)
    public HardwareOmni() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        //Hardware definitions
        //Drivetrain motors
        motor1 = hwMap.get(DcMotor.class,"motor1");
        motor2 = hwMap.get(DcMotor.class,"motor2");
        motor3 = hwMap.get(DcMotor.class,"motor3");
        motor4 = hwMap.get(DcMotor.class,"motor4");
        //Vacuum motor
        motor5 = hwMap.get(DcMotor.class, "motor5");
        //Telescoping lift motor
        motor6 = hwMap.get(DcMotor.class, "motor6");
        //Chariot Motors
        motor7 = hwMap.get(DcMotor.class, "motor7");
        //motor8 = hwMap.get(DcMotor.class, "motor8");

        //Servos
        servo1 = hwMap.get(Servo.class, "servo1");
        servo2 = hwMap.get(Servo.class, "servo2");
        servo3 = hwMap.get(CRServo.class, "servo3");

        //  Sensors
        //  Gyro
        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        //  ODS
        ods = hwMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);
        //  Color Sensor
        colorSensor = hwMap.get(NormalizedColorSensor.class, "color_sensor");
        //      Detects switchable light
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        //  Compass
        compass = hwMap.get(ModernRoboticsI2cCompassSensor.class, "compass");
        //  Range Sensor
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        //  Application Color
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
        relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);

        //  Set all motors to zero power
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        motor5.setPower(0);
        motor6.setPower(0);
        motor7.setPower(0);
        //motor8.setPower(0);

    }

}


