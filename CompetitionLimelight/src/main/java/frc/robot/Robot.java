
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
 
  private static final double CAMERA_HEIGHT = 0;       //height of camera lens from the ground.
  private static final double TARGET_HEIGHT = 0;       //height of target from the ground to the very top of the target
  private static final double TRIANGLE_HEIGHT = TARGET_HEIGHT - CAMERA_HEIGHT;   //the difference between the camera height and the target height (used to calculate distance)
  private static final double CAMERA_ANGLE = 0;        //the angle the camera is on when mounted on the robot
  private static final double MAXINUM_DISTANCE = 0;    //the maxinum distance the robot can be from the target to track
  private static final int A_BUTTON = 1;               //the id value of the A button
  private final int LEFT_Y_AXIS = 1;                  //the left vertical axis of the joystick
  private final int RIGHT_X_AXIS = 4;                 //the right horizontal axis of the joystick
  private final double JOYSTICK_DEADBAND = 0.000124;  //deadband of the joystick, 
  private final double DEFAULT_LIMELIGHT_VALUE = 0.01234;  // default value that the limelight will return if no targets are found
  private final double DEFAULT_HORIZONTAL_SPEED = 0.25; // default speed for robot rotation
  private final double HORIZONTAL_ANGLE_DEADBAND = 0.2; 

  private static double distanceFromTarget = 0.0;  //the distance the camera is from the target, horizontally
  private static double verticalAngle = 0.0;       //ty of the camera. the angle between the camera perpendicularly and the target
  private static double horizontalAngle = 0.0;     //t of the camera

  // values of the left joystick's x-axis and the right joystick's y-axis
  private static double stickX = 0.0;
  private static double stickY = 0.0;

  //Movement of each side of wheels
  private static double leftMovement;
  private static double rightMovement;

  // Stores the previous horizontal angle
  private static double prevHorizAngle = 0.0;

  // speed of the robot in the left and right directions
  private static double horizontalSpeed = 0.0;
  private static double leftHorizSpeed = 0.0;
  private static double rightHorizSpeed = 0.0;

  // speed of the robot in the forward and backward directions
  private static double distanceSpeed = 0.0;

  //network table of the limelight, stores all the values the limelight has
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  // Spark motor controllers
  CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftFollow1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax leftFollow2 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightFollow1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rightFollow2 = new CANSparkMax(6, MotorType.kBrushless);
  
  // xbox controller used for controlling the robot
  Joystick stick = new Joystick(0);

  @Override
  public void robotInit() {
    
    /* set the limelight and USB camera to picture-in-picture mode,
       which means the limelight's camera feed is shown in the
       bottom right corner of the USB camera's feed
    */
    limelightTable.getEntry("stream").setNumber(3);

    // configure 2 motors on each side of the robot to follow the main motor on each side
    leftFollow1.follow(leftMotor);
    rightFollow1.follow(rightMotor);
    leftFollow2.follow(leftMotor);
    rightFollow2.follow(rightMotor);
  }

 
  @Override
  public void robotPeriodic() {

  }

 
  @Override
  public void autonomousInit() {
    
  }


  @Override
  public void autonomousPeriodic() {
    
  }


  @Override
  public void teleopPeriodic() {
    
    // reset the horizontal and distance speeds every time the code runs
    // this will prevent previous leftover values from moving the motors
    distanceSpeed = 0.0;
    horizontalSpeed = 0.0;
    leftHorizSpeed = 0.0;
    rightHorizSpeed = 0.0;

    //get tx
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    NetworkTableEntry tv = limelightTable.getEntry("tv");
    double horizontalAngle = tx.getDouble(DEFAULT_LIMELIGHT_VALUE);
    double isTargetVisible = tv.getDouble(-1.0);
    System.out.println("HORIZONTAL : " + horizontalAngle);
    System.out.println("Is target visible: " + isTargetVisible);
      
    //turns led on camera on when the A button is down
    limelightTable.getEntry("ledMode").setNumber(3);

    // Cubing values to create smoother function
    stickX = -Math.pow(stick.getRawAxis(LEFT_Y_AXIS), 3);
    stickY = Math.pow(stick.getRawAxis(RIGHT_X_AXIS), 3);

    // Stop robot from turning too fast


    // Checks if a target is in view
    if(isTargetVisible == 1)
    {
      // If target is in view, the previous speed becomes the current horizontal speed
      // Previous angle holds the last angle recieved after losing the target
      prevHorizAngle = horizontalAngle;
    }

    // Checks if the a button is down
    if(stick.getRawButton(A_BUTTON)) {
      
      // Checks to see if the current horizontal angle (tx) is outside the deadband
      if(Math.abs(horizontalAngle) > .5)

        // Makes the horizontal speed adjust in proportion to the current horizontal angle
        // Divides current horizontal angle by the maximum angle possible 
        // Multiplies the angle by the default horizontal speed
        horizontalSpeed = (horizontalAngle / 27.0) *  DEFAULT_HORIZONTAL_SPEED;
    
      // This is for rotating it right 
      // Otherwise rotate left
      if(isTargetVisible == 0) {
        if(Math.abs(prevHorizAngle) > HORIZONTAL_ANGLE_DEADBAND + 0.8)
        {
          horizontalSpeed = (prevHorizAngle / 27.0) * DEFAULT_HORIZONTAL_SPEED;
        }
      }
    }

    // manual drive controls
    if(Math.abs(stickX) > JOYSTICK_DEADBAND) {
      distanceSpeed = stickX;
      System.out.println("Moving X");
    }
    if(Math.abs(stickY) > JOYSTICK_DEADBAND) {
      horizontalSpeed = stickY;
    }

    // left and right speeds of the drivetrain
    leftMovement = (distanceSpeed + horizontalSpeed);// + leftHorizSpeed;// * (horizontalAngle / 27);
    rightMovement = (distanceSpeed - horizontalSpeed);// - rightHorizSpeed;// * (horizontalAngle / 27);

    System.out.println("Left: " + leftMovement + " = DIstance Speed: " + distanceSpeed + " + leftHorizSpeed: " + leftHorizSpeed);
    System.out.println("Right: " + rightMovement + " = DIstance Speed: " + distanceSpeed + " + rightHorizSpeed: " + rightHorizSpeed);
    // run the robot based on the left and right speeds of the drive train
    runAt(-leftMovement, rightMovement);
  }


  @Override
  public void testPeriodic() {
  }

  private void runAt(double leftSpeed, double rightSpeed) {      
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }
}
