package frc.robot;



import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
public class Robot extends TimedRobot {
 
  Joystick stick = new Joystick(0);

  private static final CANSparkMax left1 = new CANSparkMax(1, MotorType.kBrushless);
  private static final CANSparkMax left2 = new CANSparkMax(2, MotorType.kBrushless);
  private static final CANSparkMax left3 = new CANSparkMax(3, MotorType.kBrushless);
  private static final CANSparkMax right1 = new CANSparkMax(4, MotorType.kBrushless);
  private static final CANSparkMax right2 = new CANSparkMax(5, MotorType.kBrushless);
  private static final CANSparkMax right3 = new CANSparkMax(6, MotorType.kBrushless);
   

  
  
  @Override
  public void robotInit() {
    left2.follow(left1);
    left3.follow(left1);
    
    right2.follow(right1);
    right3.follow(right1); 
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
   
    double stickX = -Math.pow(stick.getRawAxis(4),3);
    double stickY = Math.pow(stick.getRawAxis(1),3);
    stickX *= .75;

    // Joystick deadband
    if(Math.abs(stickX) < .000124){
        stickX = 0;
    }
    if(Math.abs(stickY) < .000124){
        stickY = 0;
    }

    if(stick.getRawButton(5) == true){

      left1.setSmartCurrentLimit​(99999);
      left2.setSmartCurrentLimit​(99999);
      left3.setSmartCurrentLimit​(99999);
      right1.setSmartCurrentLimit​(99999);
      right2.setSmartCurrentLimit​(99999);
      right3.setSmartCurrentLimit​(99999);
    }

    else{

      left1.setSmartCurrentLimit​(30);
      left2.setSmartCurrentLimit​(30);
      left3.setSmartCurrentLimit​(30);
      right1.setSmartCurrentLimit​(30);
      right2.setSmartCurrentLimit​(30);
      right3.setSmartCurrentLimit​(30);

    }
    runAt((stickY + stickX), -(stickY - stickX));
    
    // Arcade drive
    
  

    
  

  }
  @Override
  public void testPeriodic() {
  }

  public static void runAt(double leftSpeed, double rightSpeed) {
        
    left1.set(leftSpeed);
    right1.set(rightSpeed);
  }
}