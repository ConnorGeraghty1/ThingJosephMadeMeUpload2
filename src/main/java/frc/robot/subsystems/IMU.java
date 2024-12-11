package frc.robot.subsystems;

import java.lang.Math;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class IMU extends SubsystemBase {
  
  public ADIS16470_IMU imu;

  private double accelerationXa; // = 0
  private double accelerationXb; // = 0

  private double velocityXa; // = 0
  private double velocityXb; // = 0
  
  private double positionX; // = 0

  private double accelerationYa; // = 0
  private double accelerationYb; // = 0

  private double velocityYa; // = 0
  private double velocityYb; // = 0
  
  private double positionY; // = 0

  public IMU(){

    imu = Constants.imu;
    
    accelerationXa = 0;
    accelerationXb = 0;
    accelerationYa = 0;
    accelerationYb = 0;
    velocityXa = 0;
    velocityXb = 0;
    velocityYa = 0;
    velocityYb = 0;
    positionX = 0;
    positionX = 0;
    
  }
  
  public void updateMotion(){
    
    updateAccelerationX();
    updateAccelerationY();
    updateVelocityX();
    updateVelocityY();
    updatePositionX();
    updatePositionY();
    
  }


  //X motion methods
  public void updateAccelerationX(){
    accelerationXa = accelerationXb;
    accelerationXb = imu.getAccelX() * Math.cos(Math.toRadians(imu.getAngle())); //field-centric AccelX
  }

  public void updateVelocityX(){
    velocityXa = velocityXb;
    velocityXb += ((accelerationXa+accelerationXb)/2)*0.02; //riemann sum of acceleration for change in V
  }

  public void updatePositionX(){
    positionX += ((velocityXa + velocityXb)/2)*0.02; //riemann sum of velocity for change in P
  }


  //Y motion methods
  public void updateAccelerationY(){
    accelerationYa = accelerationYb;
    accelerationYb = imu.getAccelY() * Math.cos(Math.toRadians(imu.getAngle())); 
  }

  public void updateVelocityY(){
    velocityYa = velocityYb;
    velocityYb += ((accelerationYa+accelerationYb)/2)*0.02;
  }

  public void updatePositionY(){
    positionY += ((velocityYa + velocityYb)/2)*0.02;
  }

  //getMotion methods

  public double getVelocityX(){
    return velocityXb;
  }
  public double getVelocityY(){
    return velocityYb;
  }
  public double getPositionX(){
    return positionX;
  }
  public double getPositionY(){
    return positionY;
  }
  
  
  

  public void reset() {
    imu.reset();
    accelerationXa = 0;
    accelerationXb = 0;
    accelerationYa = 0;
    accelerationYb = 0;
    velocityXa = 0;
    velocityXb = 0;
    velocityYa = 0;
    velocityYb = 0;
    positionX = 0;
    positionX = 0;
  }

  public double yaw(){
    return imu.getAngle();
  }
  

}
