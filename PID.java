 // Error = measured distanced - actuall distance
// proportional = error * kp

// Every time through loop add errorTotal += error 
// Intergral = errorTotal * ki

// Derivate = rate of change of error 
// Everytime through loop lastError = error (at botttom of loop)
 // Derivative = error - lastError * kd

//integralActive == set to value (Don't start accumalting error until above this threshold

//Cap of integral term from growing too large with a limit value -> (50/ki)

//Power sent to motor = proportional + Integral + Derivative

//Have a wait time at the end of the loop
import ShefRobot.*;
import java.util.concurrent.TimeUnit;

public class PID {
	

	static double maxSpeed = 350;
	static double errorTotal;
	static double lastError = 0;
	static double proportional;
	static double integral;
	static double derivative;
	static double targetDistance = 50; // cm
	
	//Constants
	static double kp = 8.76;
	static double ki = 0;
	static double kd = 1.46;
	
	private static void setDistance(){
		if(targetDistance == 50){
			targetDistance = 30;
		}
		else{
			targetDistance = 50;
		}
	}
	
	public static void main(String[] args) {
		
		//Create a robot object to use and connect to it
        Robot myRobot = new Robot("dia-lego-a1");
       
        //The robot is made of components which are themselves objects.
        //Create references to them as useful shortcuts
        Motor leftMotor = myRobot.getLargeMotor(Motor.Port.B);
        Motor rightMotor = myRobot.getLargeMotor(Motor.Port.C);
        UltrasonicSensor sensor = myRobot.getUltrasonicSensor(Sensor.Port.S2);
        
        
        while(true) {
            long t = System.currentTimeMillis();
            long tenSec = 10000;
            long finish = t + 60000;
            long end = t + tenSec;
            setDistance();
            

            if(t >= finish ){
                break;
            }
            if(t >= finish){
                myRobot.close();
                break;
            }
            
            while(true) {
                myRobot.sleep(50);
                double error = (sensor.getDistance() * 100) - targetDistance; 
                if(System.currentTimeMillis() >= end){
                    break;
                }

                if(error != 0){
                    errorTotal += error;
                }
                else{
                    errorTotal = 0;
                }


                if(error == 0){
                    derivative = 0;
                }

                proportional = error * kp;
                integral = errorTotal * ki;
                derivative = (error - lastError) * kd;

                lastError = error;
                double speed = Math.abs((proportional + integral + derivative) * 10);
                System.out.println("Error: "+error);
                System.out.println("Distance: "+sensor.getDistance());

                if(speed >= maxSpeed){
                    speed = maxSpeed;
                }

                leftMotor.setSpeed((int)speed);
                rightMotor.setSpeed((int)speed);

                if(error > 0){
                    leftMotor.forward();
                    rightMotor.forward();
                }
                else{
                    leftMotor.backward();
                    rightMotor.backward();
                }
            
            
            }
        }
        // }


		

		// //Disconnect from the Robot
  //       myRobot.close();
		

	}


}
