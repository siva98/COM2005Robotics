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

public class MazeBot {
	

	static double maxSpeed = 500;
	static double errorTotal;
	static double lastError = 0;
	static double proportional;
	static double integral;
	static double derivative;
	static double targetDistance = 5; // cm
    static int defaultSpeed = 100;
	
	//Constants
	static double kp = 8.76;
	static double ki = 13.17;
	static double kd = 1.46;
	

	
	public static void main(String[] args) {
		
		//Create a robot object to use and connect to it
        Robot myRobot = new Robot("dia-lego-a1");
       
        //The robot is made of components which are themselves objects.
        //Create references to them as useful shortcuts
        Motor leftMotor = myRobot.getLargeMotor(Motor.Port.B);
        Motor rightMotor = myRobot.getLargeMotor(Motor.Port.C);
        UltrasonicSensor sensorL = myRobot.getUltrasonicSensor(Sensor.Port.S3);
        UltrasonicSensor sensorR = myRobot.getUltrasonicSensor(Sensor.Port.S2);
        
        
            
        while(true) {
            double sensorTarget = (sensorL.getDistance() + sensorR.getDistance())/2;
            double error = (sensorL.getDistance() * 100) - sensorTarget; 
            
            

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
            double motorSpeed = speed / 1.1;

            System.out.println("Error: "+error);
            System.out.println("DISTANCE LEFT: "+sensorL.getDistance());
            System.out.println("DISTANCE RIGHT: "+sensorR.getDistance());

            if(speed >= maxSpeed){
                speed = maxSpeed;
            }

            
            if(error == 0){
                leftMotor.setSpeed(defaultSpeed);
                rightMotor.setSpeed(defaultSpeed);
                leftMotor.forward();
                rightMotor.forward();
                System.println("go")

            }
              
            //Turn Left
            if(error > 0){
                leftMotor.setSpeed(defaultSpeed);
                rightMotor.setSpeed((int)motorSpeed);
                leftMotor.forward();
                rightMotor.forward();
                System.println("leftTurn")
            }
            //Turn Right
            else{
                rightMotor.setSpeed(defaultSpeed);
                leftMotor.setSpeed((int)motorSpeed);
                leftMotor.forward();
                rightMotor.forward();
                System.println("leftTurn")

            }
        
        
        }
    


		

		// //Disconnect from the Robot
  //       myRobot.close();
		

	}


}