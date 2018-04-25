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
	
	static double integralThreshold;
	static double errorTotal;
	static double lastError = 0;
	static double proportional;
	static double integral;
	static double derivative;
	static double targetDistance = 50; // cm
	
	//Constants
	static double kp = 1;
	static double ki = 0;
	static double kd = 0;
	
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
        
        

        // while(System.currentTimeMillis < end){
        	// try{
        		// int secondsToSleep = 10;
        		// Thread.sleep(secondsToSleep * 1000);

        	// }
        	// catch(InterruptedException ex){
        		// System.out.println("thread error");
        	// }
        	// setDistance();
        // }
        
        while(true) {
            long t = System.currentTimeMillis();
            long tenSec = 10000;
            long end = t + tenSec;
            setDistance();

            while(System.currentTimeMillis() < end) {
                double error = (sensor.getDistance() * 100) - targetDistance; 


                if(error != 0){
                    errorTotal += error;
                }
                else{
                    errorTotal = 0;
                }

                //Cap the integral from growing too large
                // if(errorTotal > 50/ki){
                // 	errorTotal = 50/ki;
                // 

                if(error == 0){
                    derivative = 0;
                }

                proportional = Math.abs(error) * kp;
                integral = errorTotal * ki;
                derivative = (error - lastError) * kd;

                lastError = error;
                double power = (proportional + integral + derivative) * 10;
                System.out.println("Error: "+error);
                System.out.println("Distance: "+sensor.getDistance());

                leftMotor.setSpeed((int)power);
                rightMotor.setSpeed((int)power);

                if(error > 0){
                    leftMotor.forward();
                    rightMotor.forward();
                }
                else{
                    leftMotor.backward();
                    rightMotor.backward();
                }
                try{
                    Thread.sleep(10000);

                }
                catch(InterruptedException ex){
                    System.out.println("thread error");
                }

            
            
            }
        }
        // }


		

		// //Disconnect from the Robot
  //       myRobot.close();
		

	}


}
