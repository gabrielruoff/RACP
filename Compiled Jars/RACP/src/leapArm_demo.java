import com.leapmotion.leap.Arm;
import com.leapmotion.leap.Bone;
import com.leapmotion.leap.Controller;
import com.leapmotion.leap.Finger;
import com.leapmotion.leap.FingerList;
import com.leapmotion.leap.Frame;
import com.leapmotion.leap.Gesture;
import com.leapmotion.leap.Hand;
import com.leapmotion.leap.HandList;
import com.leapmotion.leap.Listener;
import com.leapmotion.leap.Vector;

import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;


class SampleListenerMain extends Listener{
	
	//define position lock booleans
	public boolean leftLock = false;
	public boolean rightLock = false;
	
	public static int oldZRotationPos;
	public static int oldShoulderAngle;
	public static int oldElbowAngle;
	public static int oldWristAngle;
	public static int oldClawPos;
	public static int oldClawRotPos;
	
	public static double gestureIndex = 0;
	
	//Displacement variables
	double deltaX, deltaY, deltaZ, angle;
	
	//initialize Kalman Filters
	public static KalmanFilter shoulderKFilter = new KalmanFilter();
	public static KalmanFilter elbowKFilter = new KalmanFilter();
	
	public Joint zRot = new Joint(90);
	public Joint shoulder = new Joint(90);
	public Joint elbow = new Joint(90);
	public Joint wrist = new Joint(90);
	public Joint clawRot = new Joint(90);
	public Joint claw = new Joint(90);
	
	Joint[] joints = {zRot, shoulder, elbow, wrist, clawRot, claw};
	
	robotArm arm = new robotArm(joints, "/dev/cu.usbmodem14201");

	public void onInit(Controller controller) {
		//System.out.println("Initialized");

		}

		public void onConnect(Controller controller) {
		//System.out.println("Connected");
		controller.enableGesture(Gesture.Type.TYPE_CIRCLE);
		controller.enableGesture(Gesture.Type.TYPE_KEY_TAP);
		controller.config().setFloat("Gesture.KeyTap.MinDownVelocity", 40.0f);
		controller.config().setFloat("Gesture.KeyTap.HistorySeconds", .2f);
		controller.config().setFloat("Gesture.KeyTap.MinDistance", 1.0f);
		controller.config().save();

		}

		public void onDisconnect(Controller controller) {
		//System.out.println("Disconnected");
		}

		public void onExit(Controller controller) {
		//System.out.println("Exited");
		}
		
		public void onFrame(Controller controller) {
			
			long startTime = System.nanoTime();
			
			//Define position variables
			double shoulderAngle, elbowAngle, clawPos, zRotationPos, wristAngle, clawRotPos;
			
			//Define object variables
			//Frame
			Frame frame = controller.frame();
			//Hands
			Hand leftHand = frame.hands().leftmost();
			Hand rightHand = frame.hands().rightmost();
			//Arms
			Arm leftArm = leftHand.arm();
			Arm rightArm = rightHand.arm();
			//Fingers
			FingerList fingerList = rightHand.fingers().fingerType(Finger.Type.TYPE_INDEX);
			Finger rightIndexFinger = fingerList.get(0); 
			
			//Right Thumb
			fingerList = rightHand.fingers().fingerType(Finger.Type.TYPE_THUMB);
			Finger rightThumb = fingerList.get(0);
			
			//find the distance between the bones to detect pinch
			Vector rightIndexDistal = rightIndexFinger.bone(Bone.Type.TYPE_DISTAL).center();
			Vector rightThumbDistal = rightThumb.bone(Bone.Type.TYPE_DISTAL).center();
			
			/* Control of robotic arm with Z-rotation based on the left hand, arm 'wrist' position based on the wrist,
			 * arm 'elbow position based on the elbow, and claw based on the fingers. 'Shoulder' is based on the left elbow
			 */
			
			//Control position locks for left hand controls and right hand controls
			//Gesture gesture = new Gesture(gesture);
			for(Gesture gesture : frame.gestures()) {
				
				
				HandList handsForGesture = gesture.hands();
				
				//Sleep for 20ms to let gesture be recognized
				try {
					Thread.sleep(1);
				} catch (InterruptedException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				
				//print handList
				for(Hand hand : handsForGesture) {
					
					if(hand.equals(rightHand)) {
						
						System.out.println("Right Hand");
						
					} else if(hand.equals(leftHand)) {
						
						System.out.println("Left Hand");
						
					}
					
				}
				//sleep
				try {
					Thread.sleep(100);
				} catch (InterruptedException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				
				switch(gesture.type()) {
				
				case TYPE_KEY_TAP:
					
					System.out.println("Key tap from" + handsForGesture + " Hand");
					
					for(Hand hand : handsForGesture) {
						
						if(hand.equals(rightHand)) {
							
							rightLock = !rightLock;
							
						} else if(hand.equals(leftHand)) {
							
							leftLock = !leftLock;
							
						}
						
					}
					
					break;
					
				default:
					
					System.out.println("Unrecognized gesture");
					
					break;
				
				}
				
				break;
				
			}
			
			//Lock left controls if left lock is on
			if(!leftLock) {
			
				//'Shoulder' control. MIRRORS LEFT ELBOW ANGLE
				//find angle between the left elbow and the left wrist center
				Vector leftElbow = leftArm.elbowPosition();
				Vector leftWrist = leftArm.wristPosition();
				
				deltaZ = leftElbow.getZ() - leftWrist.getZ();
				deltaY = leftElbow.getY() - leftWrist.getY();
				
				angle = Math.atan(deltaY/deltaZ);
				
				//map angle so servo can understand it
				shoulderAngle = leapArm_demo.map(angle, (0.1), (-0.8), 0, 180);
				
				//Set Kalman Filter sensor input to angle
				if(shoulderAngle<=0 || Double.isNaN(shoulderAngle)) {
					
				shoulderKFilter.Zk.setEntry(0, 0, 0);
					
				} else {
			
				System.out.println("Yk_prev: " + shoulderKFilter.Yk_prev);
				shoulderKFilter.Zk.setEntry(0, 0, shoulderAngle);
				
				}
				
				//Run Kalman Filter
				shoulderAngle = leapArm_demo.filterInput(shoulderKFilter);
				
				//Write position to 'shoulder'
				
				//Z-rotation control
				Vector leftHandPos = leftHand.palmPosition();
				//rotate z-axis with speed proportional to left hand X position
				//map X position to motor power
				//bounded 50/230 so hands don't knock into each other
				zRotationPos = leapArm_demo.map(leftHandPos.getX(), -230, 50, 0, 180);
				oldZRotationPos = (int) zRotationPos;
				
				zRot.setPosition(zRotationPos);;
				
				//write shoulder value to data
				if(Double.isNaN(shoulderAngle) || shoulderAngle <= 0) {
					shoulder.setPosition(0);
				} else {
					shoulder.setPosition(shoulderAngle);
				}
			
			//'elbow' control. MIRRORS LEFT WRIST ANGLE
			//find angle between the left wrist and palm
			leftWrist = leftArm.wristPosition();
			
			//refresh deltas and angle
			deltaZ = leftWrist.getZ() - leftHand.palmPosition().getZ();
			deltaY = leftWrist.getY() - leftHand.palmPosition().getY();
			
			angle = Math.atan(deltaY/deltaZ);
			
			//map angle so the servo can understand it
			elbowAngle = leapArm_demo.map(angle, (-1), 0.5, 0, 180);
			
			//Set Kalman Filter sensor input to angle
			if(elbowAngle<=0 || Double.isNaN(elbowAngle)) {
				
			elbowKFilter.Zk.setEntry(0, 0, 0);
				
			} else {

			//System.out.println("Yk_prev: " + elbowKFilter.Yk_prev);
			elbowKFilter.Zk.setEntry(0, 0, elbowAngle);
			
			}
			
			//Run Kalman Filter
			elbowAngle = leapArm_demo.filterInput(elbowKFilter);
			
			//Write elbow value to data
			//data += leapArm.linearApproximation(elbowAngle, oldElbowAngle, 2);
			//System.out.println("Returned: " + data);
			if(Double.isNaN(elbowAngle) || elbowAngle <= 0) {
				elbow.setPosition(0);
			} else {
				elbow.setPosition(elbowAngle);
			}
			
			//Send the old value if left lock is on
			} else {
			
				//null
			
			}
			
			//'wrist' control
			//update vectors
			Vector rightElbow = rightArm.elbowPosition();
			Vector rightWrist = rightArm.wristPosition();
			
			Vector rightHandPos = rightHand.palmPosition();
			
			//update deltas
			deltaZ = rightWrist.getZ() - rightHandPos.getZ();
			deltaY = rightWrist.getY() - rightHandPos.getY();
			
			
			////System.out.println("Wrist pos: " + rightWrist.getX() + ", " + rightWrist.getY() + ", " + rightWrist.getZ());
			////System.out.println("Right hand pos: " + rightHandPos.getX() + ", " + rightHandPos.getY() + ", " + rightHandPos.getZ());
			
			angle = Math.atan(deltaY/deltaZ);
			
			////System.out.println("Wrist angle: " + angle);
			
			//map value
			wristAngle = leapArm_demo.map(angle, 0.4, (-1.2), 0, 180);
			//Write wrist value to data
			//data+=leapArm.linearApproximation(wristAngle, oldWristAngle, 3);
			if(Double.isNaN(wristAngle) || wristAngle <= 0) {
				wrist.setPosition(0);
			} else {
				wrist.setPosition(wristAngle);
			}
			
			//Claw rotation position
			//bounded 50/230 so hands don't knock into each other
			////System.out.println("Right hand x: " + rightHandPos.getX());
			clawRotPos = leapArm_demo.map(rightHandPos.getX(), 80, 170, 0, 180);
			//Write clawrot value to data
			//data+=leapArm.linearApproximation(clawRotPos, oldClawRotPos, 4);
			if(Double.isNaN(clawRotPos) || clawRotPos <= 0) {
				clawRot.setPosition(0);
			} else {
				clawRot.setPosition(clawRotPos);
			}
			
			
			
			//System.out.println("wristAngle: " + wristAngle + " degrees");
			
			//pinch control
			
			//Calculate distance between joints
			double distalDistance = Math.sqrt(Math.pow((rightIndexDistal.getX()-rightThumbDistal.getX()),2) + Math.pow((rightIndexDistal.getY()-rightThumbDistal.getY()),2) + Math.pow((rightIndexDistal.getZ()-rightThumbDistal.getZ()),2));
			////System.out.println("Distal Distance: " + distalDistance);
			if(distalDistance >= 50) {
				
				clawPos = 0;
				
			} else {
				
				clawPos = 90;
				
			}
			
			oldClawPos = (int) clawPos;

				claw.setPosition(clawPos);
			
			/* wait for Arduino to catch up to avoid buffer overflow. Arduino can handle ~30 packets/sec (19bytes each) **need to update for new packet length
			 * basically see how long the Arduino takes to process one packet and flush the receiving arrays to prevent array 'pollution'.
			 */
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		}
		
}
		
public class leapArm_demo implements SerialPortEventListener {

			public static double map(double input, double in_min, double in_max, double out_min, double out_max) {
				
				return ((input - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
				
			}
			
			public static int filterInput(KalmanFilter k) {
				
				k.predict(k.Yk, k.A, k.Yk_prev, k.Pk, k.Pk_prev, k.Q);
				k.update(k.K, k.Pk, k.Pk_prev, k.H, k.R, k.YkHat, k.YkHat_prev, k.Zk, k.I);
				
				return (int) k.getOutput();
				
			}
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub

		// Create a sample listener and controller
		SampleListenerMain listener = new SampleListenerMain();
		Controller controller = new Controller();

		// Have the sample listener receive events from the controller
		controller.addListener(listener);
		// Keep this process running until Enter is pressed
		
		//System.out.println("Press Enter to quit...");
		
		// Remove the sample listener when done
		controller.removeListener(listener);
		
	}

	@Override
	public void serialEvent(SerialPortEvent arg0) {
		// TODO Auto-generated method stub
		
	}

}

