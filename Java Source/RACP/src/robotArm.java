import java.io.BufferedReader;
import java.io.IOException;
import java.io.OutputStream;
import java.text.DecimalFormat;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

public class robotArm implements SerialPortEventListener {
	
	public Joint[] Joints;
	static OutputStream out = null;
	static BufferedReader input;
	
	static DecimalFormat df = new DecimalFormat("000");
	
	//Constructor
	public robotArm(Joint[] Joints, String portName) {
		
		this.Joints = Joints;
		
		System.out.println("Connecting to Arm...");
		
		try {
			new robotArm(null, null).connect(portName);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			System.out.println("connection failed");
			e.printStackTrace();
		}
		
		System.out.println("initializing Arm");
		this.update();
		
	}
	
	public void update() {
		
		String data = "";
		
		for(Joint j : Joints) {
		
			if(!j.equals(Joints[-1])) {
			
				data += df.format(j.position) + ":";
		
			} else {
			
				data += df.format(j.position);
			
			}
		
		}
		
		writeToArduino(data);
		
	}
	
		
	void connect (String portName) throws Exception {

		CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
		if ( portIdentifier.isCurrentlyOwned() )
		{
		//System.out.println("Error: Port is currently in use");
		}
		else
		{
		CommPort commPort = portIdentifier.open(this.getClass().getName(),2000);

		if ( commPort instanceof SerialPort )
		{
		SerialPort serialPort = (SerialPort) commPort;
		serialPort.setSerialPortParams(4800,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,
		SerialPort.PARITY_NONE);
		out = serialPort.getOutputStream();
		//input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
		
		// add event listeners
		try {
					serialPort.addEventListener(this);
					serialPort.notifyOnDataAvailable(true);
				} catch (Exception e) {
					//System.err.println(e.toString());
		
		}
		}
		else
		{
		System.out.println("Selected port is not a Serial Port");
		}
		}
		
		}

		public static void writeToArduino(String data)
		{
		String tmpStr = data;
		byte bytes[] = tmpStr.getBytes();
		try {
			/*//System.out.println("Sending Bytes: ");
			for(int i = 0; i<bytes.length; i++) {
				//System.out.println(bytes[i]);
			}*/
		out.write(bytes);
		} catch (IOException e) { }
		}
		
		public synchronized void serialEvent(SerialPortEvent oEvent) {
			if (oEvent.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
				try {
					String inputLine=input.readLine();
					System.out.println("Received: " + inputLine);
				} catch (Exception e) {
					System.err.println(e.toString());
				}
			}
			// Ignore all the other eventTypes, but you should consider the other ones.
		}
	
		
	}
