import org.apache.commons.math.*;
import org.apache.commons.math.linear.LUDecompositionImpl;
import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;
import org.apache.commons.math.random.JDKRandomGenerator;
import org.apache.commons.math.random.RandomGenerator;

public class KalmanFilter {
	
	public KalmanFilter() {
		
		
	}
	
	//Data needed for the predict step
	 double[][] YkData = { {0/*Predicted state positiom*/}, {0/*Predicted state velocity*/} };
	 double[][] AData = { {1, 0.03}, {0, 1} }; //Motion model
	 double[][] Yk_prevData = { {0/*Previous state positiom*/}, {0/*Previous state velocity*/} };
	 double[][] PkData = {{1,1}, {1,1}}; //Projected error covariance
	 double[][] Pk_prevData = {{1,1}, {1,1}}; //Previous error covariance
	 double[][] QData = {{(2.025*Math.pow(10, (-7))), 1.35*Math.pow(10, (-5))}, {1.35*Math.pow(10, (-5)), Math.pow(0.03, 2)}}; //Covariance of error noise
	 double[][] HData = {{1, 0}}; //How readings affect the state
	 double[][] RData = {{5}, {5}}; //Measurement noise
	 double[][] YkHatData = {{0}};
	 double[][] YkHat_prevData = {{0}};
	 double[][] ZkData = {{0}};
	 double[][] identityData = { {1, 0}, {0,1} };
	 double[][] KData = {{0}};
	
	 double sensorInput = 0;
	
	//Create matrices
	 RealMatrix Yk = MatrixUtils.createRealMatrix(YkData);
	 RealMatrix A = MatrixUtils.createRealMatrix(AData);
	 RealMatrix Yk_prev = MatrixUtils.createRealMatrix(Yk_prevData);
	 RealMatrix Pk = MatrixUtils.createRealMatrix(PkData);
	 RealMatrix Pk_prev = MatrixUtils.createRealMatrix(Pk_prevData);
	 RealMatrix Q = MatrixUtils.createRealMatrix(QData);
	 RealMatrix H = MatrixUtils.createRealMatrix(HData);
	 RealMatrix R = MatrixUtils.createRealMatrix(RData);
	 RealMatrix YkHat = MatrixUtils.createRealMatrix(YkHatData);
	 RealMatrix YkHat_prev = MatrixUtils.createRealMatrix(YkHat_prevData);
	 RealMatrix Zk = MatrixUtils.createRealMatrix(ZkData);
	 RealMatrix I = MatrixUtils.createRealMatrix(identityData);
	 RealMatrix K = MatrixUtils.createRealMatrix(KData);
	
	public  void main(String[] args) {

		//Run loop 100 times
		for(int i = 0; i<100000; i++) {
			
			//System.out.println("Iteration " + (i+1) + ": ");
			
			//predict(Yk, A, Yk_prev, Pk, Pk_prev, Q);
			//update(K, Pk, Pk_prev, H, R, YkHat, YkHat_prev, Zk, I);
			
			//System.out.println("");
			
		}
		
	}

	public void predict(RealMatrix Yk, RealMatrix A, RealMatrix Yk_prev, RealMatrix Pk, RealMatrix Pk_prev, RealMatrix Q) {
		
		//Predict state ahead
		this.Yk = A.multiply(Yk_prev);
		
		//System.out.println("Estimate: " + Yk);
		
		//Project error covariance ahead
		this.Pk = (A.multiply(Pk_prev).multiply(A.transpose())).add(Q);
		
	}
	
	public void update(RealMatrix K, RealMatrix Pk, RealMatrix Pk_prev, RealMatrix H, RealMatrix R, RealMatrix YkHat, RealMatrix YkHat_prev, RealMatrix Zk, RealMatrix I) {
		
		/*RandomGenerator rand = new JDKRandomGenerator();
		
		double randNum = rand.nextGaussian();
		
		KalmanFilter.sensorInput+=(rand.nextGaussian()*100);
		
		System.out.println("Sensor input: " + KalmanFilter.sensorInput);
		
		Zk.setEntry(0,0, KalmanFilter.sensorInput);*/
		
		//Find the inverse of the S matrix term in the K equation
		RealMatrix S_inverse = new LUDecompositionImpl(H.multiply(Pk).multiply(H.transpose().add(R))).getSolver().getInverse();
		
		//Compute Kalman Gain
		K = Pk.multiply(H.transpose()).multiply(S_inverse);
		
		//System.out.println("Kalman Gain: " + K);
		
		//Update estimate with measurements
		
		//create intermediate (the operation in parentheses)
		RealMatrix temp = Zk.subtract((H.multiply(Yk)));
		
		this.YkHat = YkHat = Yk.add(K.multiply(temp));
		
		//New predict is now old predict
		this.Yk_prev = YkHat;
		
		//Print out new value
		System.out.println("");
		System.out.println("YkHat: " + YkHat);
		
		//Update error covariance:
		
		//update temp matrix with terms in parentheses
		temp = (I.subtract(K.multiply(H)));
		
		this.Pk = temp.multiply(Pk);
		
		this.Pk_prev = Pk;
		
	}
	
	public int getOutput() {
		
		return (int) this.YkHat.getEntry(0, 0);
		
	}
	
}