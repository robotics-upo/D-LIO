#ifndef __IMUFILTER_HPP__
#define __IMUFILTER_HPP__
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <cmath>


// Convenient constants
#define RAD2DEG(X) ( (X) * 57.2957795131)
#define DEG2RAD(X) ( (X) * 0.01745329251)
#define LIN2GRAV(X) ( (X) * 0.10197162)

class ImuFilter
{
public:
	
	// Default constructor
	// Input: 
	// - prediction period in seconds
	// - initial calibration time in seconds
	ImuFilter(double _T, double _calibTime =3.0)
	{
		init = false;
		T = _T;
		T2 = _T*_T;
		calibTime = _calibTime;
		calibSize = (int)_calibTime/_T;
		calibData.resize(calibSize);
		calibIndex = 0;

		// IMU EKF parameters
		acc_dev = 0.0365432018302;// - 0.0365432018302
		gyr_dev = 0.00367396706572;//;//0.00367396706572;
		
		gyr_rw_dev = 2.66e-07; //2.616129872371749e-06; // random walk
		acc_rw_dev = 0.000433;
		sec = 0;

		output_file.open("predict_data.txt", std::ios::out);
		if (!output_file.is_open()) {
			throw std::runtime_error("No se pudo abrir el archivo predict_data.txt");
		}
		output_file << "stamp x y z vx vy vz rx ry rz gx gy gz ax ay az\n";



		// Abrir el archivo CSV para odometría
        odom_file.open("predict_data.csv", std::ios::out);
        if (!odom_file.is_open()) {
            throw std::runtime_error("No se pudo abrir el archivo odometry_data.csv");
        }
        // Escribir la cabecera del archivo CSV
        odom_file << "time,field.header.seq,field.header.stamp,field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w,field.twist.twist.linear.x,field.twist.twist.linear.y,field.twist.twist.linear.z,field.twist.twist.angular.x,field.twist.twist.angular.y,field.twist.twist.angular.z, ex,ey,ez,erx,ery,erz,evx,evy,evz,gbx,gby,gbz,abx,aby,abz\n";


	}
	~ImuFilter() {
		if (output_file.is_open()) {
			output_file.close();
		}
		 if (odom_file.is_open()) {
            odom_file.close();
        }
	}

	
	// Initialize EKF
	// Input: last sensor_msgs::Imu
	// The user must continue calling this function with new gyro data until it returns true
	bool initialize(sensor_msgs::msg::Imu &msg)
	{
		double gx_m, gy_m, gz_m, ax_m, ay_m, az_m; 
		
		// Almacenar datos para calibración
		calibData[calibIndex++ % calibSize] = msg;
		if(calibIndex < calibSize)
			return false;
		

		// Calcular la media de las velocidades angulares
		gx_m = gy_m = gz_m = 0.0;
		ax_m = ay_m = az_m = 0.0;
		for(int i = 0; i < (int)calibData.size(); i++)
		{
			double gx_val = calibData[i].angular_velocity.x;
			double gy_val = calibData[i].angular_velocity.y;
			double gz_val = calibData[i].angular_velocity.z;

			if (!std::isnan(gx_val)) gx_m += gx_val;
			if (!std::isnan(gy_val)) gy_m += gy_val;
			if (!std::isnan(gz_val)) gz_m += gz_val;

			double ax_val = calibData[i].linear_acceleration.x;
			double ay_val = calibData[i].linear_acceleration.y;
			double az_val = calibData[i].linear_acceleration.z;

			if (!std::isnan(ax_val)) ax_m += ax_val;
			if (!std::isnan(ay_val)) ay_m += ay_val;
			if (!std::isnan(az_val)) az_m += az_val;
			
		}

		gx_m /= static_cast<double>(calibData.size());
		gy_m /= static_cast<double>(calibData.size());
		gz_m /= static_cast<double>(calibData.size());

		// Calcular la media de las aceleraciones lineales
		ax_m /= static_cast<double>(calibData.size());
		ay_m /= static_cast<double>(calibData.size());
		az_m /= static_cast<double>(calibData.size());
		

		// Inicializar el vector de estado
		x = y = z = rz = rx = ry = vx = vy = vz = abx = aby = abz = 0.0;

		rx = atan2(ay_m, az_m); 
		ry = atan2(-ax_m, sqrt(ay_m * ay_m + az_m * az_m)); 

		gbx = gx_m;
		gby = gy_m;
		gbz = gz_m;

		gxf = gx_m;
		gyf = gy_m;
		gzf = gz_m;


		// Construir la matriz de rotación inicial (Yaw = 0)
		double cx = std::cos(rx), sx = std::sin(rx);
		double cy = std::cos(ry), sy = std::sin(ry);
		double cz = std::cos(rz), sz = std::sin(rz);

		Eigen::Matrix3d R;
		R << cz * cy,                   cz * sy * sx - sz * cx,    cz * sy * cx + sz * sx,
			sz * cy,                   sz * sy * sx + cz * cx,    sz * sy * cx - cz * sx,
			-sy,                       cy * sx,                   cy * cx;

		// Aceleración medida en el marco local
		Eigen::Vector3d a_measured(ax_m, ay_m, az_m);
		Eigen::Vector3d gravity_global(0.0, 0.0, 9.80665);

		// Aceleración esperada en el marco local
		Eigen::Vector3d a_expected_local = R.transpose() * gravity_global;

		// Calcular el bias del acelerómetro
		Eigen::Vector3d bias = a_measured - a_expected_local;

		abx = bias(0);
		aby = bias(1);
		abz = bias(2);

		std::cout<<"acc bias: x-"<<abx<<" y- "<<aby<<" z- "<<abz<<std::endl;

		// Inicializar la matriz de covarianza P (x,y,z,rx,ry,rz,vx,vy,vz,gbx,gby,gbz,abx,aby,abz)
		P(0,0) = 0.0001*0.0001;
		P(1,1) = 0.0001*0.0001;
		P(2,2) = 0.0001*0.0001;

		P(3,3) = 0.0025*0.0025; // rx
		P(4,4) = 0.0025 * 0.0025; // ry	
		P(5,5) = 0.0025 * 0.0025;

		P(6,6) = 0.0001*0.0001;
		P(7,7) = 0.0001*0.0001;
		P(8,8) = 0.0001*0.0001;

		P(9,9) = 0.000003 * 0.000003;
		P(10,10) = 0.000003 * 0.000003;
		P(11,11) = 0.000003 * 0.000003;
		P(12,12) = 0.003 * 0.003;
		P(13,13) = 0.003 * 0.003;
		P(14,14) = 0.003 * 0.003;	
		init = true;	
		return true;
	}
	
	
	// EKF prediction stage based on gyro information
	// Input: raw gyro sensors rad/s
	bool predict(double gx, double gy, double gz,double ax, double ay, double az,double stamp)
	{
		gxf = gx;
		gyf = gy;
		gzf = gz;

		// Check initialization 
		if(!init)
			return false;

		double cx = std::cos(rx), sx = std::sin(rx);
		double cy = std::cos(ry), sy = std::sin(ry);
		double cz = std::cos(rz), sz = std::sin(rz);

		tf2::Quaternion q;
		q.setRPY(rx, ry, rz);  // rx, ry, rz en radianes (roll, pitch, yaw)

		// Convertir el cuaternión en una matriz de rotación 3x3
		tf2::Matrix3x3 tf2_R(q.normalize());


		// Update state vector
		rx += T*(gx - gbx);  
		ry += T*(gy - gby);  
		rz += T*(gz - gbz);

		

		// Convertir tf2::Matrix3x3 a Eigen::Matrix3d
		Eigen::Matrix3d R;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				R(i, j) = tf2_R[i][j];


		Eigen::Vector3d increment = Eigen::Vector3d(T, T, T).cwiseProduct(
			(R * (Eigen::Vector3d(ax, ay, az) - Eigen::Vector3d(abx, aby, abz))) 
			- Eigen::Vector3d(0.0, 0.0, 9.80665));


		v(0) = vx; v(1) = vy; v(2) = vz;
		v += increment;
		vx = v(0);
		vy = v(1); 
		vz = v(2);

		x += vx*T+increment(0)*T*0.5;
		y += vy*T+increment(1)*T*0.5;
		z += vz*T+increment(2)*T*0.5;
		
		output_file << std::fixed << std::setprecision(9);
		output_file << stamp << " " <<x << " " << y << " " << z << " "
            << vx << " " << vy << " " << vz << " "
            << rx << " " << ry << " " << rz << " "
            << gx << " " << gy << " " << gz << " "
            << ax << " " << ay << " " << az << "\n";


		

		

		// Compute matrix F  --------------------------------------------------------------------------
		
		
		// Diferencias de aceleración
		double dax = abx -ax , day = aby - ay, daz = abz - az;

		// Precomputación de expresiones comunes
		double T2 = T * T, T2_2 = T2 / 2, T_sy = T * sy, T_cy = T * cy;

		// Inicialización de la matriz Jacobiana F a cero
		Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();

		// Llenado de la matriz F con las ecuaciones en una sola línea

		// Elementos identidad
		F(0, 0) = 1;F(1, 1) = 1;F(2, 2) = 1;F(3, 3) = 1;F(4, 4) = 1;F(5, 5) = 1;F(6, 6) = 1;F(8, 8) = 1;
		F(9, 9) = 1;F(10, 10) = 1;F(11, 11) = 1;F(12, 12) = 1;F(13, 13) = 1;F(14, 14) = 1;
		//Fila 1
		F(0, 3) = -T2_2 * ((sx * sz + cx * cz * sy) * day + (cx * sz - cz * sx * sy) * daz);
		F(0, 4) = -T2_2 * (cx * cy * cz * daz - cz * sy * dax + cy * cz * sx * day);
		F(0, 5) =  T2_2 * ((cx * cz + sx * sy * sz) * day - (cz * sx - cx * sy * sz) * daz + cy * sz * dax);
		F(0, 6) = T;
		F(0, 9) = -T2_2 * cy * cz;
		F(0, 10) =  T2_2 * (cx * sz - cz * sx * sy);
		F(0, 11) = -T2_2 * (sx * sz + cx * cz * sy);

		// Fila 2
		F(1, 3) =  T2_2 * ((cz * sx - cx * sy * sz) * day + (cx * cz + sx * sy * sz) * daz);
		F(1, 4) = -T2_2 * (cx * cy * sz * daz - sy * sz * dax + cy * sx * sz * day);
		F(1, 5) = -T2_2 * ((sx * sz + cx * cz * sy) * daz - (cx * sz - cz * sx * sy) * day + cy * cz * dax);
		F(1, 7) = T;
		F(1, 9) = -T2_2 * cy * sz;
		F(1, 10) = -T2_2 * (cx * cz + sx * sy * sz);
		F(1, 11) =  T2_2 * (cz * sx - cx * sy * sz);

		// Fila 3
		F(2, 3) = -T2_2 * (cx * cy * day - cy * sx * daz);
		F(2, 4) =  T2_2 * (cy * dax + cx * sy * daz + sx * sy * day);
		F(2, 8) = T;
		F(2, 12) =  T2_2 * sy;
		F(2, 13) = -T2_2 * cy * sx;
		F(2, 14) = -T2_2 * cx * cy;

		// Fila 4,5,6
		F(3, 9) = -T;F(4, 10) = -T;F(5, 11) = -T;

		// Fila 7
		F(6, 3) = -T * ((sx * sz + cx * cz * sy) * day + (cx * sz - cz * sx * sy) * daz);
		F(6, 4) = -T * (cx * cy * cz * daz - cz * sy * dax + cy * cz * sx * day);
		F(6, 5) =  T * ((cx * cz + sx * sy * sz) * day - (cz * sx - cx * sy * sz) * daz + cy * sz * dax);
		
		F(6, 9) = -T * cy * cz;
		F(6, 10) =  T * (cx * sz - cz * sx * sy);
		F(6, 11) = -T * (sx * sz + cx * cz * sy);

		// Fila 8
		F(7, 3) =  T * ((cz * sx - cx * sy * sz) * day + (cx * cz + sx * sy * sz) * daz);
		F(7, 4) = -T * (cx * cy * sz * daz - sy * sz * dax + cy * sx * sz * day);
		F(7, 5) = -T * ((sx * sz + cx * cz * sy) * daz - (cx * sz - cz * sx * sy) * day + cy * cz * dax);
		F(7, 7) = 1;
		F(7, 9) = -T * cy * sz;
		F(7, 10) = -T * (cx * cz + sx * sy * sz);
		F(7, 11) =  T * (cz * sx - cx * sy * sz);

		// Fila 9
		F(8, 3) = -T * (cx * cy * day - cy * sx * daz);
		F(8, 4) =  T * (cy * dax + cx * sy * daz + sx * sy * day);
		F(8, 12) = T_sy;
		F(8, 13) = -T_cy * sx;
		F(8, 14) = -T_cy * cx;

		// Compute matrix R and B -----------------------------------------------------------------------
		Eigen::Matrix<double, 6, 6> R_p;
		R_p.setZero();
		R_p(0,0) = gyr_dev*gyr_dev;
		R_p(1,1) = gyr_dev*gyr_dev;
		R_p(2,2) = gyr_dev*gyr_dev;
		R_p(3,3) = acc_dev*acc_dev;
		R_p(4,4) = acc_dev*acc_dev;
		R_p(5,5) = acc_dev*acc_dev;

		Eigen::Matrix<double, 15, 6> B;
		B.setZero();

		// rx, ry, rz
		B(0, 3) = T2_2 * cy * cz;
		B(0, 4) = -T2_2 * (cx * sz - cz * sx * sy);
		B(0, 5) = T2_2 * (sx * sz + cx * cz * sy);

		B(1, 3) = T2_2 * cy * sz;
		B(1, 4) = T2_2 * (cx * cz + sx * sy * sz);
		B(1, 5) = -T2_2 * (cz * sx - cx * sy * sz);

		B(2, 3) = -T2_2 * sy;
		B(2, 4) = T2_2 * cy * sx;
		B(2, 5) = T2_2 * cx * cy;

		B(3, 0) = T;
		B(4, 1) = T;
		B(5, 2) = T;
		B(6, 3) = T * cy * cz;
		B(6, 4) = T * (cz * sx * sy - cx * sz);
		B(6, 5) = T * (sx * sz + cx * cz * sy);

		B(7, 3) = T * cy * sz;
		B(7, 4) = T * (cx * cz + sx * sy * sz);
		B(7, 5) = T * (cx * sy * sz - cz * sx);

		B(8, 3) = -T * sy;
		B(8, 4) = T * cy * sx;
		B(8, 5) = T * cx * cy;

		// Update covariance matrix ---------------------------------------------------------------------
		P = F*P*F.transpose() + B*R_p*B.transpose();
		P(9,9) += gyr_rw_dev*gyr_rw_dev*T2;
		P(10,10) += gyr_rw_dev*gyr_rw_dev*T2;
		P(11,11) += gyr_rw_dev*gyr_rw_dev*T2;
		P(12,12) += acc_rw_dev*acc_rw_dev*T2;
		P(13,13) += acc_rw_dev*acc_rw_dev*T2;
		P(14,14) += acc_rw_dev*acc_rw_dev*T2;
				
		return true; 
	}
	
	// EKF update stage based on accelerometer information
	// Input: raw accelerometer sensors in g
	bool update_opt(double xopt, double yopt, double zopt, double yaw,double vxopt, double vyopt,double vzopt,double var_pos_opt,double var_a_opt,double var_v_opt,double stamp)
	{
		// Check initialization 
		if(!init)
			return false;

		// Create measurement jacobian H -> Filas x, y, z, y 
		Eigen::Matrix<double, 7, 15> H;
		H.setZero();
		H(0,0) = 1;
		H(1,1) = 1;
		H(2,2) = 1;
		H(3,5) = 1;
		H(4,6) = 1;
		H(5,7) = 1;
		H(6,8) = 1;

		// Compute measurement noise jacoban R
		Eigen::Matrix<double, 7, 7> R; // Ruido proveniente del optimizador?
		R.setZero();
		R(0,0) = var_pos_opt;
		R(1,1) = var_pos_opt;
		R(2,2) = var_pos_opt;
		R(3,3) = var_a_opt;
		R(4,4) = var_v_opt;
		R(5,5) = var_v_opt;
		R(6,6) = var_v_opt;
		
		
		// Compute innovation matrix
		Eigen::Matrix<double, 7, 7> S;
		S = H*P*H.transpose()+R;
		
		// Compute kalman gain
		Eigen::Matrix<double, 15, 7> K;
		K = P*H.transpose()*S.inverse();

		double e[7];
		// Compute mean error
		e[0] = xopt - x;
		e[1] = yopt - y;
		e[2] = zopt - z;
		e[3] = yaw - rz;
		e[4] = vxopt - vx;
		e[5] = vyopt - vy;
		e[6] = vzopt - vz;


		
		/*std::cout<<"ERRORES - X = "<<e[0]<<", Y = "<<e[1]<<", Z = "<<e[2]<<", R = "<<e[3]<<", Vx = "<<e[4]<<", Vy = "<<e[5]<<", Vz = "<<e[6]<<std::endl;
		std::cout<<"Medidas externas - x = "<<xopt<<", y = "<<yopt<<", z = "<<zopt<<", y = "<<yaw<<std::endl;
		std::cout<<"Medidas internas - x = "<<x<<", y = "<<y<<", z = "<<z<<", y = "<<rz<<std::endl;
		
		std::cout<<"State vector ---------------"<<std::endl;*/


		// Compute new state vector
		x  += K(0,0)*e[0] + K(0,1)*e[1] + K(0,2)*e[2] + K(0,3)*e[3] + K(0,4)*e[4] + K(0,5)*e[5] + K(0,6)*e[6];
		y  += K(1,0)*e[0] + K(1,1)*e[1] + K(1,2)*e[2] + K(1,3)*e[3] + K(1,4)*e[4] + K(1,5)*e[5] + K(1,6)*e[6];
		z  += K(2,0)*e[0] + K(2,1)*e[1] + K(2,2)*e[2] + K(2,3)*e[3] + K(2,4)*e[4] + K(2,5)*e[5] + K(2,6)*e[6];
		rx += K(3,0)*e[0] + K(3,1)*e[1] + K(3,2)*e[2] + K(3,3)*e[3] + K(3,4)*e[4] + K(3,5)*e[5] + K(3,6)*e[6];
		ry += K(4,0)*e[0] + K(4,1)*e[1] + K(4,2)*e[2] + K(4,3)*e[3] + K(4,4)*e[4] + K(4,5)*e[5] + K(4,6)*e[6];
		rz += K(5,0)*e[0] + K(5,1)*e[1] + K(5,2)*e[2] + K(5,3)*e[3] + K(5,4)*e[4] + K(5,5)*e[5] + K(5,6)*e[6];
		vx += K(6,0)*e[0] + K(6,1)*e[1] + K(6,2)*e[2] + K(6,3)*e[3] + K(6,4)*e[4] + K(6,5)*e[5] + K(6,6)*e[6];
		vy += K(7,0)*e[0] + K(7,1)*e[1] + K(7,2)*e[2] + K(7,3)*e[3] + K(7,4)*e[4] + K(7,5)*e[5] + K(7,6)*e[6];
		vz += K(8,0)*e[0] + K(8,1)*e[1] + K(8,2)*e[2] + K(8,3)*e[3] + K(8,4)*e[4] + K(8,5)*e[5] + K(8,6)*e[6];
		gbx += K(9,0)*e[0] + K(9,1)*e[1] + K(9,2)*e[2] + K(9,3)*e[3] + K(9,4)*e[4] + K(9,5)*e[5] + K(9,6)*e[6];
		gby += K(10,0)*e[0] + K(10,1)*e[1] + K(10,2)*e[2] + K(10,3)*e[3] + K(10,4)*e[4] + K(10,5)*e[5] + K(10,6)*e[6];
		gbz += K(11,0)*e[0] + K(11,1)*e[1] + K(11,2)*e[2] + K(11,3)*e[3] + K(11,4)*e[4] + K(11,5)*e[5] + K(11,6)*e[6];
		abx += K(12,0)*e[0] + K(12,1)*e[1] + K(12,2)*e[2] + K(12,3)*e[3] + K(12,4)*e[4] + K(12,5)*e[5] + K(12,6)*e[6];
		aby += K(13,0)*e[0] + K(13,1)*e[1] + K(13,2)*e[2] + K(13,3)*e[3] + K(13,4)*e[4] + K(13,5)*e[5] + K(13,6)*e[6];
		abz += K(14,0)*e[0] + K(14,1)*e[1] + K(14,2)*e[2] + K(14,3)*e[3] + K(14,4)*e[4] + K(14,5)*e[5] + K(14,6)*e[6];
	
	
		/*	std::cout << "x: " << x << " " << "y: " << y << " " << "z: " << z << std::endl;
			std::cout << "rx: " << rx << " " << "ry: " << ry << " " << "rz: " << rz << std::endl;
			std::cout << "vx: " << vx << " " << "vy: " << vy << " " << "vz: " << vz << std::endl;
			std::cout << "gbx: " << gbx << " " << "gby: " << gby << " " << "gbz: " << gbz << std::endl;
			std::cout << "abx: " << abx << " " << "aby: " << aby << " " << "abz: " << abz << std::endl;
			std::cout<<"---------------------------------------------------------------"<<std::endl;
		*/	
		// Compute new covariance matrix
		Eigen::Matrix<double, 15, 15> I;
		I.setIdentity();
		P = (I - K * H) * P;

		tf2::Quaternion q_ori;
   		q_ori.setRPY(rx, ry, rz);
		odom_file << std::fixed << std::setprecision(0)
                  << stamp * 1000000000ULL << ","  // %time
                  << sec << ","       // field.header.seq (poniéndolo a 0)
                  << stamp * 1000000000ULL<< ","       // field.header.stamp (poniéndolo a 0)
				  << std::fixed << std::setprecision(9)
                  << x << ","
                  << y << ","
                  << z << ","
                  << q_ori.x() << ","  // field.pose.pose.orientation.x (por ejemplo, 0 si no se tiene valor)
                  << q_ori.y() << ","  // field.pose.pose.orientation.y
                  << q_ori.z() << ","  // field.pose.pose.orientation.z
                  << q_ori.w() << ","  // field.pose.pose.orientation.w (valor 1 para orientación nula)
                  << vx << ","
                  << vy << ","
                  << vz << ","
                  << gxf << ","
                  << gyf << ","
                  << gzf << "\n";
		sec++;

		
		return true;
	}


		bool update_opt_full(double xopt, double yopt, double zopt, double roll, double pitch, double yaw,double vxopt, double vyopt,double vzopt,double var_pos_opt,double var_pos_z_opt,double var_a_opt,double var_a2_opt, double var_v_opt,double var_v_z_opt,double stamp)
	{
		// Check initialization 
		if(!init)
			return false;

		// Create measurement jacobian H -> Filas x, y, z, y 
		Eigen::Matrix<double, 9, 15> H;
		H.setZero();
		H(0,0) = 1;
		H(1,1) = 1;
		H(2,2) = 1;
		H(3,3) = 1;
		H(4,4) = 1;
		H(5,5) = 1;
		H(6,6) = 1;
		H(7,7) = 1;
		H(8,8) = 1;

		// Compute measurement noise jacoban R
		Eigen::Matrix<double, 9, 9> R; // Ruido proveniente del optimizador?
		R.setZero();
		R(0,0) = var_pos_opt;
		R(1,1) = var_pos_opt;
		R(2,2) = var_pos_z_opt;
		R(3,3) = var_a2_opt;
		R(4,4) = var_a2_opt;
		R(5,5) = var_a_opt;
		R(6,6) = var_v_opt;
		R(7,7) = var_v_opt;
		R(8,8) = var_v_z_opt;
		
		
		// Compute innovation matrix
		Eigen::Matrix<double, 9, 9> S;
		S = H*P*H.transpose()+R;
		
		// Compute kalman gain
		Eigen::Matrix<double, 15, 9> K;
		K = P*H.transpose()*S.inverse();

		double e[9];
		// Compute mean error
		e[0] = xopt - x;
		e[1] = yopt - y;
		e[2] = zopt - z;
		e[3] = (roll - rx);
		e[4] = (pitch - ry);
		e[5] = (yaw - rz);
		e[6] = vxopt - vx;
		e[7] = vyopt - vy;
		e[8] = vzopt - vz;


		
		std::cout<<"ERRORES - X = "<<e[0]<<", Y = "<<e[1]<<", Z = "<<e[2]<<", R = "<<e[3]<<", P = "<<e[4]<<", Y = "<<e[5]<<", Vx = "<<e[6]<<", Vy = "<<e[7]<<", Vz = "<<e[8]<<std::endl;
		std::cout<<"Medidas externas - x = "<<xopt<<", y = "<<yopt<<", z = "<<zopt<<", r = "<<roll<<", p = "<<pitch<<", y = "<<yaw<<std::endl;
		std::cout<<"Medidas internas - x = "<<x<<", y = "<<y<<", z = "<<z<<", r = "<<rx<<", p = "<<ry<<", y = "<<rz << "vx: " << vx << " " << "vy: " << vy << " " << "vz: " << vz <<std::endl;
		
		std::cout<<"State vector ---------------"<<std::endl;


		// Compute new state vector
		x  += K(0,0)*e[0] + K(0,1)*e[1] + K(0,2)*e[2] + K(0,3)*e[3] + K(0,4)*e[4] + K(0,5)*e[5] + K(0,6)*e[6] + K(0,7)*e[7] + K(0,8)*e[8];
		y  += K(1,0)*e[0] + K(1,1)*e[1] + K(1,2)*e[2] + K(1,3)*e[3] + K(1,4)*e[4] + K(1,5)*e[5] + K(1,6)*e[6] + K(1,7)*e[7] + K(1,8)*e[8];
		z  += K(2,0)*e[0] + K(2,1)*e[1] + K(2,2)*e[2] + K(2,3)*e[3] + K(2,4)*e[4] + K(2,5)*e[5] + K(2,6)*e[6] + K(2,7)*e[7] + K(2,8)*e[8];
		rx += K(3,0)*e[0] + K(3,1)*e[1] + K(3,2)*e[2] + K(3,3)*e[3] + K(3,4)*e[4] + K(3,5)*e[5] + K(3,6)*e[6] + K(3,7)*e[7] + K(3,8)*e[8];
		ry += K(4,0)*e[0] + K(4,1)*e[1] + K(4,2)*e[2] + K(4,3)*e[3] + K(4,4)*e[4] + K(4,5)*e[5] + K(4,6)*e[6] + K(4,7)*e[7] + K(4,8)*e[8];
		rz += K(5,0)*e[0] + K(5,1)*e[1] + K(5,2)*e[2] + K(5,3)*e[3] + K(5,4)*e[4] + K(5,5)*e[5] + K(5,6)*e[6] + K(5,7)*e[7] + K(5,8)*e[8];
		vx += K(6,0)*e[0] + K(6,1)*e[1] + K(6,2)*e[2] + K(6,3)*e[3] + K(6,4)*e[4] + K(6,5)*e[5] + K(6,6)*e[6] + K(6,7)*e[7] + K(6,8)*e[8];
		vy += K(7,0)*e[0] + K(7,1)*e[1] + K(7,2)*e[2] + K(7,3)*e[3] + K(7,4)*e[4] + K(7,5)*e[5] + K(7,6)*e[6] + K(7,7)*e[7] + K(7,8)*e[8];
		vz += K(8,0)*e[0] + K(8,1)*e[1] + K(8,2)*e[2] + K(8,3)*e[3] + K(8,4)*e[4] + K(8,5)*e[5] + K(8,6)*e[6] + K(8,7)*e[7] + K(8,8)*e[8];
		gbx += K(9,0)*e[0] + K(9,1)*e[1] + K(9,2)*e[2] + K(9,3)*e[3] + K(9,4)*e[4] + K(9,5)*e[5] + K(9,6)*e[6] + K(9,7)*e[7] + K(9,8)*e[8];
		gby += K(10,0)*e[0] + K(10,1)*e[1] + K(10,2)*e[2] + K(10,3)*e[3] + K(10,4)*e[4] + K(10,5)*e[5] + K(10,6)*e[6] + K(10,7)*e[7] + K(10,8)*e[8];
		gbz += K(11,0)*e[0] + K(11,1)*e[1] + K(11,2)*e[2] + K(11,3)*e[3] + K(11,4)*e[4] + K(11,5)*e[5] + K(11,6)*e[6] + K(11,7)*e[7] + K(11,8)*e[8];
		abx += K(12,0)*e[0] + K(12,1)*e[1] + K(12,2)*e[2] + K(12,3)*e[3] + K(12,4)*e[4] + K(12,5)*e[5] + K(12,6)*e[6] + K(12,7)*e[7] + K(12,8)*e[8];
		aby += K(13,0)*e[0] + K(13,1)*e[1] + K(13,2)*e[2] + K(13,3)*e[3] + K(13,4)*e[4] + K(13,5)*e[5] + K(13,6)*e[6] + K(13,7)*e[7] + K(13,8)*e[8];
		abz += K(14,0)*e[0] + K(14,1)*e[1] + K(14,2)*e[2] + K(14,3)*e[3] + K(14,4)*e[4] + K(14,5)*e[5] + K(14,6)*e[6] + K(14,7)*e[7] + K(14,8)*e[8];

	
		std::cout << "x: " << x << " " << "y: " << y << " " << "z: " << z << std::endl;
		std::cout << "rx: " << rx << " " << "ry: " << ry << " " << "rz: " << rz << std::endl;
		std::cout << "vx: " << vx << " " << "vy: " << vy << " " << "vz: " << vz << std::endl;
	/*	std::cout << "gbx: " << gbx << " " << "gby: " << gby << " " << "gbz: " << gbz << std::endl;
		std::cout << "abx: " << abx << " " << "aby: " << aby << " " << "abz: " << abz << std::endl;
		std::cout<<"---------------------------------------------------------------"<<std::endl;
	*/	
		// Compute new covariance matrix
		Eigen::Matrix<double, 15, 15> I;
		I.setIdentity();
		P = (I - K * H) * P;

		tf2::Quaternion q_ori;
   		q_ori.setRPY(rx, ry, rz);
		odom_file << std::fixed << std::setprecision(0)
                  << stamp * 1000000000ULL << ","  // %time
                  << sec << ","       // field.header.seq (poniéndolo a 0)
                  << stamp * 1000000000ULL<< ","       // field.header.stamp (poniéndolo a 0)
				  << std::fixed << std::setprecision(9)
                  << x << ","
                  << y << ","
                  << z << ","
                  << q_ori.x() << ","  // field.pose.pose.orientation.x (por ejemplo, 0 si no se tiene valor)
                  << q_ori.y() << ","  // field.pose.pose.orientation.y
                  << q_ori.z() << ","  // field.pose.pose.orientation.z
                  << q_ori.w() << ","  // field.pose.pose.orientation.w (valor 1 para orientación nula)
                  << vx << ","
                  << vy << ","
                  << vz << ","
                  << gxf << ","
                  << gyf << ","
                  << gzf << ","
				  << e[0] << ","
				  << e[1] << ","
				  << e[2] << ","
				  << e[3] << ","
				  << e[4] << ","
				  << e[5] << ","
				  << e[6] << ","
				  << e[7] << ","
				  << e[8] << ","
				  << gbx << ","
				  << gby << ","
				  << gbz << ","
				  << abx << ","
				  << aby << ","
				  << abz  <<"\n";
		sec++;

		
		return true;
	}


	bool update_imu(double roll, double pitch,double var_a_opt)
	{

		// Check initialization 
		if(!init)
			return false;

		// Create measurement jacobian H -> Filas x, y, z, y 
		Eigen::Matrix<double, 2, 15> H;
		H.setZero();
		H(0,3) = 1;
		H(1,4) = 1;
		


		// Compute measurement noise jacoban R
		Eigen::Matrix<double, 2, 2> R; // Ruido proveniente del optimizador?
		R.setZero();
		R(0,0) = var_a_opt;
		R(1,1) = var_a_opt;

		
		
		// Compute innovation matrix
		Eigen::Matrix<double, 2, 2> S;
		S = H*P*H.transpose()+R;
		
		// Compute kalman gain
		Eigen::Matrix<double, 15, 2> K;
		K = P*H.transpose()*S.inverse();


		double e[2];
		// Compute mean error
		e[0] =  roll - rx;
		e[1] =  pitch - ry;

		// Compute new state vector
		x +=   K(0,0)*e[0]+K(0,1)*e[1];
		y +=   K(1,0)*e[0]+K(1,1)*e[1];
		z +=   K(2,0)*e[0]+K(2,1)*e[1];
		rx +=  K(3,0)*e[0]+K(3,1)*e[1];
		ry +=  K(4,0)*e[0]+K(4,1)*e[1];
		rz +=  K(5,0)*e[0]+K(5,1)*e[1];
		vx +=  K(6,0)*e[0]+K(6,1)*e[1];
		vy +=  K(7,0)*e[0]+K(7,1)*e[1];
		vz +=  K(8,0)*e[0]+K(8,1)*e[1];
		gbx += K(9,0)*e[0]+K(9,1)*e[1];
		gby += K(10,0)*e[0]+K(10,1)*e[1];
		gbz += K(11,0)*e[0]+K(11,1)*e[1];
		abx += K(12,0)*e[0]+K(12,1)*e[1];
		aby += K(13,0)*e[0]+K(13,1)*e[1];
		abz += K(14,0)*e[0]+K(14,1)*e[1];
		
		// Compute new covariance matrix
		Eigen::Matrix<double, 15, 15> I;
		I.setIdentity();
		P = (I-K*H)*P*(I-K*H).transpose() + K*R*K.transpose();

		
		return true;
	}


	
	// Get estimated roll, pith and yaw in radians interval [-PI,PI] rad
	bool getAngles(double &rx_, double &ry_, double &rz_)
	{
		rx_ = (rx);
		ry_ =(ry);
		rz_ = (rz);
		
		return true;
	}
	double get_sec(){
		return sec;
	}
	bool getVelocities(double &vx_, double &vy_, double &vz_)
	{
		vx_ = vx;
		vy_ = vy;
		vz_ = vz;
		
		return true;
	}

	bool getposition(double &x_, double &y_, double &z_)
	{
		x_ = x;
		y_ = y;
		z_ = z;
		
		return true;
	}

	// Get estimated bx, by and bz gyro BIAS in rad/s
	bool getBIAS(double &gbx_, double &gby_, double &gbz_)
	{
		gbx_ = gbx;
		gby_ = gby;
		gbz_ = gbz;
		
		return true;
	}
	
	// Is the IMU init?
	bool isInit(void)
	{
		return init;
	}
	// Función para calcular la diferencia mínima entre dos ángulos
	double AngleDifference(double angle1, double angle2)
	{
		return Pi2PiRange(angle1) - Pi2PiRange(angle2);
	}

	
protected:

	//! Round down absolute function
	double Floor_absolute( double value )
	{
	  if (value < 0.0)
		return ceil( value );
	  else
		return floor( value );
	}

	//! Convert angles into interval [-PI,0,PI]
	double Pi2PiRange(double cont_angle)
	{
		double bound_angle = 0.0;
		if(fabs(cont_angle)<=M_PI)
			bound_angle= cont_angle;
		else
		{
			if(cont_angle > M_PI)
				bound_angle = (cont_angle-2*M_PI) - 2*M_PI*Floor_absolute((cont_angle-M_PI)/(2*M_PI));
			
			if(cont_angle < - M_PI)
				bound_angle = (cont_angle+2*M_PI) - 2*M_PI*Floor_absolute((cont_angle+M_PI)/(2*M_PI));
		}
		
		return bound_angle;
	}

		
	// IMU Kalman filter prediction period and period^2
	double T, T2;
	
	// IMU Kalman filter matrixes 
	double x,y,z,rx, ry, rz,vx,vy,vz, gbx, gby, gbz,abx,aby,abz;  			// x = [rx, ry, rz, gbx, gby, gbz, ax_m, ay_m, az_m]
	double gxf, gyf, gzf, ax, ay, az;
	Eigen::Vector3d v;

	Eigen::Matrix<double, 15, 15> P;

	// Sensor variances [varX, varY, varZ]
	double acc_var[3]; 
	double gyr_var[3];

	// Filter initialized
	bool init;
	
	// EKF Parameters
	double acc_dev;
	double gyr_dev;
	double gyr_rw_dev, acc_rw_dev;
	double sec;
	// Input calibration data
	double calibTime;
	int calibIndex, calibSize;
	std::vector<sensor_msgs::msg::Imu> calibData;
	std::ofstream output_file;
	std::ofstream odom_file;

};

#endif