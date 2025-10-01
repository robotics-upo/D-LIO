#ifndef __TDF3D_64_HPP__
#define __TDF3D_64_HPP__

#include <algorithm>  
#include <bitset>
#include <dlo3d/df3d.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "rclcpp/clock.hpp"
#include <dlo3d/grid_64.hpp>

class TDF3D64
{
	
public:

	TDF3D64(void) 
	{
		m_maxX = 40;
		m_maxY = 40;
		m_maxZ = 40;
		m_minX = -40;
		m_minY = -40;
		m_minZ = -10;
		m_resolution = 0.05;
		m_oneDivRes = 1/m_resolution;
		
		int k = 0;
		// Creates the manhatan distance mask kernel
		for(int z=-20; z<=20; z++){
			for(int y=-20; y<=20; y++){
				for(int x=-20; x<=20; x++){
				int d = abs(x) + abs(y) + abs(z);
				if (d == 0) {
					kernel[k++] = 0ULL;
				} else {
					kernel[k++] = ((uint64_t)0xffffffffffffffff) >> (64 - d);
				}
				}
			}
		}
	}

	~TDF3D64(void)
	{
	}

	void setup(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, float resolution, int maxCells = 200000)
	{
		m_maxX = maxX;
		m_maxY = maxY;
		m_maxZ = maxZ;
		m_minX = minX;
		m_minY = minY;
		m_minZ = minZ;
		m_resolution = resolution;
		m_oneDivRes = 1/m_resolution;

		// Setup grid
		m_grid.setup(m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ, m_resolution, maxCells);
	}

	void clear(void)
	{
		m_grid.clear();
	}

	void exportGridToPCD(const std::string& filename, int subsampling_factor)
	{

		m_grid.exportGridToPCD(filename, subsampling_factor);
	}

	virtual inline bool isIntoGrid(const float &x, const float &y, const float &z)
	{
		return (x > m_minX+1 && y > m_minY+1 && z > m_minZ+1 && x < m_maxX-1 && y < m_maxY-1 && z < m_maxZ-1);
	}


	void loadCloud(std::vector<pcl::PointXYZ> &cloud, float tx, float ty, float tz, float yaw)
	{
		std::vector<pcl::PointXYZ> out;

		float c = cos(yaw);
		float s = sin(yaw);
		out.resize(cloud.size());
		for(uint32_t i=0; i<out.size(); i++)
		{
			out[i].x = c*cloud[i].x - s*cloud[i].y + tx;
			out[i].y = s*cloud[i].x + c*cloud[i].y + ty; 
			out[i].z = cloud[i].z + tz;
		}
		loadCloud(out);
	}
	void loadCloudFiltered(std::vector<pcl::PointXYZ> &cloud, 
						float tx, float ty, float tz,
						float roll, float pitch, float yaw,
						float max_dist_xy)
		{
		std::vector<pcl::PointXYZ> out;
		out.reserve(cloud.size()); // reservar para eficiencia

		float max_dist_sq = max_dist_xy * max_dist_xy; // comparar al cuadrado

		// Filtrar primero en coordenadas locales
		for(size_t i = 0; i < cloud.size(); ++i)
		{
			float x = cloud[i].x;
			float y = cloud[i].y;

			if(x*x + y*y <= max_dist_sq)
			{
				out.push_back(cloud[i]);
			}
		}

		// Si no quedan puntos tras el filtrado, salir
		if(out.empty()) return;

		// Precompute rotation matrix
		float cr, sr, cp, sp, cy, sy;
		float r00, r01, r02, r10, r11, r12, r20, r21, r22;
		sr = sin(roll);
		cr = cos(roll);
		sp = sin(pitch);
		cp = cos(pitch);
		sy = sin(yaw);
		cy = cos(yaw);
		r00 = cy*cp; 	r01 = cy*sp*sr - sy*cr; 	r02 = cy*sp*cr + sy*sr;
		r10 = sy*cp; 	r11 = sy*sp*sr + cy*cr; 	r12 = sy*sp*cr - cy*sr;
		r20 = -sp; 	r21 = cp*sr; 			r22 = cp*cr;

		// Transformar los puntos filtrados al marco global
		for(size_t i = 0; i < out.size(); ++i)
		{
			float x = out[i].x;
			float y = out[i].y;
			float z = out[i].z;

			out[i].x = x*r00 + y*r01 + z*r02 + tx;
			out[i].y = x*r10 + y*r11 + z*r12 + ty;
			out[i].z = x*r20 + y*r21 + z*r22 + tz;
		}

		// Llamar al loadCloud normal
		loadCloud(out);
	}

	void loadCloud(std::vector<pcl::PointXYZ> &cloud, float tx, float ty, float tz, float roll, float pitch, float yaw)
	{
		std::vector<pcl::PointXYZ> out;

		// Get rotation matrix
		float cr, sr, cp, sp, cy, sy;
		float r00, r01, r02, r10, r11, r12, r20, r21, r22;
		sr = sin(roll);
		cr = cos(roll);
		sp = sin(pitch);
		cp = cos(pitch);
		sy = sin(yaw);
		cy = cos(yaw);
		r00 = cy*cp; 	r01 = cy*sp*sr-sy*cr; 	r02 = cy*sp*cr+sy*sr;
		r10 = sy*cp; 	r11 = sy*sp*sr+cy*cr;	r12 = sy*sp*cr-cy*sr;
		r20 = -sp;		r21 = cp*sr;			r22 = cp*cr;

		// Tiltcompensate points 
		out.resize(cloud.size());
		for(uint i=0; i<cloud.size(); i++) 
		{
			out[i].x = cloud[i].x*r00 + cloud[i].y*r01 + cloud[i].z*r02 + tx;
			out[i].y = cloud[i].x*r10 + cloud[i].y*r11 + cloud[i].z*r12 + ty;
			out[i].z = cloud[i].x*r20 + cloud[i].y*r21 + cloud[i].z*r22 + tz;	
		}
		loadCloud(out);
	}


	void loadCloud(std::vector<pcl::PointXYZ> &cloud)
	{	
		// Allocate required submetric cells
		for(uint32_t i=0; i<cloud.size(); i++)
		{
			for(float z=cloud[i].z-2; z<=cloud[i].z+2; z+=1)
				for(float y=cloud[i].y-2; y<=cloud[i].y+2; y+=1)
					for(float x=cloud[i].x-2; x<=cloud[i].x+2; x+=1)
						if(isIntoGrid(x, y, z))
							m_grid.allocCell(x, y, z);
		}

		// Applies the pre-computed kernel to all grid cells centered in the cloud points 
		const float step = 20*m_resolution;
		#pragma omp parallel for num_threads(20) shared(m_grid) 
		for(uint32_t i=0; i<cloud.size(); i++)
		{
			
			if(!isIntoGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step) || 
			   !isIntoGrid(cloud[i].x+step, cloud[i].y+step, cloud[i].z+step))
				continue;
						
			if(m_grid(cloud[i].x,cloud[i].y,cloud[i].z) == 0)
				continue;

			int xi, yi, zi, k = 0;
			float x, y, z;
			for(zi=0, z=cloud[i].z-step; zi<41; zi++, z+=m_resolution)
				for(yi=0, y=cloud[i].y-step; yi<41; yi++, y+=m_resolution){
						GRID64::Iterator it = m_grid.getIterator(cloud[i].x-step,y,z);
					for(xi=0, x=cloud[i].x-step; xi<41; xi++, x+=m_resolution,++it){
						*it &= kernel[k++];
					}
				}
		}
		#pragma omp barrier
	}

	inline TrilinearParams computeDistInterpolation(const double &x, const double &y, const double &z)
	{
		TrilinearParams r;

		if(isIntoGrid(x, y, z))
		{

			// Get neightbour values to compute trilinear interpolation
			float c000, c001, c010, c011, c100, c101, c110, c111;
			c000 = std::bitset<64>(m_grid.read(x, y, z)).count(); 
			c001 = std::bitset<64>(m_grid.read(x, y, z+m_resolution)).count(); 
			c010 = std::bitset<64>(m_grid.read(x, y+m_resolution, z)).count(); 
			c011 = std::bitset<64>(m_grid.read(x, y+m_resolution, z+m_resolution)).count();  
			c100 = std::bitset<64>(m_grid.read(x+m_resolution, y, z)).count();  
			c101 = std::bitset<64>(m_grid.read(x+m_resolution, y, z+m_resolution)).count();  
			c110 = std::bitset<64>(m_grid.read(x+m_resolution, y+m_resolution, z)).count();  
			c111 = std::bitset<64>(m_grid.read(x+m_resolution, y+m_resolution, z+m_resolution)).count(); 

			// Compute trilinear parameters
			const float div = -m_oneDivRes*m_oneDivRes*m_oneDivRes;
			float x0, y0, z0, x1, y1, z1;
			x0 = ((int)(x*m_oneDivRes))*m_resolution;
			if(x0<0)
				x0 -= m_resolution; 
			x1 = x0+m_resolution;
			y0 = ((int)(y*m_oneDivRes))*m_resolution;
			if(y0<0)
				y0 -= m_resolution;
			y1 = y0+m_resolution;
			z0 = ((int)(z*m_oneDivRes))*m_resolution;
			if(z0<0)
				z0 -= m_resolution;
			z1 = z0+m_resolution;
			r.a0 = (-c000*x1*y1*z1 + c001*x1*y1*z0 + c010*x1*y0*z1 - c011*x1*y0*z0 
			+ c100*x0*y1*z1 - c101*x0*y1*z0 - c110*x0*y0*z1 + c111*x0*y0*z0)*div;
			r.a1 = (c000*y1*z1 - c001*y1*z0 - c010*y0*z1 + c011*y0*z0
			- c100*y1*z1 + c101*y1*z0 + c110*y0*z1 - c111*y0*z0)*div;
			r.a2 = (c000*x1*z1 - c001*x1*z0 - c010*x1*z1 + c011*x1*z0 
			- c100*x0*z1 + c101*x0*z0 + c110*x0*z1 - c111*x0*z0)*div;
			r.a3 = (c000*x1*y1 - c001*x1*y1 - c010*x1*y0 + c011*x1*y0 
			- c100*x0*y1 + c101*x0*y1 + c110*x0*y0 - c111*x0*y0)*div;
			r.a4 = (-c000*z1 + c001*z0 + c010*z1 - c011*z0 + c100*z1 
			- c101*z0 - c110*z1 + c111*z0)*div;
			r.a5 = (-c000*y1 + c001*y1 + c010*y0 - c011*y0 + c100*y1 
			- c101*y1 - c110*y0 + c111*y0)*div;
			r.a6 = (-c000*x1 + c001*x1 + c010*x1 - c011*x1 + c100*x0 
			- c101*x0 - c110*x0 + c111*x0)*div;
			r.a7 = (c000 - c001 - c010 + c011 - c100
			+ c101 + c110 - c111)*div;
		}

		return r;
	}





protected:

	// Grid parameters
	float m_maxX, m_maxY, m_maxZ;
	float m_minX, m_minY, m_minZ;
	float m_resolution, m_oneDivRes;	
	uint64_t kernel[41*41*41];
	GRID64 m_grid;
};	


#endif
