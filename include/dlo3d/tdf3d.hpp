#ifndef __TDF3D_HPP__
#define __TDF3D_HPP__

#include <algorithm>  
#include <bitset>
#include <dll3d/df3d.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/clock.hpp"

class TDF3D : public DF3D
{
	
public:

	TDF3D(void) 
	{
		m_gridMask = NULL;
		m_isUpdated = NULL;
		m_isOccupied = NULL;
		m_gridTrilinear = NULL;
		m_gridDist = NULL;
	}

	~TDF3D(void)
	{
		if(m_gridMask != NULL)
			free(m_gridMask);
		m_gridMask = NULL;
		if(m_isUpdated != NULL)
			free(m_isUpdated);
		m_isUpdated = NULL;
		if(m_isOccupied != NULL)
			free(m_isOccupied);
		m_isOccupied = NULL;
		if(m_gridTrilinear != NULL)
			free(m_gridTrilinear);
		m_gridTrilinear = NULL;
		if(m_gridDist != NULL)
			free(m_gridDist);
		m_gridDist = NULL;
	}

	void setup(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, float resolution)
	{
		m_maxX = maxX;
		m_maxY = maxY;
		m_maxZ = maxZ;
		m_minX = minX;
		m_minY = minY;
		m_minZ = minZ;
		m_resolution = resolution;

		// Free memory if needed
		if(m_gridTrilinear != NULL)
			free(m_gridTrilinear);
		if(m_gridDist != NULL)
			free(m_gridDist);
		if(m_gridMask != NULL)
			free(m_gridMask);
		if(m_isUpdated != NULL)
			free(m_isUpdated);
		if(m_isOccupied != NULL)
			free(m_isOccupied);
		
		// Memory allocation for the grid
		m_oneDivRes = 1.0/m_resolution;
		m_gridSizeX = fabs(m_maxX-m_minX)*m_oneDivRes;
		m_gridSizeY = fabs(m_maxY-m_minY)*m_oneDivRes;
		m_gridSizeZ = fabs(m_maxZ-m_minZ)*m_oneDivRes;
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;
		m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		m_gridTrilinear = (TrilinearParams *)malloc(m_gridSize*sizeof(TrilinearParams));
		m_gridDist = (float *)malloc(m_gridSize*sizeof(float));
		m_gridMask = (uint32_t *)malloc(m_gridSize*sizeof(uint32_t));
		m_isUpdated = (uint8_t *)malloc(m_gridSize*sizeof(uint8_t));
		m_isOccupied = (uint8_t *)malloc(m_gridSize*sizeof(uint8_t));

		// Constants for fast indexing
		m_k1 = (uint64_t) m_gridSizeX;
		m_k2 = (uint64_t)(m_gridSizeX*m_gridSizeY);

		// Clear buffers
		clear();
	}

	void clear(void)
	{
		std::memset(m_isOccupied, 0, m_gridSize*sizeof(uint8_t));
		std::memset(m_gridMask, -1, m_gridSize*sizeof(uint32_t));
		const float *pEnd = m_gridDist+m_gridSize;
		float *p = m_gridDist;
		for( ; p<pEnd; )
			*(p++) = 32.0;	
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

	void loadCloud(std::vector<pcl::PointXYZ> &cloud)
	{
		// Creates the manhatan distance mask kernel
		// The point is centered into 3D kernel
		int k = 0;
		uint32_t kernel[21*21*21];
		for(int z=-10; z<=10; z++)
			for(int y=-10; y<=10; y++)
				for(int x=-10; x<=10; x++)
					kernel[k++] = ((uint32_t)0xffffffff) >> (32 - abs(x) - abs(y) - abs(z));
 
		// Applies the pre-computed kernel to all grid cells centered in the cloud points 
		const float step = 10*m_resolution;
		#pragma omp parallel for num_threads(20) shared(m_gridMask, m_isUpdated) 
		for(uint32_t i=0; i<cloud.size(); i++)
		{

			// --------------------------------------------------------------
			
			if(!isIntoGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step) || 
			   !isIntoGrid(cloud[i].x+step, cloud[i].y+step, cloud[i].z+step))
				continue;
				
			uint64_t index = pointToGrid(cloud[i].x,cloud[i].y,cloud[i].z);
			if(m_gridMask[index] == 0)
				continue;
			// ---------------------------------------------------------------

			int xi, yi, zi, k = 0;
			uint64_t idx, idy, idz = pointToGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step);
			for(zi=0; zi<21; zi++, idz+=m_gridStepZ)
				for(yi=0, idy=idz; yi<21; yi++, idy+=m_gridStepY)
					for(xi=0, idx=idy; xi<21; xi++)
					{
						m_gridMask[idx] &= kernel[k++];  
						m_isUpdated[idx++] = 1;
					}
		}
		#pragma omp barrier

		// Computes manhatan distance based on the mask value
		// We do not set m_gridDist, m_isOccupied and m_isUpdated as "shared" because evecty cell 
		// modification ONLY affects to such cells.
		#pragma omp parallel for num_threads(20) 
		for(uint64_t j=0; j<m_gridSize; j++)
			if(m_isUpdated[j] == 1)
			{
				m_gridDist[j] = std::bitset<32>(m_gridMask[j]).count();	
				m_isOccupied[j] = 1;
				m_isUpdated[j] = 0; // Mark again as not updated for future updates
			}
		#pragma omp barrier
	}

	bool computeTrilinearInterpolation(void)
	{
		// Compute the distance to the closest point of the grid
		const double div = -1.0/(m_resolution*m_resolution*m_resolution);
		#pragma omp parallel for num_threads(20) shared(m_gridTrilinear)
		for(int iz=0; iz<m_gridSizeZ-1; iz++)
		{
			int ix, iy;
			double x0, y0, z0, x1, y1, z1;
			z0 = m_minZ + iz*m_resolution;
			z1 = z0+m_resolution;
			for(iy=0, y0=m_minY, y1=m_minY+m_resolution; iy<m_gridSizeY-1; iy++, y0+=m_resolution, y1+=m_resolution)
			{
				for(ix=0, x0=m_minX, x1=m_minX+m_resolution; ix<m_gridSizeX-1; ix++, x0+=m_resolution, x1+=m_resolution)
				{
					double c000, c001, c010, c011, c100, c101, c110, c111;
					TrilinearParams p;
					//if(!m_isUpdated[(ix+0) + (iy+0)*m_gridStepY + (iz+0)*m_gridStepZ])
					//	continue;

					c000 = m_gridDist[(ix+0) + (iy+0)*m_gridStepY + (iz+0)*m_gridStepZ];
					c001 = m_gridDist[(ix+0) + (iy+0)*m_gridStepY + (iz+1)*m_gridStepZ];
					c010 = m_gridDist[(ix+0) + (iy+1)*m_gridStepY + (iz+0)*m_gridStepZ];
					c011 = m_gridDist[(ix+0) + (iy+1)*m_gridStepY + (iz+1)*m_gridStepZ];
					c100 = m_gridDist[(ix+1) + (iy+0)*m_gridStepY + (iz+0)*m_gridStepZ];
					c101 = m_gridDist[(ix+1) + (iy+0)*m_gridStepY + (iz+1)*m_gridStepZ];
					c110 = m_gridDist[(ix+1) + (iy+1)*m_gridStepY + (iz+0)*m_gridStepZ];
					c111 = m_gridDist[(ix+1) + (iy+1)*m_gridStepY + (iz+1)*m_gridStepZ];
					
					p.a0 = (-c000*x1*y1*z1 + c001*x1*y1*z0 + c010*x1*y0*z1 - c011*x1*y0*z0 
					+ c100*x0*y1*z1 - c101*x0*y1*z0 - c110*x0*y0*z1 + c111*x0*y0*z0)*div;
					p.a1 = (c000*y1*z1 - c001*y1*z0 - c010*y0*z1 + c011*y0*z0
					- c100*y1*z1 + c101*y1*z0 + c110*y0*z1 - c111*y0*z0)*div;
					p.a2 = (c000*x1*z1 - c001*x1*z0 - c010*x1*z1 + c011*x1*z0 
					- c100*x0*z1 + c101*x0*z0 + c110*x0*z1 - c111*x0*z0)*div;
					p.a3 = (c000*x1*y1 - c001*x1*y1 - c010*x1*y0 + c011*x1*y0 
					- c100*x0*y1 + c101*x0*y1 + c110*x0*y0 - c111*x0*y0)*div;
					p.a4 = (-c000*z1 + c001*z0 + c010*z1 - c011*z0 + c100*z1 
					- c101*z0 - c110*z1 + c111*z0)*div;
					p.a5 = (-c000*y1 + c001*y1 + c010*y0 - c011*y0 + c100*y1 
					- c101*y1 - c110*y0 + c111*y0)*div;
					p.a6 = (-c000*x1 + c001*x1 + c010*x1 - c011*x1 + c100*x0 
					- c101*x0 - c110*x0 + c111*x0)*div;
					p.a7 = (c000 - c001 - c010 + c011 - c100
					+ c101 + c110 - c111)*div;

					m_gridTrilinear[ix + iy*m_gridStepY + iz*m_gridStepZ] = p;
				}
			}
		}
		#pragma omp barrier

		return true;
	}

	inline TrilinearParams getDistInterpolation(const double &x, const double &y, const double &z)
	{
		TrilinearParams r;
		if(isIntoGrid(x, y, z))
		{	
			uint64_t i = pointToGrid(x, y, z); 
			if(m_isOccupied[i])
				r = m_gridTrilinear[i];	
		}
		return r;
	}

	inline TrilinearParams computeDistInterpolation(const double &x, const double &y, const double &z)
	{
		TrilinearParams r;

		// Get 3D point index
		uint64_t i = pointToGrid(x, y, z); 
		uint64_t j = i+1+m_gridStepY+m_gridStepZ;
		
		// Compute trilateration if we are into the grid
		if(i >= 0 && j<m_gridSize)
		{
			// Get neightbour values to compute trilinear interpolation
			float c000, c001, c010, c011, c100, c101, c110, c111;
			c000 = m_gridDist[i]; 
			c001 = m_gridDist[i+m_gridStepZ]; 
			c010 = m_gridDist[i+m_gridStepY]; 
			c011 = m_gridDist[i+m_gridStepY+m_gridStepZ]; 
			c100 = m_gridDist[i+1]; 
			c101 = m_gridDist[i+1+m_gridStepZ]; 
			c110 = m_gridDist[i+1+m_gridStepY]; 
			c111 = m_gridDist[i+1+m_gridStepY+m_gridStepZ]; 

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

	inline double getDist(const double &x, const double &y, const double &z)
	{
		float r = 0.0;
		if(isIntoGrid(x, y, z))
		{
			uint64_t i = pointToGrid(x, y, z); 
			if(m_isOccupied[i])
				r = m_gridDist[i];		
		}

		return r;
	}

	void buildGridSliceMsg(float z, nav_msgs::msg::OccupancyGrid &gridSliceMsg)
	{		
		// Setup grid msg
		// Crear una instancia de rclcpp::Clock
		rclcpp::Clock clock(RCL_SYSTEM_TIME);

		// Asignar el tiempo actual al campo map_load_time
		gridSliceMsg.info.map_load_time = clock.now();
		gridSliceMsg.info.resolution = m_resolution;
		gridSliceMsg.info.width = m_gridSizeX;
		gridSliceMsg.info.height = m_gridSizeY;
		gridSliceMsg.info.origin.position.x = m_minX;
		gridSliceMsg.info.origin.position.y = m_minY;
		gridSliceMsg.info.origin.position.z = z;
		gridSliceMsg.info.origin.orientation.x = 0.0;
		gridSliceMsg.info.origin.orientation.y = 0.0;
		gridSliceMsg.info.origin.orientation.z = 0.0;
		gridSliceMsg.info.origin.orientation.w = 1.0;
		gridSliceMsg.data.resize(m_gridSizeX*m_gridSizeY);

		// Extract max probability
		int offset = (int)((z-m_minZ)*m_oneDivRes)*m_gridSizeX*m_gridSizeY;
		int end = offset + m_gridSizeX*m_gridSizeY;
		float maxVal = -100000;
		for(int i=offset; i<end; i++)
			if(m_gridDist[i] > maxVal)
				maxVal = m_gridDist[i];

		// Copy data into grid msg and scale the probability to [0,100]
		maxVal = 100.0/maxVal;
		for(int i=0; i<m_gridSizeX*m_gridSizeY; i++)
		{
			if(m_isOccupied[i+offset])
				gridSliceMsg.data[i] = (int8_t)(m_gridDist[i+offset]*maxVal);
			else
				gridSliceMsg.data[i] = 100;
		}
	}
	void exportGridToCSV(const std::string& filename)
	{
		std::ofstream file(filename);
		
		if (!file.is_open())
		{
			std::cerr << "Error: No se pudo abrir el archivo " << filename << std::endl;
			return;
		}

		// Escribir encabezado
		file << "x,y,z,dist,occupied" << std::endl;

		// Iterar sobre la grilla
		for (int iz = 0; iz < m_gridSizeZ; ++iz)
		{
			for (int iy = 0; iy < m_gridSizeY; ++iy)
			{
				for (int ix = 0; ix < m_gridSizeX; ++ix)
				{
					uint64_t index = ix + iy * m_gridStepY + iz * m_gridStepZ;
					float x = m_minX + ix * m_resolution;
					float y = m_minY + iy * m_resolution;
					float z = m_minZ + iz * m_resolution;
					float dist = m_gridDist[index];
					bool occupied = m_isOccupied[index];

					file << std::fixed << std::setprecision(4) 
						<< x << "," << y << "," << z << "," 
						<< dist << "," << occupied << std::endl;
				}
			}
		}

		file.close();
		std::cout << "CSV exportado correctamente: " << filename << std::endl;
	}


	inline bool isOccupied(const float &x, const float &y, const float &z)
	{
		if(isIntoGrid(x, y, z))
			return m_isOccupied[pointToGrid(x, y, z)];
		else
			return false;
	}


protected:

	uint32_t *m_gridMask;				// Binary mask for Manhatan distance
	uint8_t *m_isUpdated;				// 1 if corresponding cell was updated, 0 otherwise
	uint8_t *m_isOccupied;				// 1 if the corresponding cell is occupied, 0 otherwise
};	


#endif
