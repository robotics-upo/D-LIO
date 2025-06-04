#ifndef __TDF3D_64_HPP__
#define __TDF3D_64_HPP__

#include <algorithm>  
#include <bitset>
#include <dlo3d/df3d.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "rclcpp/clock.hpp"

class TDF3D64 : public DF3D
{
	
public:

	TDF3D64(void) 
	{
		m_isUpdated = NULL;
		m_isOccupied = NULL;
		m_gridMask64 = NULL;
		m_gridDist = NULL;
		
		// Creates cloud loading thread
		m_running = true;
		m_updating = false;
		m_newCloud = false;
		boost::thread* m_thread = new boost::thread(boost::bind(&TDF3D64::mapUpdateThread, this));
	}

	~TDF3D64(void)
	{
		if(m_isUpdated != NULL)
			free(m_isUpdated);
		m_isUpdated = NULL;
		if(m_isOccupied != NULL)
			free(m_isOccupied);
		m_isOccupied = NULL;
		if(m_gridMask64 != NULL)
			free(m_gridMask64);
		m_gridMask64 = NULL;
		if(m_gridDist != NULL)
			free(m_gridDist);
		m_gridDist = NULL;
		m_running = false;
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
		if(m_gridDist != NULL)
			free(m_gridDist);
		if(m_isUpdated != NULL)
			free(m_isUpdated);
		if(m_isOccupied != NULL)
			free(m_isUpdated);
		if(m_gridMask64 != NULL)
			free(m_gridMask64);
		
		// Memory allocation for the grid
		m_oneDivRes = 1.0/m_resolution;
		m_gridSizeX = fabs(m_maxX-m_minX)*m_oneDivRes;
		m_gridSizeY = fabs(m_maxY-m_minY)*m_oneDivRes;
		m_gridSizeZ = fabs(m_maxZ-m_minZ)*m_oneDivRes;
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;
		m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		m_gridDist = (float *)malloc(m_gridSize*sizeof(float));
		m_gridMask64 = (uint64_t *)malloc(m_gridSize*sizeof(uint64_t));
		m_isUpdated = (uint8_t *)malloc(m_gridSize*sizeof(uint8_t));
		m_isOccupied = (uint8_t *)malloc(m_gridSize*sizeof(uint8_t));

		// Constants for fast indexing
		m_k1 = (uint64_t) m_gridSizeX;
		m_k2 = (uint64_t)(m_gridSizeX*m_gridSizeY);

		// Clear buffers
		clear();
	}

	void setupGridLimits(float minX, float maxX, float minY, float maxY, float minZ, float maxZ){
		m_maxX = maxX;
		m_maxY = maxY;
		m_maxZ = maxZ;
		m_minX = minX;
		m_minY = minY;
		m_minZ = minZ;
	}

	void clear(void)
	{
		// Wait until last update is performed
		while(m_updating)
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds{2});
		}
		std::memset(m_isOccupied, 0, m_gridSize*sizeof(uint8_t));
		std::memset(m_gridMask64, -1, m_gridSize*sizeof(uint64_t));
		const float *pEnd = m_gridDist+m_gridSize;
		float *p = m_gridDist;
		for( ; p<pEnd; )
			*(p++) = 64;
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
		// Clear update flag array
		std::memset(m_isUpdated, 0, m_gridSize*sizeof(uint8_t));

		// Creates the manhatan distance mask kernel
		// The point is centered into 3D kernel
		int k = 0;
		uint64_t kernel[41*41*41];
		for(int z=-20; z<=20; z++)
			for(int y=-20; y<=20; y++)
				for(int x=-20; x<=20; x++)
					kernel[k++] = ((uint64_t)0xffffffffffffffff) >> (64 - abs(x) - abs(y) - abs(z));
		// Applies the pre-computed kernel to all grid cells centered in the cloud points 
		const float step = 20*m_resolution;
		#pragma omp parallel for num_threads(20) shared(m_gridMask64, m_isUpdated) 
		for(uint32_t i=0; i<cloud.size(); i++)
		{
			
			// ---------------------------------------------------------------
			if(!isIntoGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step) || 
			   !isIntoGrid(cloud[i].x+step, cloud[i].y+step, cloud[i].z+step))
				continue;
			
			// -------------------------------------------------------------------
			uint64_t index = pointToGrid(cloud[i].x,cloud[i].y,cloud[i].z);
			if(m_gridMask64[index] == 0)
				continue;

			int xi, yi, zi, k = 0;
			uint64_t idx, idy, idz = pointToGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step);
			for(zi=0; zi<41; zi++, idz+=m_gridStepZ)
				for(yi=0, idy=idz; yi<41; yi++, idy+=m_gridStepY)
					for(xi=0, idx=idy; xi<41; xi++)
					{
						m_gridMask64[idx] &= kernel[k++];  
						m_isUpdated[idx++] = 1;
					}
		}
		#pragma omp barrier
		// Computes manhatan distance based on the mask value
		#pragma omp parallel for num_threads(20) 
		for(uint64_t j=0; j<m_gridSize; j++)
			if(m_isUpdated[j])
			{
				float d = std::bitset<64>(m_gridMask64[j]).count();
				m_gridDist[j] = d;	
				m_isOccupied[j] = 1;
			}
		#pragma omp barrier
	}

	void loadCloudThreaded(std::vector<pcl::PointXYZ> &cloud, float tx, float ty, float tz, float roll, float pitch, float yaw)
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
		
		m_cloud = out;
		m_newCloud = true;
	}

	void loadCloudThreaded(std::vector<pcl::PointXYZ> &cloud, float tx, float ty, float tz, float yaw)
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
		
		m_cloud = out;
		m_newCloud = true;
	}

	void loadCloudThreaded(std::vector<pcl::PointXYZ> &cloud)
	{
		m_cloud = cloud;
		m_newCloud = true;
	}

	float getD(int index){

		return m_gridDist[index];

	}

	inline TrilinearParams getDistInterpolation(const double &x, const double &y, const double &z)
	{
		TrilinearParams r;
		if(isIntoGrid(x, y, z))
		{	
			uint64_t i = pointToGrid(x, y, z); 
			if(m_isOccupied[i])
			{
				r = m_gridTrilinear[i];	
			}	
		}
		return r;
	}

	inline TrilinearParams computeDistInterpolation(const double &x, const double &y, const double &z)
	{
		TrilinearParams r;

		if(isIntoGrid(x, y, z))
		{
			// Get 3D point index
			uint64_t i = pointToGrid(x, y, z); 

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
		rclcpp::Clock clock(RCL_SYSTEM_TIME);
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

	void exportGridToCSV(const std::string& filename, 
                     float minX_filter, float maxX_filter, 
                     float minY_filter, float maxY_filter, 
                     float minZ_filter, float maxZ_filter,
                     int subsampling_factor) // Factor de submuestreo (mismo en x, y, z)
{
    std::ofstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    // Escribir encabezado
    file << "x,y,z,dist,occupied" << std::endl;

    // Iterar sobre la grilla con el mismo paso en x, y, z
    for (int iz = 0; iz < m_gridSizeZ; iz += subsampling_factor)
    {
        float z = m_minZ + iz * m_resolution;
        if (z < minZ_filter || z > maxZ_filter) continue; // Filtrar por Z

        for (int iy = 0; iy < m_gridSizeY; iy += subsampling_factor)
        {
            float y = m_minY + iy * m_resolution;
            if (y < minY_filter || y > maxY_filter) continue; // Filtrar por Y

            for (int ix = 0; ix < m_gridSizeX; ix += subsampling_factor)
            {
                float x = m_minX + ix * m_resolution;
                if (x < minX_filter || x > maxX_filter) continue; // Filtrar por X

                uint64_t index = ix + iy * m_gridStepY + iz * m_gridStepZ;
                float dist = m_gridDist[index];
                bool occupied = m_isOccupied[index];
				if(occupied == 1.0){
					 file << std::fixed << std::setprecision(4) 
                    << x << "," << y << "," << z << "," 
                    << dist << "," << occupied << std::endl;
				}

               
            }
        }
    }

    file.close();
    std::cout << "CSV exported successfully: " << filename << std::endl;
}


void exportGridToPCD(const std::string& filename, int subsampling_factor)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (int iz = 0; iz < m_gridSizeZ; iz += subsampling_factor)
    {
        float z = m_minZ + iz * m_resolution;
        for (int iy = 0; iy < m_gridSizeY; iy += subsampling_factor)
        {
            float y = m_minY + iy * m_resolution;
            for (int ix = 0; ix < m_gridSizeX; ix += subsampling_factor)
            {
                float x = m_minX + ix * m_resolution;

                uint64_t index = ix + iy * m_gridStepY + iz * m_gridStepZ;
                if (!m_isOccupied[index]) continue; 
                float dist = m_gridDist[index];

                float maxVal = 100.0f;  
                float prob = (dist / maxVal) * 100.0f;

                if (prob <= 1.0f) 
                {
                    pcl::PointXYZI point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    point.intensity = prob;

                    cloud->push_back(point);
                }
            }
        }
    }

    if (cloud->empty())
    {
        std::cerr << "Empty Cloud." << std::endl;
        return;
    }

    pcl::io::savePCDFileBinary(filename, *cloud);
    std::cout << "PCD exported successfully: " << filename << std::endl;
}

inline bool isOccupied(const float &x, const float &y, const float &z)
{
	if(isIntoGrid(x, y, z))
		return m_isOccupied[pointToGrid(x, y, z)];
	else
		return false;
}


protected:

	uint64_t *m_gridMask64;				// Binary mask for Manhatan distance (64 bit mask)
	uint8_t *m_isUpdated;				// 1 if corresponding cell was updated, 0 otherwise
	uint8_t *m_isOccupied;				// 1 if the corresponding cell is occupied, 0 otherwise
	boost::thread *m_thread;			// Cloud loading thread
	
	std::vector<pcl::PointXYZ> m_cloud;	// Cloud to process
	bool m_running, m_newCloud, m_updating;

	void mapUpdateThread(void)
	{
		std::vector<pcl::PointXYZ> cloud;

		while(m_running)
		{
			if(!m_newCloud)
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds{2});
				continue;
			}

			// Get a copy of the point-cloud
			cloud = m_cloud;
			m_newCloud = false;
			m_updating = true;

			// Clear update flag array
			std::memset(m_isUpdated, 0, m_gridSize*sizeof(uint8_t));

			// Creates the manhatan distance mask kernel
			// The point is centered into 3D kernel
			int k = 0;
			uint64_t kernel[41*41*41];
			for(int z=-20; z<=20; z++)
				for(int y=-20; y<=20; y++)
					for(int x=-20; x<=20; x++)
						kernel[k++] = ((uint64_t)0xffffffffffffffff) >> (64 - abs(x) - abs(y) - abs(z));
	
			// Applies the pre-computed kernel to all grid cells centered in the cloud points 
			const float step = 20*m_resolution;
			#pragma omp parallel for num_threads(20) shared(m_gridMask64, m_isUpdated) 
			for(uint32_t i=0; i<cloud.size(); i++)
			{
				if(!isIntoGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step) || 
				!isIntoGrid(cloud[i].x+step, cloud[i].y+step, cloud[i].z+step))
					continue;

				int xi, yi, zi, k = 0;
				uint64_t idx, idy, idz = pointToGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step);
				for(zi=0; zi<41; zi++, idz+=m_gridStepZ)
					for(yi=0, idy=idz; yi<41; yi++, idy+=m_gridStepY)
						for(xi=0, idx=idy; xi<41; xi++)
						{
							m_gridMask64[idx] &= kernel[k++];  
							m_isUpdated[idx++] = 1;
						}
			}
			#pragma omp barrier

			// Computes manhatan distance based on the mask value
			#pragma omp parallel for num_threads(20) 
			for(uint64_t j=0; j<m_gridSize; j++)
				if(m_isUpdated[j])
				{
					float d = std::bitset<64>(m_gridMask64[j]).count();
					m_gridDist[j] = d;	
					m_isOccupied[j] = 1;
				}
			#pragma omp barrier
					
			m_updating = false;
		}
	}
};	


#endif
