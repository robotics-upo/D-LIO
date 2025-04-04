#ifndef __DF3D_HPP__
#define __DF3D_HPP__

#include <stdlib.h>
#include <vector>
#include <cstring>
#include <ANN/ANN.h>
#include <pcl/point_cloud.h>


struct TrilinearParams
{
	float a0, a1, a2, a3, a4, a5, a6, a7;

	TrilinearParams(void)
	{
		a0 = a1 = a2 = a3 = a4 = a5 = a6 = a7 = 0.0;
	}

	float interpolate(float x, float y, float z)
	{
		return a0 + a1*x + a2*y + a3*z + a4*x*y + a5*x*z + a6*y*z + a7*x*y*z;
	}
};

class DF3D
{

public:

	DF3D(void) 
	{
	 	m_maxX = 50;
		m_maxY = 50;
		m_maxZ = 10;
		m_minX = -50;
		m_minY = -50;
		m_minZ = -10;
		m_resolution = 0.05;
		m_gridDist = NULL;
		m_gridTrilinear = NULL;
	}

	~DF3D(void)
	{
		if(m_gridTrilinear != NULL)
			free(m_gridTrilinear);
		if(m_gridDist != NULL)
			free(m_gridDist);
		m_gridDist = NULL;
		m_gridTrilinear = NULL;
	}

	virtual void setup(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, float resolution)
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

		// Constants for fast indexing
		m_k1 = (uint64_t) m_gridSizeX;
		m_k2 = (uint64_t)(m_gridSizeX*m_gridSizeY);
	}

	virtual void clear(void)
	{
		std::memset(m_gridTrilinear, 0, m_gridSize*sizeof(TrilinearParams));
		std::memset(m_gridDist, 0, m_gridSize*sizeof(float));
	}

	virtual void loadCloud(std::vector<pcl::PointXYZ> &cloud)
	{
		// Build the object positions array
		ANNpointArray points = annAllocPts(cloud.size(), 3);
		for(int i=0; i<cloud.size(); i++)
		{
			points[i][0] = cloud[i].x;
			points[i][1] = cloud[i].y;
			points[i][2] = cloud[i].z;
		}

		// Build KDtree
		ANNkd_tree* kdTree = new ANNkd_tree(points, cloud.size(), 3);

		// Evaluate distance from all grid-cell to map-points  
		int i, j, k, m = 0;
		float x, y, z;
		double count = 0.0;
		double size = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		ANNpoint queryPt = annAllocPt(3);
		ANNidxArray nnIdx = new ANNidx[1];            
		ANNdistArray dists = new ANNdist[1];
		for(i=0, z=m_minZ; i<m_gridSizeZ; i++, z+=m_resolution)
		{
			printf("Computing distance grid: : %3.2lf%%        \r", count/size * 100.0);
			for(j=0, y=m_minY; j<m_gridSizeY; j++, y+=m_resolution)
			{
				for(k=0, x=m_minX; k<m_gridSizeX; k++, x+=m_resolution, m++)
				{
					queryPt[0] = x;
					queryPt[1] = y;
					queryPt[2] = z;
					kdTree->annkSearch(queryPt, 1, nnIdx, dists);
					m_gridDist[m] = dists[0];
					count++;
				}
			}
		}
		delete kdTree;
		annClose(); 
		printf("Computing distance grid: 100%%                               \n");

		// Computes tilinear interpolation parameters
		computeTrilinearInterpolation();
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
			out[i].x = cloud[i].x*r00 + cloud[i].y*r01 + cloud[i].z*r02;
			out[i].y = cloud[i].x*r10 + cloud[i].y*r11 + cloud[i].z*r12;
			out[i].z = cloud[i].x*r20 + cloud[i].y*r21 + cloud[i].z*r22;	
		}
		loadCloud(out);
	}
	
	virtual inline bool isIntoGrid(const float &x, const float &y, const float &z)
	{
		return (x > m_minX+1 && y > m_minY+1 && z > m_minZ+1 && x < m_maxX-2 && y < m_maxY-2 && z < m_maxZ-2);
	}

	virtual inline bool isIntoGrid(const uint64_t &index)
	{
		return (index >= 0 && index < m_gridSize);
	}

	virtual inline bool isOccupied(const float &x, const float &y, const float &z)
	{
		return true;
	}

	virtual inline double getDist(const double &x, const double &y, const double &z)
	{
		float r = 0.0;
		if(isIntoGrid(x, y, z))
			r = m_gridDist[pointToGrid(x, y, z)];
		return r;
	}

	virtual inline TrilinearParams getDistInterpolation(const double &x, const double &y, const double &z)
	{
		TrilinearParams r;
		if(isIntoGrid(x, y, z))
			r = m_gridTrilinear[pointToGrid(x, y, z)];
		return r;
	}

	virtual inline TrilinearParams computeDistInterpolation(const double &x, const double &y, const double &z)
	{
		return getDistInterpolation(x, y, z);
	}


protected:

	virtual bool computeTrilinearInterpolation(void)
	{
		// Compute the distance to the closest point of the grid
		int ix, iy, iz;
		double count = 0.0;
		double size = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		double x0, y0, z0, x1, y1, z1;
		double div = -1.0/(m_resolution*m_resolution*m_resolution);
		for(iz=0, z0=m_minZ, z1=m_minZ+m_resolution; iz<m_gridSizeZ-1; iz++, z0+=m_resolution, z1+=m_resolution)
		{
			printf("Computing trilinear interpolation map: : %3.2lf%%        \r", count/size * 100.0);
			for(iy=0, y0=m_minY, y1=m_minY+m_resolution; iy<m_gridSizeY-1; iy++, y0+=m_resolution, y1+=m_resolution)
			{
				for(ix=0, x0=m_minX, x1=m_minX+m_resolution; ix<m_gridSizeX-1; ix++, x0+=m_resolution, x1+=m_resolution)
				{
					double c000, c001, c010, c011, c100, c101, c110, c111;
					TrilinearParams p;
					count++;

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
		printf("Computing trilinear interpolation map: 100%%                               \n");

		return true;
	}
	
	inline uint64_t pointToGrid(const float &x, const float &y, const float &z)
	{
		return (uint64_t)((x-m_minX)*m_oneDivRes) + (uint64_t)((y-m_minY)*m_oneDivRes)*m_k1 + (uint64_t)((z-m_minZ)*m_oneDivRes)*m_k2;
	}

	// 3D grid information
	TrilinearParams *m_gridTrilinear;	// Trilinear parameters interpolation
	float *m_gridDist;					// Distance grid
	uint64_t m_gridSize; 
	uint32_t m_gridSizeX, m_gridSizeY, m_gridSizeZ;
	uint32_t m_gridStepY, m_gridStepZ;
	float m_maxDist;					// Maximum distance into the grid

	// Grid parameters
	float m_maxX, m_maxY, m_maxZ;
	float m_minX, m_minY, m_minZ;
	float m_resolution, m_oneDivRes;	

	// Internal parameters
	uint64_t m_k1, m_k2;
};	


#endif
