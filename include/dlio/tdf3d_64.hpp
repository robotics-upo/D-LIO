#ifndef __TDF3D_64_HPP__
#define __TDF3D_64_HPP__

#include <algorithm>  
#include <bitset>
#include "nav_msgs/msg/occupancy_grid.hpp"

// TrilinearParams: inlined from df3d.hpp to avoid pulling in unused ANN/KD-tree dependencies
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
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "rclcpp/clock.hpp"
#include <dlio/grid_64.hpp>

// Bit layout for uint64_t voxel mask:
//   bits [0..34]  : distance mask (popcount = distance to nearest surface)
//   bits [35..63] : hit counter   (number of scans that confirmed this voxel)
//
// The kernel AND operation preserves the upper bits because the kernel values
// have bits 35-63 set to 1; the hit counter is updated separately.
static constexpr uint64_t DIST_MASK    = 0x00000007FFFFFFFFULL;  // bits  0-34
static constexpr uint64_t HIT_MASK     = 0xFFFFFFF800000000ULL;  // bits 35-63
static constexpr int      HIT_SHIFT    = 35;
static constexpr uint64_t HIT_SENTINEL = 0x1FFFFFFFULL;          // all hit bits = 1 (memset -1 init)

/// Extract the hit counter from a raw voxel value.
/// Returns 0 for uninitialized voxels (sentinel = all hit bits set from memset -1).
static inline int extractHits(uint64_t raw)
{
	uint64_t h = (raw >> HIT_SHIFT) & HIT_SENTINEL;
	return (h == HIT_SENTINEL) ? 0 : static_cast<int>(h);
}

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
		m_firstLoad = true;
		
		int k = 0;

		for(int z=-20; z<=20; z++){
			for(int y=-20; y<=20; y++){
				for(int x=-20; x<=20; x++){
					
					// 1. Euclidean L2 distance
					float dist_euclidiana = std::sqrt(x*x + y*y + z*z);
					int d = std::round(dist_euclidiana);
					
					// 2. Clamp to usable distance range (bits 0-34)
					if (d > 34) d = 34; 

					if (d == 0) {
						// Center: zero all distance bits, preserve hit bits
						kernel[k++] = HIT_MASK;
					} else {
						// Set d distance bits + preserve hit bits
						kernel[k++] = (((uint64_t)0xffffffffffffffff) >> (64 - d)) | HIT_MASK;
					}
				}
			}
		}
	}

	~TDF3D64(void){}

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
		m_firstLoad = true;
	}

	void exportGridToPCD(const std::string& filename, int subsampling_factor)
	{

		m_grid.exportGridToPCD(filename, subsampling_factor);
	}

	void exportMesh(const std::string& filename, float iso_level)
	{

		m_grid.exportMesh(filename, iso_level);
	}

	virtual inline bool isIntoGrid(const float &x, const float &y, const float &z)
	{
		return (x > m_minX+1 && y > m_minY+1 && z > m_minZ+1 && x < m_maxX-1 && y < m_maxY-1 && z < m_maxZ-1);
	}

	float getResolution() const { return m_resolution; }
	float getMinX() const { return m_minX; }
	float getMaxX() const { return m_maxX; }
	float getMinY() const { return m_minY; }
	float getMaxY() const { return m_maxY; }
	float getMinZ() const { return m_minZ; }
	float getMaxZ() const { return m_maxZ; }
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr extractPointCloudFromGrid(int subsampling_factor = 1)
	{
		return m_grid.extractPointCloud(subsampling_factor);
	}

	/// Read the hit counter (observation count) for a voxel at world coordinates.
	/// Returns 0 for uninitialized/unobserved voxels.
	inline int readHits(float x, float y, float z)
	{
		if(isIntoGrid(x, y, z))
			return extractHits(m_grid.read(x, y, z));
		return 0;
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
		out.reserve(cloud.size()); 

		float max_dist_sq = max_dist_xy * max_dist_xy; 

		for(size_t i = 0; i < cloud.size(); ++i)
		{
			float x = cloud[i].x;
			float y = cloud[i].y;

			if(x*x + y*y <= max_dist_sq)
			{
				out.push_back(cloud[i]);
			}
		}

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

		for(size_t i = 0; i < out.size(); ++i)
		{
			float x = out[i].x;
			float y = out[i].y;
			float z = out[i].z;

			out[i].x = x*r00 + y*r01 + z*r02 + tx;
			out[i].y = x*r10 + y*r11 + z*r12 + ty;
			out[i].z = x*r20 + y*r21 + z*r22 + tz;
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
		// Allocate required submetric cells
		for(uint32_t i=0; i<cloud.size(); i++)
		{
			for(float z=cloud[i].z-2; z<=cloud[i].z+2; z+=1)
				for(float y=cloud[i].y-2; y<=cloud[i].y+2; y+=1)
					for(float x=cloud[i].x-2; x<=cloud[i].x+2; x+=1)
						if(isIntoGrid(x, y, z))
							m_grid.allocCell(x, y, z);
		}

		// Cache first-load flag (read once, shared across threads)
		const bool is_first_load = m_firstLoad;

		// Applies the pre-computed kernel to all grid cells centered in the cloud points 
		const float step = 20*m_resolution;
		#pragma omp parallel for num_threads(20) shared(m_grid) 
		for(uint32_t i=0; i<cloud.size(); i++)
		{
			
			if(!isIntoGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step) || 
			   !isIntoGrid(cloud[i].x+step, cloud[i].y+step, cloud[i].z+step))
				continue;

			// Hit-counter logic
			// Read center voxel to check observation history
			uint64_t center_val = m_grid.read(cloud[i].x, cloud[i].y, cloud[i].z);
			int hits = extractHits(center_val);

			// On non-first loads, skip kernel for never-observed voxels (dynamic filtering)
			if (!is_first_load && hits == 0) {
				// Mark as seen once (hit_count = 1) but don't apply the kernel,
				// so the distance field stays untouched and transient points are excluded.
				uint64_t dist_bits = center_val & DIST_MASK;
				m_grid(cloud[i].x, cloud[i].y, cloud[i].z) = (1ULL << HIT_SHIFT) | dist_bits;
				continue;
			}

			// Skip if center already at distance 0 (only check distance bits)
			if((center_val & DIST_MASK) == 0)
			{
				// Still increment hit counter even if distance is already 0
				uint64_t new_hits = std::min((uint64_t)(hits + 1), HIT_SENTINEL - 1);
				m_grid(cloud[i].x, cloud[i].y, cloud[i].z) = (new_hits << HIT_SHIFT);  // dist=0
				continue;
			}

			// Apply 41^3 kernel (hot loop)
			int xi, yi, zi, k = 0;
			float x, y, z;
			for(zi=0, z=cloud[i].z-step; zi<41; zi++, z+=m_resolution)
				for(yi=0, y=cloud[i].y-step; yi<41; yi++, y+=m_resolution){
						GRID64::Iterator it = m_grid.getIterator(cloud[i].x-step,y,z);
					for(xi=0, x=cloud[i].x-step; xi<41; xi++, x+=m_resolution,++it){
						*it &= kernel[k++];
					}
				}

			// Update hit counter at center voxel
			center_val = m_grid.read(cloud[i].x, cloud[i].y, cloud[i].z);
			uint64_t dist_bits = center_val & DIST_MASK;
			uint64_t new_hits = std::min((uint64_t)(hits + 1), HIT_SENTINEL - 1);
			m_grid(cloud[i].x, cloud[i].y, cloud[i].z) = (new_hits << HIT_SHIFT) | dist_bits;
		}

		m_firstLoad = false;
	}

inline TrilinearParams computeDistInterpolation(const double &x, const double &y, const double &z)
{
    TrilinearParams r;

    if(isIntoGrid(x, y, z))
    {
        // Neighbour values, masked to distance bits, via hardware popcount
        float c000, c001, c010, c011, c100, c101, c110, c111;
        c000 = __builtin_popcountll(m_grid.read(x, y, z) & DIST_MASK); 
        c001 = __builtin_popcountll(m_grid.read(x, y, z+m_resolution) & DIST_MASK); 
        c010 = __builtin_popcountll(m_grid.read(x, y+m_resolution, z) & DIST_MASK); 
        c011 = __builtin_popcountll(m_grid.read(x, y+m_resolution, z+m_resolution) & DIST_MASK);  
        c100 = __builtin_popcountll(m_grid.read(x+m_resolution, y, z) & DIST_MASK);  
        c101 = __builtin_popcountll(m_grid.read(x+m_resolution, y, z+m_resolution) & DIST_MASK);  
        c110 = __builtin_popcountll(m_grid.read(x+m_resolution, y+m_resolution, z) & DIST_MASK);  
        c111 = __builtin_popcountll(m_grid.read(x+m_resolution, y+m_resolution, z+m_resolution) & DIST_MASK); 

        // Compute trilinear parameters
        const float div = -m_oneDivRes*m_oneDivRes*m_oneDivRes;
        float x0, y0, z0, x1, y1, z1;
        x0 = std::floor(x * m_oneDivRes) * m_resolution;
        x1 = x0 + m_resolution;
        y0 = std::floor(y * m_oneDivRes) * m_resolution;
        y1 = y0 + m_resolution;
        z0 = std::floor(z * m_oneDivRes) * m_resolution;
        z1 = z0 + m_resolution;
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

	float m_maxX, m_maxY, m_maxZ;
	float m_minX, m_minY, m_minZ;
	float m_resolution, m_oneDivRes;	
	uint64_t kernel[41*41*41];
	GRID64 m_grid;
	bool m_firstLoad;	// True until first loadCloud completes after clear()
};	


#endif