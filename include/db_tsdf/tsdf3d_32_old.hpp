#ifndef __TDF3D_32_HPP__
#define __TDF3D_32_HPP__

// TDF3D32: 32-bit directional TSDF grid with per-voxel Manhattan-mask distance,
// sign bit, and hit counter. Supports directional kernels, TSDF updates,
// distance queries, slice visualization, and CSV/PCD/PLY exports.

#include <algorithm>          
#include <cmath>          
#include <chrono>           
#include <atomic>          
#include <fstream>          

#include "rclcpp/clock.hpp"
#include <rclcpp/rclcpp.hpp> 
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <Eigen/Dense>		

#include <db_tsdf/df3d.hpp>    

// Binning for directional kernels
constexpr int BINS_AZ  = 40;		// azimuth bins               
constexpr int BINS_EL  = 40;		// elevation bins
constexpr int NUM_BINS = BINS_AZ * BINS_EL;  

constexpr int OCC_MIN_HITS = 50;		// hits threshold to mark occupied
constexpr int SHADOW_RADIUS_MD = 8; 	// shadow radius (voxel units)

// Direction-dependent kernel (21³ support)
struct DirectionalKernel
{
    uint32_t distance_masks[21*21*21];		// Manhattan distance masks
    uint8_t  signs[21*21*21];		        // 0 = occ, 1 = free
};

// 8-byte voxel record
struct VoxelData
{
	uint32_t d;		// Manhattan mask (bit-count -> distance)
	uint8_t s;		// bit0: sign (0 occ / 1 free)
	uint16_t hits;	// hit counter
};
static_assert(sizeof(VoxelData) == 8, "VoxelData must be 8-bytes aligned");

// Grid class
class TDF3D32 : public DF3D
{
public:
	TDF3D32() : m_gridData(nullptr) {}
    ~TDF3D32() { free(m_gridData); }

	// Allocate grid and reset all state
	void setup(float minX, float maxX, 
			   float minY, float maxY, 
			   float minZ, float maxZ, 
			   float resolution)
	{
		m_maxX = maxX;	m_maxY = maxY;
		m_maxZ = maxZ;	m_minX = minX;
		m_minY = minY;	m_minZ = minZ;
		m_resolution = resolution;

		free(m_gridData);	//reset

    	// Dimensions and strides
		m_oneDivRes = 1.0/m_resolution;
		m_gridSizeX = fabs(m_maxX-m_minX)*m_oneDivRes;
		m_gridSizeY = fabs(m_maxY-m_minY)*m_oneDivRes;
		m_gridSizeZ = fabs(m_maxZ-m_minZ)*m_oneDivRes;
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;
		m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		m_gridData = (VoxelData *)malloc(m_gridSize*sizeof(VoxelData));

    	// Fast index helpers
		m_k1 = (uint64_t) m_gridSizeX;
		m_k2 = (uint64_t)(m_gridSizeX*m_gridSizeY);

		// Clear buffers
		initDirectionalKernels();
		clear();
	}

	// Update only the world limits
	void setupGridLimits(float minX, float maxX, 
						 float minY, float maxY, 
						 float minZ, float maxZ){
		m_maxX = maxX;	m_maxY = maxY;
		m_maxZ = maxZ;	m_minX = minX;
		m_minY = minY;	m_minZ = minZ;
	}

	// Reset all voxels to free, distance=32, hits=0
	void clear(void)
	{
		for (uint64_t i = 0; i < m_gridSize; ++i)
		{
			m_gridData[i].d = 0xFFFFFFFFu;   // bits set to 1 → distance 32
			m_gridData[i].s = 1;             // 1 = free
			m_gridData[i].hits = 0; 
		}
	}

	// Rigid correction (yaw + translation), then update
	void loadCloud(std::vector<pcl::PointXYZ> &cloud, 
				   float tx, float ty, float tz, float yaw)
	{
		std::vector<pcl::PointXYZ> out(cloud.size());

		const float c = cos(yaw), s = sin(yaw);
		for(uint32_t i=0; i<out.size(); i++)
		{
			out[i].x = c*cloud[i].x - s*cloud[i].y + tx;
			out[i].y = s*cloud[i].x + c*cloud[i].y + ty; 
			out[i].z = cloud[i].z + tz;
		}

		m_last_corrected_cloud = out; 
		loadCloud(out);
	}

	// Full SE(3) correction, then update
	void loadCloud(std::vector<pcl::PointXYZ> &cloud, 
				   float tx, float ty, float tz, 
				   float roll, float pitch, float yaw)
	{
		std::vector<pcl::PointXYZ> out(cloud.size());

    	// Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
		const float cr = std::cos(roll),  sr = std::sin(roll);
		const float cp = std::cos(pitch), sp = std::sin(pitch);
		const float cy = std::cos(yaw),   sy = std::sin(yaw);
		const float r00 = cy*cp,      r01 = cy*sp*sr - sy*cr,  r02 = cy*sp*cr + sy*sr;
		const float r10 = sy*cp,      r11 = sy*sp*sr + cy*cr,  r12 = sy*sp*cr - cy*sr;
		const float r20 = -sp,        r21 = cp*sr,             r22 = cp*cr;

		for(uint i=0; i<cloud.size(); i++) 
		{
			out[i].x = cloud[i].x*r00 + cloud[i].y*r01 + cloud[i].z*r02 + tx;
			out[i].y = cloud[i].x*r10 + cloud[i].y*r11 + cloud[i].z*r12 + ty;
			out[i].z = cloud[i].x*r20 + cloud[i].y*r21 + cloud[i].z*r22 + tz;	
		}

		m_last_corrected_cloud = out; 
		loadCloud(out);
	}

	// Dense TSDF update using 21³ directional kernels around each point
	void loadCloud(std::vector<pcl::PointXYZ> &cloud)
	{
		const auto t0 = std::chrono::steady_clock::now();
		const float step = 10 * m_resolution;			// 20-voxel cube

		#pragma omp parallel for num_threads(20) shared(m_dirKernels, m_gridData) 
		for(uint32_t i=0; i<cloud.size(); i++)
		{
        	// Skip if 21³ neighborhood falls outside the grid
			if(!isIntoGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step) || 
			   !isIntoGrid(cloud[i].x+step, cloud[i].y+step, cloud[i].z+step))
				continue;
			
			// Select kernel by ray direction
			Eigen::Vector3f dir(cloud[i].x, cloud[i].y, cloud[i].z);
    		const DirectionalKernel& DK = m_dirKernels[dirToBin(dir)];

			// Convolve neighborhood
			int xi, yi, zi, k = 0;
			uint64_t idx, idy, idz = pointToGrid(cloud[i].x-step, cloud[i].y-step, cloud[i].z-step);
			for(zi=0; zi<21; zi++, idz+=m_gridStepZ)
				for(yi=0, idy=idz; yi<21; yi++, idy+=m_gridStepY)
					for(xi=0, idx=idy; xi<21; xi++, k++, idx++)
					{
						uint32_t old_mask = m_gridData[idx].d;                    
						uint32_t new_mask = old_mask & DK.distance_masks[k];
						
						if (new_mask != old_mask)
							m_gridData[idx].d = new_mask;

						if (DK.signs[k] == 0) { // occupied evidence
							if (m_gridData[idx].hits < std::numeric_limits<uint16_t>::max()) 
								++m_gridData[idx].hits;
								if (m_gridData[idx].hits == OCC_MIN_HITS)
									m_gridData[idx].s &= uint8_t(~0x01);   // bit0=0 → ocupado
						}
					}
		}

		const auto t1 = std::chrono::steady_clock::now();
		recordKernelTime(std::chrono::duration<double,std::milli>(t1 - t0).count());
	}

	// Distance at linear index
	inline float getD(uint64_t i) const { return voxelDist(i); }

	// Return cached trilinear parameters if voxel is confident
	inline TrilinearParams getDistInterpolation(const double &x, const double &y, const double &z)
	{
		TrilinearParams p{};
		if (isIntoGrid(x,y,z))
		{
			uint64_t i = pointToGrid(x,y,z);
			if (m_gridData[i].hits >= OCC_MIN_HITS)
				p = m_gridTrilinear[i];
		}
		return p;
	}

	// Compute trilinear parameters on the fly
	inline TrilinearParams computeDistInterpolation(const double &x, const double &y, const double &z)
	{
		TrilinearParams p{};
		if (!isIntoGrid(x,y,z)) return p;

		// 8-neighbour values
		uint64_t i = pointToGrid(x,y,z);
		const float c000 = voxelDist(i);
		const float c001 = voxelDist(i + m_gridStepZ);
		const float c010 = voxelDist(i + m_gridStepY);
		const float c011 = voxelDist(i + m_gridStepY + m_gridStepZ);
		const float c100 = voxelDist(i + 1);
		const float c101 = voxelDist(i + 1 + m_gridStepZ);
		const float c110 = voxelDist(i + 1 + m_gridStepY);
		const float c111 = voxelDist(i + 1 + m_gridStepY + m_gridStepZ);

		// Coefficients
		const float div = -m_oneDivRes * m_oneDivRes * m_oneDivRes;
		const float x0  = std::floor(x * m_oneDivRes) * m_resolution;
		const float y0  = std::floor(y * m_oneDivRes) * m_resolution;
		const float z0  = std::floor(z * m_oneDivRes) * m_resolution;
		const float x1  = x0 + m_resolution;
		const float y1  = y0 + m_resolution;
		const float z1  = z0 + m_resolution;

		p.a0 = (-c000*x1*y1*z1 + c001*x1*y1*z0 + c010*x1*y0*z1 - c011*x1*y0*z0
				+ c100*x0*y1*z1 - c101*x0*y1*z0 - c110*x0*y0*z1 + c111*x0*y0*z0) * div;
		p.a1 = ( c000*y1*z1 - c001*y1*z0 - c010*y0*z1 + c011*y0*z0
				- c100*y1*z1 + c101*y1*z0 + c110*y0*z1 - c111*y0*z0) * div;
		p.a2 = ( c000*x1*z1 - c001*x1*z0 - c010*x1*z1 + c011*x1*z0
				- c100*x0*z1 + c101*x0*z0 + c110*x0*z1 - c111*x0*z0) * div;
		p.a3 = ( c000*x1*y1 - c001*x1*y1 - c010*x1*y0 + c011*x1*y0
				- c100*x0*y1 + c101*x0*y1 + c110*x0*y0 - c111*x0*y0) * div;
		p.a4 = (-c000*z1 + c001*z0 + c010*z1 - c011*z0 + c100*z1
				- c101*z0 - c110*z1 + c111*z0) * div;
		p.a5 = (-c000*y1 + c001*y1 + c010*y0 - c011*y0 + c100*y1
				- c101*y1 - c110*y0 + c111*y0) * div;
		p.a6 = (-c000*x1 + c001*x1 + c010*x1 - c011*x1 + c100*x0
				- c101*x0 - c110*x0 + c111*x0) * div;
		p.a7 = ( c000 - c001 - c010 + c011 - c100
				+ c101 + c110 - c111) * div;

		return p;
	}

	// Signed distance at point (0 if outside or not confident)
	inline double getDist(const double &x, const double &y, const double &z)
	{
		if (!isIntoGrid(x,y,z)) return 0.f;
		uint64_t i = pointToGrid(x,y,z);
		return (m_gridData[i].hits >= OCC_MIN_HITS) ? voxelDist(i) : 0.f;
	}

	// XY slice as ROS OccupancyGrid for visualization
	void buildGridSliceMsg(float z0, nav_msgs::msg::OccupancyGrid &msg)
	{
		msg.header.stamp    = rclcpp::Clock(RCL_SYSTEM_TIME).now();
		msg.header.frame_id = "odom";
		msg.info.resolution = m_resolution;
		msg.info.width      = m_gridSizeX;
		msg.info.height     = m_gridSizeY;
		msg.info.origin.position.x = m_minX;
		msg.info.origin.position.y = m_minY;
		msg.info.origin.position.z = z0;
		msg.info.origin.orientation.w = 1.0;

		const int k = int((z0 - m_minZ) * m_oneDivRes + 0.5);
		if (k < 0 || k >= int(m_gridSizeZ)) { msg.data.clear(); return; }
		const std::size_t offset = std::size_t(k) * m_gridStepZ;

		constexpr float  D_MAX = 32.f;
		constexpr int8_t FREE_MIN =   1,    FREE_MAX =  49,      
						 BORDER   =  60,     
						 OCC_MIN  =  61,	OCC_MAX  = 100;
		constexpr float  scale_free = (FREE_MAX - FREE_MIN) / D_MAX;
		constexpr float  scale_occ  = (OCC_MAX - OCC_MIN) / (D_MAX - 1.f);

		msg.data.assign(m_gridSizeX * m_gridSizeY, FREE_MIN);  

		for (std::size_t idx = 0; idx < m_gridStepZ; ++idx) {
			const std::size_t g = offset + idx;
			if (m_gridData[g].hits == 0) continue;                    

			float sdist = ((m_gridData[g].s & 0x01) ? -1.f : 1.f) * voxelDist(g);
			sdist = std::clamp(sdist, -D_MAX, D_MAX);

			if (!(m_gridData[g].hits >= OCC_MIN_HITS)) {
				msg.data[idx] =	int8_t(std::round((D_MAX - sdist) * scale_free));  
			}

			else {
				float d = std::min(std::fabs(sdist), D_MAX);
				if (d <= 1.f)
					msg.data[idx] = BORDER;             
				else
					msg.data[idx] = OCC_MIN +
						int8_t(std::round((d - 1.f) * scale_occ));    
			}
		}
	}

	// Export voxel stats to CSV within XYZ filters (optional subsampling)
	void exportGridToCSV(const std::string& filename, 
                     	 float minX_filter, float maxX_filter, 
                     	 float minY_filter, float maxY_filter, 
                     	 float minZ_filter, float maxZ_filter,
                     	 int subsampling_factor){
							
		std::ofstream file(filename);

		if (!file.is_open())
		{
			std::cerr << "Error: Unable to open file " << filename << std::endl;
			return;
		}

		file << "x,y,z,sdist,hits" << std::endl;

		for (int iz = 0; iz < m_gridSizeZ; iz += subsampling_factor)
		{
			float z = m_minZ + iz * m_resolution;
			if (z < minZ_filter || z > maxZ_filter) continue; 

			for (int iy = 0; iy < m_gridSizeY; iy += subsampling_factor)
			{
				float y = m_minY + iy * m_resolution;
				if (y < minY_filter || y > maxY_filter) continue; 

				for (int ix = 0; ix < m_gridSizeX; ix += subsampling_factor)
				{
					float x = m_minX + ix * m_resolution;
					if (x < minX_filter || x > maxX_filter) continue; 

					uint64_t index = ix + iy * m_gridStepY + iz * m_gridStepZ;

					if (m_gridData[index].hits >= OCC_MIN_HITS)
					{
						float sdist = (m_gridData[index].s==0? -1.f : 1.f)*voxelDist(index);
							file << x << ',' << y << ',' << z << ','
							<< sdist << ',' << int(m_gridData[index].hits) << '\n';				
				
					}
				}
			}
		}

		file.close();
		std::cout << "CSV exported successfully: " << filename << std::endl;
	}

	// Export voxel centers to PCD (optional filters)
	void exportGridToPCD(const std::string& filename, 
						 int subsampling_factor,
						 bool only_occupied = true,
                     	 bool only_border   = true)
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
					if (only_occupied && !(m_gridData[index].hits >= OCC_MIN_HITS))	continue;
					float dist = voxelDist(index);
					if (only_border && dist != 0.0f)	continue;
					
					pcl::PointXYZI point;
					point.x = x;	point.y = y;	point.z = z;
					point.intensity = static_cast<float>(m_gridData[index].hits);
					cloud->push_back(point);
					
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


	// Export voxel centers to PLY (optional filters)
	void exportGridToPLY(const std::string& filename,
						int subsampling_factor,
						bool only_occupied = true,
						bool only_border   = true,
						uint16_t min_hits  = OCC_MIN_HITS)     
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->reserve((m_gridSizeX*m_gridSizeY*m_gridSizeZ)/(subsampling_factor*subsampling_factor*subsampling_factor));

		for (int iz = 0; iz < m_gridSizeZ; iz += subsampling_factor) {
			float z = m_minZ + iz * m_resolution;
			for (int iy = 0; iy < m_gridSizeY; iy += subsampling_factor) {
				float y = m_minY + iy * m_resolution;
				for (int ix = 0; ix < m_gridSizeX; ix += subsampling_factor) {
					float x = m_minX + ix * m_resolution;
					uint64_t index = ix + iy * m_gridStepY + iz * m_gridStepZ;

					if (only_occupied && m_gridData[index].hits < min_hits) continue; 
					float dist = voxelDist(index);
					if (only_border && dist != 0.0f) continue;                     

					cloud->push_back(pcl::PointXYZ{x,y,z});
				}
			}
		}
		if (cloud->empty()) { std::cerr << "Empty Cloud.\n"; return; }

		pcl::io::savePLYFileBinary(filename, *cloud);  
		std::cout << "PLY exported: " << filename << "  points=" << cloud->size() << std::endl;
	}


	// Queries and accessors
	inline bool isOccupied(const float &x, const float &y, const float &z)
	{
		return isIntoGrid(x,y,z) && (m_gridData[pointToGrid(x,y,z)].hits >= OCC_MIN_HITS);
	}

    inline uint32_t   getSizeX()     const { return m_gridSizeX; }
    inline uint32_t   getSizeY()     const { return m_gridSizeY; }
    inline uint32_t   getSizeZ()     const { return m_gridSizeZ; }
    inline float      getResolution()const { return m_resolution; }
    inline float      getMinX()      const { return m_minX; }
    inline float      getMinY()      const { return m_minY; }
    inline float      getMinZ()      const { return m_minZ; }
	inline const VoxelData* getVoxelPtr() const { return m_gridData; }

	inline float voxelDist(uint64_t idx) const 
	{ return float(__builtin_popcount(m_gridData[idx].d)); }

	const std::vector<pcl::PointXYZ>& getLastCorrectedCloud() const
	{ return m_last_corrected_cloud; }


protected:
	// Data
	std::vector<pcl::PointXYZ>  m_last_corrected_cloud;		

	VoxelData *m_gridData;								 
	
	// Directional kernels
	std::vector<DirectionalKernel> m_dirKernels;
	inline int dirToBin(const Eigen::Vector3f &v) const;
	void initDirectionalKernels();

	// Timing stats (printed every STAT_PERIOD frames)
	static constexpr int  STAT_PERIOD = 50;
	static inline std::atomic<uint64_t> s_frames{0};
	static inline std::atomic<double>   s_sumMs{0.0};
	static inline std::atomic<double>   s_sumSqMs{0.0};   		

	// Accumulate and log kernel timings
	static inline void recordKernelTime(double ms)
	{
		uint64_t n = ++s_frames;         
		s_sumMs   += ms;                 
		s_sumSqMs += ms * ms;          

		if (n % STAT_PERIOD == 0)
		{
			const double mean = s_sumMs.load() / static_cast<double>(n);
			const double var  = (s_sumSqMs.load() / static_cast<double>(n)) - mean * mean;
			double stddev = std::sqrt(std::max(var, 0.0));

			RCLCPP_INFO(rclcpp::get_logger("tdf_time"),
				"[32-bit]  %lu frames · mean = %.3f ms · σ = %.3f ms   (last = %.3f ms)",
				n, mean, stddev, ms);
		}
	}
};

// Map direction to bin index (azimuth × elevation)
inline int TDF3D32::dirToBin(const Eigen::Vector3f &v) const
{
	float az = std::atan2(v.y(), v.x());
	if (az < 0) az += 2.0f*M_PI;
	float el = std::asin(v.z() / v.norm());

	int baz = int(az * BINS_AZ / (2.0f*M_PI));
	int bel = int((el + M_PI/2) * BINS_EL / M_PI);
	bel = std::clamp(bel, 0, BINS_EL-1);
	baz = std::clamp(baz, 0, BINS_AZ-1);

	return bel*BINS_AZ + baz;          
}

// Build 40×40 directional kernels (21³ window per bin)
inline void TDF3D32::initDirectionalKernels()
{
	m_dirKernels.resize(NUM_BINS);

	auto binToDir = [](int az,int el){			 
		float azr = (az+0.5f)*2.0f*M_PI/BINS_AZ;
		float elr = (-M_PI/2)+ (el+0.5f)*M_PI/BINS_EL;
		float c = cosf(elr);
		return Eigen::Vector3f(c*cosf(azr), c*sinf(azr), sinf(elr));
	};

	for(int el=0; el<BINS_EL; ++el)
	for(int az=0; az<BINS_AZ; ++az)
	{
		DirectionalKernel& DK = m_dirKernels[el*BINS_AZ+az];
		Eigen::Vector3f dir = binToDir(az,el).normalized();

		int k=0;
		for(int z=-10;z<=10;++z)
		for(int y=-10;y<=10;++y)
		for(int x=-10;x<=10;++x,++k)
		{       
			constexpr float R_E = float(SHADOW_RADIUS_MD); 

			float re2 = float(x*x + y*y + z*z);            
			float re  = std::sqrt(re2);
			int   rd  = std::min(32, int(std::ceil(re)));   

			uint32_t mask = (rd == 0) ? 0u : (0xFFFFFFFFu >> (32 - rd));
			DK.distance_masks[k] = mask;

			bool behind   = dir.dot(Eigen::Vector3f(x,y,z)) > 0.0f;     
			const bool at_hit = (x == 0 && y == 0 && z == 0);
			bool inShadow = behind && (re2 <= R_E*R_E);                 
			DK.signs[k]   = (at_hit || inShadow) ? 0 : 1; 
		}
	}
}



#endif