#ifndef __TSDF3D_16_HPP__
#define __TSDF3D_16_HPP__

#include <algorithm>  
#include <bitset>
#include <db_tsdf/trilinear_params.hpp>
#include <db_tsdf/grid_16.hpp>

#include <Eigen/Dense>	
#include <iostream>
#include <string>
#include <map>  
#include <set>

struct DirectionalKernel
{
    std::vector<uint16_t> distance_masks;		// Manhattan distance masks
    std::vector<uint8_t>  signs;		        // 0 = occ, 1 = free
};

class TSDF3D16
{
	
public:

	TSDF3D16(void) 
	{
		m_maxX = 40;
		m_maxY = 40;
		m_maxZ = 40;
		m_minX = -40;
		m_minY = -40;
		m_minZ = -10;
		m_resolution = 0.05;
		m_oneDivRes = 1/m_resolution;
	}

	~TSDF3D16(void)
	{
	}

	void setup(float minX, float maxX, 
			   float minY, float maxY, 
			   float minZ, float maxZ, 
			   float resolution, 
			   int kernelSize,
			   int occMinHits,
			   int binsAz,
               int binsEl,
               int shadowRadius,
               std::string distanceMode,
			   int maxCells = 50000,
			   bool verboseInit = false
			   )
	{
		m_maxX = maxX;
		m_maxY = maxY;
		m_maxZ = maxZ;
		m_minX = minX;
		m_minY = minY;
		m_minZ = minZ;
		m_resolution = resolution;
		m_oneDivRes = 1/m_resolution;
		m_occMinHits = occMinHits;
		m_binsAz = binsAz;
        m_binsEl = binsEl;
        m_numBins = m_binsAz * m_binsEl;
        m_shadowRadiusMd = shadowRadius;
        m_distanceMode = distanceMode;
		m_kernelSize = kernelSize;
    	m_kernelRadius = (kernelSize - 1) / 2;
        m_verboseInit = verboseInit;

		initDirectionalKernels();

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

	void exportGridToPLY(const std::string& filename, int subsampling_factor)
	{

		m_grid.exportGridToPLY(filename, subsampling_factor);
	}

	void exportSubgridToCSV(const std::string& filename, int subsampling_factor)
	{

		m_grid.exportSubgridToCSV(filename, subsampling_factor);
	}

	void exportMesh(const std::string& filename, float iso_level)
    {
        m_grid.exportMesh(filename, iso_level, m_occMinHits);
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
		loadCloud(out, Eigen::Vector3f(tx, ty, tz));
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
		loadCloud(out, Eigen::Vector3f(tx, ty, tz));
	}

	void loadCloudFiltered(std::vector<pcl::PointXYZ> &cloud,
                       float tx, float ty, float tz,
                       float roll, float pitch, float yaw,
                       float maxAbs)
	{
		std::vector<pcl::PointXYZ> out;
		out.reserve(cloud.size()); 

		// Matriz de rotación precalculada (radianes)
		const float sr = std::sin(roll),  cr = std::cos(roll);
		const float sp = std::sin(pitch), cp = std::cos(pitch);
		const float sy = std::sin(yaw),   cy = std::cos(yaw);

		const float r00 =  cy*cp;           const float r01 =  cy*sp*sr - sy*cr;  const float r02 =  cy*sp*cr + sy*sr;
		const float r10 =  sy*cp;           const float r11 =  sy*sp*sr + cy*cr;  const float r12 =  sy*sp*cr - cy*sr;
		const float r20 = -sp;              const float r21 =  cp*sr;              const float r22 =  cp*cr;

		for (std::size_t i = 0; i < cloud.size(); ++i)
		{
			const float ix = cloud[i].x;
			const float iy = cloud[i].y;
			const float iz = cloud[i].z;

			// Sólo calcular x', y' para el filtro rápido
			const float xw = ix*r00 + iy*r01 + iz*r02 + tx;
			const float yw = ix*r10 + iy*r11 + iz*r12 + ty;

			// Filtro por ventana rectangular en XY (Chebyshev / AABB)
			if (std::fabs(xw) <= maxAbs && std::fabs(yw) <= maxAbs)
			{
				// Calcula z' sólo si el punto pasa el filtro
				const float zw = ix*r20 + iy*r21 + iz*r22 + tz;

				pcl::PointXYZ p;
				p.x = xw; p.y = yw; p.z = zw;
				out.push_back(p);
			}
		}
		loadCloud(out, Eigen::Vector3f(tx, ty, tz));
	}


	void loadCloud(std::vector<pcl::PointXYZ> &cloud, const Eigen::Vector3f &sensor_pos_world)
	{	
		// Allocate required submetric cells
		for(uint32_t i=0; i<cloud.size(); i++)
		{
			for(float z=cloud[i].z-1; z<=cloud[i].z+1; z+=1)
				for(float y=cloud[i].y-1; y<=cloud[i].y+1; y+=1)
					for(float x=cloud[i].x-1; x<=cloud[i].x+1; x+=1)
						if(isIntoGrid(x, y, z))
							m_grid.allocCell(x, y, z);
		}

		// Applies the pre-computed kernel to all grid cells centered in the cloud points 
		const float step = m_kernelRadius * m_resolution;

		#pragma omp parallel for num_threads(16) shared(m_dirKernels, m_grid) 
		for(uint32_t i=0; i<cloud.size(); i++)
		{
			
			if(!isIntoGrid(cloud[i].x - step*2, cloud[i].y - step*2, cloud[i].z - step*2) || 
			   !isIntoGrid(cloud[i].x + step*2, cloud[i].y + step*2, cloud[i].z + step*2))
				continue;

			// Select kernel by ray direction
			Eigen::Vector3f point_world(cloud[i].x, cloud[i].y, cloud[i].z);
        	Eigen::Vector3f dir = point_world - sensor_pos_world;
			const DirectionalKernel& DK = m_dirKernels[dirToBin(dir)];

			int xi, yi, zi, k = 0;
			float x, y, z;
			for(zi=0, z=cloud[i].z-step; zi<m_kernelSize; zi++, z+=m_resolution)
				for(yi=0, y=cloud[i].y-step; yi<m_kernelSize; yi++, y+=m_resolution){
					GRID16::Iterator it = m_grid.getIterator(cloud[i].x-step,y,z);
					for(xi=0, x=cloud[i].x-step; xi<m_kernelSize; xi++, x+=m_resolution,++it, ++k)
					{						
						VoxelData &v = *it;
						uint16_t old_mask = v.d;
						uint16_t new_mask = old_mask & DK.distance_masks[k];
						if (new_mask != old_mask) v.d = new_mask;

						if(DK.signs[k] == 0 && v.hits < m_occMinHits) 
						{
							++v.hits;
							if (v.hits == m_occMinHits)
								v.s &= uint8_t(~0x01);
						}
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
			c000 = std::bitset<16>(m_grid.read(x, y, z).d).count(); 
			c001 = std::bitset<16>(m_grid.read(x, y, z+m_resolution).d).count(); 
			c010 = std::bitset<16>(m_grid.read(x, y+m_resolution, z).d).count(); 
			c011 = std::bitset<16>(m_grid.read(x, y+m_resolution, z+m_resolution).d).count();  
			c100 = std::bitset<16>(m_grid.read(x+m_resolution, y, z).d).count();  
			c101 = std::bitset<16>(m_grid.read(x+m_resolution, y, z+m_resolution).d).count();  
			c110 = std::bitset<16>(m_grid.read(x+m_resolution, y+m_resolution, z).d).count();  
			c111 = std::bitset<16>(m_grid.read(x+m_resolution, y+m_resolution, z+m_resolution).d).count(); 

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
	GRID16 m_grid;
	float m_maxX, m_maxY, m_maxZ;
	float m_minX, m_minY, m_minZ;
	float m_resolution, m_oneDivRes;	
	int m_occMinHits;
	int m_kernelSize;
    int m_kernelRadius;
	int m_binsAz;
    int m_binsEl;
    int m_numBins;
    int m_shadowRadiusMd;
    std::string m_distanceMode;
    bool m_verboseInit{false};

	// Directional kernels
	std::vector<DirectionalKernel> m_dirKernels;
	inline int dirToBin(const Eigen::Vector3f &v) const;
	void initDirectionalKernels();

	std::map<int, uint16_t> m_r_squared_to_mask16;

};

// Map direction to bin index (azimuth × elevation)
inline int TSDF3D16::dirToBin(const Eigen::Vector3f &v) const
{
	float az = std::atan2(v.y(), v.x());
	if (az < 0) 
	{
		az += 2.0f*M_PI;
	}

	float el  = std::asin(v.z() / v.norm());
	int   baz = int(az * m_binsAz / (2.0f*M_PI));
    int   bel = int((el + M_PI/2) * m_binsEl / M_PI);

    bel = std::clamp(bel, 0, m_binsEl - 1);
    baz = std::clamp(baz, 0, m_binsAz - 1);

    return bel*m_binsAz + baz;    
}


inline void TSDF3D16::initDirectionalKernels()
{
    if (m_distanceMode == "L2")
    {
        if (m_verboseInit) std::cout << "--- [TSDF3D16] Generating 16-bit L2 (Euclidean) distance LUT..." << std::endl;
        if (m_r_squared_to_mask16.empty())
        {
            std::set<int> unique_r_squared;
            for (int z = -m_kernelRadius; z <= m_kernelRadius; ++z)
                for (int y = -m_kernelRadius; y <= m_kernelRadius; ++y)
                    for (int x = -m_kernelRadius; x <= m_kernelRadius; ++x)
                    {
                        unique_r_squared.insert(x*x + y*y + z*z);
                    }

            m_r_squared_to_mask16.clear();
            uint16_t rank = 0;

            if (m_verboseInit) std::cout << "---Rank -> L2 Distance (Voxels)" << std::endl;
            for (int r2 : unique_r_squared)
            {
                uint16_t mask;
                if (rank == 0)      { mask = 0u; }
                else if (rank < 16) { mask = (0xFFFFu >> (16 - rank)); }
                else                { mask = 0xFFFFu; }
                m_r_squared_to_mask16[r2] = mask;
                if (m_verboseInit)
                {
                    float l2_dist = std::sqrt(static_cast<float>(r2));
                    std::cout << std::setw(5) << rank << " -> " << l2_dist;
                    if (rank >= 16) { std::cout << " (Truncated to 16 bits)"; }
                    std::cout << std::endl;
                }
                rank++;
            }
        }
    }
    else if (m_distanceMode == "L1")
    {
        if (m_verboseInit) std::cout << "--- [TSDF3D16] Using 16-bit L1 (Manhattan) distance masks." << std::endl;
    }
    else
    {
    	std::cerr << "[TSDF3D16] Warning: distance_mode '" << m_distanceMode << "' not recognized. Defaulting to 'L1'." << std::endl;
    	m_distanceMode = "L1";
    }

	// --- Kernel Generation ---
    m_dirKernels.resize(m_numBins);

	const int kernel_total_voxels = m_kernelSize * m_kernelSize * m_kernelSize;

    auto binToDir = [&](int az,int el) // <-- Añadido [&] para capturar 'this'
    {            
        float azr = (az+0.5f)*2.0f*M_PI / m_binsAz;
        float elr = (-M_PI/2)+ (el+0.5f)*M_PI / m_binsEl;
        float c   = cosf(elr);
        return Eigen::Vector3f(c*cosf(azr), c*sinf(azr), sinf(elr));
    };
	
    for(int el=0; el < m_binsEl; ++el) 
    for(int az=0; az < m_binsAz; ++az) 
    {
        DirectionalKernel& DK = m_dirKernels[el*m_binsAz+az]; 
        Eigen::Vector3f dir = binToDir(az,el).normalized();

		DK.distance_masks.resize(kernel_total_voxels);
        DK.signs.resize(kernel_total_voxels);
		
        int k=0;
        for(int z=-m_kernelRadius;z<=m_kernelRadius;++z) 
        for(int y=-m_kernelRadius;y<=m_kernelRadius;++y)
        for(int x=-m_kernelRadius;x<=m_kernelRadius;++x,++k)
        {       
            const float R_E = float(m_shadowRadiusMd); 
            float re2 = float(x*x + y*y + z*z); 

			// The Distance Mode "switch"
            if (m_distanceMode == "L1")
            {
                int l1_dist = std::abs(x) + std::abs(y) + std::abs(z);
                int   rd  = std::min(16, l1_dist);         
                uint16_t mask = (rd == 0) ? 0u : (0xFFFFu >> (16 - rd));
                DK.distance_masks[k] = mask;
            }
            else // "L2"
            {
                int r2_int = x*x + y*y + z*z; 
                DK.distance_masks[k] = m_r_squared_to_mask16[r2_int];
            }
            
            bool behind   = dir.dot(Eigen::Vector3f(x,y,z)) >= 0.0f;     
            const bool at_hit = (x == 0 && y == 0 && z == 0);
            bool inShadow = behind && (re2 <= R_E*R_E);                 
            DK.signs[k]   = (at_hit || inShadow) ? 0 : 1; 
        }

		// --- Kernel preview (only with verbose_init) ---
        int az_ray_from_left = 0; // Azimuth bin 0 corresponde a +X
        int el_horizontal = m_binsEl / 2; // Bin de elevación central (horizontal)

        if (m_verboseInit && el == el_horizontal && az == az_ray_from_left)
        {
            std::cout << "\n--- Kernel preview (+X Ray) [Z=0 Slice] ---" << std::endl;
            std::cout << "--- Rank " << m_distanceMode << " | Shadow Radius: " << m_shadowRadiusMd << " ---" << std::endl;
            std::cout << "Format: [Rank][Sign] (#=Occupied/Shadow, .=Free)\n" << std::endl;
            std::cout << " (Y+) \n" << "  ^ \n" << "  | \n";
            
            int z_central = 0;
            int iz = z_central + m_kernelRadius;
            
            for(int y = m_kernelRadius; y >= -m_kernelRadius; --y) { 
                int iy = y + m_kernelRadius; 
                std::cout << std::setw(2) << y << " | "; 
                for(int x = -m_kernelRadius; x <= m_kernelRadius; ++x) { 
                    int ix = x + m_kernelRadius;
                    int k_idx = (iz * m_kernelSize * m_kernelSize) + (iy * m_kernelSize) + ix;
                    uint16_t mask = DK.distance_masks[k_idx];
                    int rank = std::bitset<16>(mask).count();
                    uint8_t sign = DK.signs[k_idx];
                    char sign_char = (sign == 0) ? '#' : '.';
                    std::cout << " " << std::setw(2) << rank << sign_char;
                }
                std::cout << std::endl; 
            }
            
            std::cout << "   " << std::string(60, '-') << "> (X+)" << std::endl;
            std::cout << "     ";
            for(int x = -m_kernelRadius; x <= m_kernelRadius; ++x) {
                std::cout << " " << std::setw(3) << x;
            }
            std::cout << "\n---------------------------------------------------------------------\n" << std::endl;
        }
    }
}

#endif
