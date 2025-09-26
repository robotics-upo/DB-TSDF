#ifndef __GRID_32_HPP__
#define __GRID_32_HPP__


#include <algorithm>  
#include <bitset>
#include <stdint.h>

class GRID32
{
    public:

	struct Iterator
    {
        Iterator(uint64_t **grid, uint32_t i, uint32_t base, uint32_t j, uint32_t cellSizeX) 
        { 
            _grid = grid; 
            _i = i;
            _j = j;
            _base = base;
            _cellSizeX = cellSizeX;
            _curr = _grid[_i];
        }

        Iterator& operator=(const Iterator &it)
        { 
            _grid = it._grid; 
            _i = it._i;
            _j = it._j;
            _base = it._base;
            _cellSizeX = it._cellSizeX;
            _curr = it._curr;
            return *this;
        }

        uint64_t &operator*() { 
            return _curr[_j+_base]; 
        }

        uint64_t *operator->() { return _curr + _j + _base; }

        Iterator& operator++() 
        { 
            _j++;
            if(_j >= _cellSizeX)
            {
                _j = 0;
                _i++;
                _curr = _grid[_i];
            }
            return *this; 
        }  

        protected:

        uint64_t **_grid;
        uint64_t *_curr;
        uint32_t _i, _j, _base, _cellSizeX;
    };


    GRID32(void)
	{
		_grid = NULL;
        _buffer = NULL; // Circular buffer to store all cell masks
		_garbage = UINT64_MAX;
		_dummy = NULL;       

    }

    void setup(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, float cellRes = 0.05, int maxCells = 100000)
	{
		if(_grid != NULL)
			free(_grid);

        if(_buffer != NULL)
			free(_buffer);

		if(_dummy != NULL)  
			free(_dummy);

		_maxX = (int)ceil(maxX);
		_maxY = (int)ceil(maxY);
		_maxZ = (int)ceil(maxZ);
		_minX = (int)floor(minX);
		_minY = (int)floor(minY);
		_minZ = (int)floor(minZ);
        
		// Memory allocation for the world grid. It is initialized to 0 (NULL)
		_gridSizeX = abs(_maxX-_minX);
		_gridSizeY = abs(_maxY-_minY);
		_gridSizeZ = abs(_maxZ-_minZ);
		_gridStepY = _gridSizeX;
		_gridStepZ = _gridSizeX*_gridSizeY;
		_gridSize = _gridSizeX*_gridSizeY*_gridSizeZ;
		//_grid = (uint64_t **)malloc(_gridSize*sizeof(uint64_t *));
        //std::memset(_grid, 0, _gridSize*sizeof(uint64_t *)); // Set pointers to NULL
        
        // Memory allocation for maxCells
        _maxCells = (uint32_t)maxCells;
        _cellRes = cellRes;
        _oneDivRes = 1.0/_cellRes;
        _cellSizeX = (uint32_t)_oneDivRes;
        _cellSizeY = (uint32_t)_oneDivRes;
        _cellSizeZ = (uint32_t)_oneDivRes;
        _cellStepY = _cellSizeX;
        _cellStepZ = _cellSizeX*_cellSizeY;
        _cellSize = 1 + _cellSizeX*_cellSizeY*_cellSizeZ;  // The 1 is to store control information of the cell
        _buffer = (uint64_t *)malloc(_maxCells*_cellSize*sizeof(uint64_t)); // Circular buffer to store all cell masks
        std::memset(_buffer, -1, _maxCells*_cellSize*sizeof(uint64_t));     // Init the buffer to longest distance
        for(int i=0; i<_maxCells; i++)
			_buffer[i*_cellSize] = (uint64_t)_gridSize;
        _cellIndex = 0;

		_dummy = (uint64_t*)malloc(_cellSize * sizeof(uint64_t));
		std::memset(_dummy, -1, _cellSize * sizeof(uint64_t));
		_dummy[0] = (uint64_t)_gridSize;

        _grid = (uint64_t**)malloc(_gridSize * sizeof(uint64_t*));
        for (uint32_t k = 0; k < _gridSize; ++k) _grid[k] = _dummy;

    }    

    ~GRID32(void)
	{
		if(_grid != NULL)
			free(_grid);

        if(_buffer != NULL)
			free(_buffer);
		if(_dummy != NULL)  
			free(_dummy); 
	
    }

	void clear(void)
	{
		for (uint32_t k = 0; k < _gridSize; ++k) _grid[k] = _dummy; // Set pointers to dummy
		std::memset(_buffer, -1, _maxCells*_cellSize*sizeof(uint64_t));     // Init the buffer to longest distance
        for(int i=0; i<_maxCells; i++)
			_buffer[i*_cellSize] = _gridSize;
        _cellIndex = 0;
	}

	void allocCell(float x, float y, float z)
	{
		x -= _minX;
		y -= _minY;
		z -= _minZ;
		uint32_t int_x = (uint32_t)x, int_y = (uint32_t)y, int_z = (uint32_t)z;
        uint32_t i = int_x + int_y*_gridStepY + int_z*_gridStepZ;
		if( _grid[i] == _dummy)  
		{
			_grid[i] = _buffer + (_cellIndex % _maxCells)*_cellSize;

			if(_grid[i][0] != (uint64_t)_gridSize)
			{
				_grid[(uint64_t)_grid[i][0]] = _dummy;
				std::memset(&(_grid[i][1]), -1, (_cellSize-1)*sizeof(uint64_t));     // Init the mask to longest distance
			} 
			_grid[i][0] = (uint64_t)i; 
			_cellIndex++;
		}
	}

	uint64_t &operator()(float x, float y, float z)
	{
		x -= _minX;
		y -= _minY;
		z -= _minZ;
		uint32_t int_x = (uint32_t)x, int_y = (uint32_t)y, int_z = (uint32_t)z;
        uint32_t i = int_x + int_y*_gridStepY + int_z*_gridStepZ;
		if(_grid[i] == _dummy) { return _garbage; }
		uint32_t j = 1 + (uint32_t)((x-int_x)*_oneDivRes) + (uint32_t)((y-int_y)*_oneDivRes)*_cellStepY + (uint32_t)((z-int_z)*_oneDivRes)*_cellStepZ;

        return _grid[i][j];
	}

	uint64_t read(float x, float y, float z)
	{
		x -= _minX;
		y -= _minY;
		z -= _minZ;
		uint32_t int_x = (uint32_t)x, int_y = (uint32_t)y, int_z = (uint32_t)z;
        uint32_t i = int_x + int_y*_gridStepY + int_z*_gridStepZ;
		if(_grid[i] == _dummy) { return _garbage; }

        uint32_t j = 1 + (uint32_t)((x-int_x)*_oneDivRes) + (uint32_t)((y-int_y)*_oneDivRes)*_cellStepY + (uint32_t)((z-int_z)*_oneDivRes)*_cellStepZ;

        return _grid[i][j];
	}

	Iterator getIterator(float x, float y, float z)
	{
		x -= _minX;
		y -= _minY;
		z -= _minZ;
		uint32_t int_x = (uint32_t)x, int_y = (uint32_t)y, int_z = (uint32_t)z;
        uint32_t i = int_x + int_y*_gridStepY + int_z*_gridStepZ;

		return Iterator(_grid, i, 1 + (uint32_t)((y-int_y)*_oneDivRes)*_cellStepY + (uint32_t)((z-int_z)*_oneDivRes)*_cellStepZ, (uint32_t)((x-int_x)*_oneDivRes), _cellSizeX);
	}

void exportGridToPCD(const std::string& filename, int subsampling_factor, bool only_occupied=true, bool only_border=true)
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
		std::cerr << "[GRID32] Warning: Empty Cloud (no mask==0 found).\n";
		return;
	}

	std::cout << "[GRID32] Total points (mask==0): " << cloud->size() << "\n";
    pcl::io::savePCDFileBinary(filename, *cloud);
    std::cout << "[GRID32] PCD exported: " << filename << "\n";
}

void exportGridToPLY(const std::string& filename, int subsampling_factor, bool only_occupied=true, bool only_border=true, uint16_t min_hits  = OCC_MIN_HITS)     
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
	if (cloud->empty()) { 
        std::cerr << "[GRID32] Warning: Empty Cloud (no mask==0 found).\n";
        return; 
    }

    std::cout << "[GRID32] Total points (mask==0): " << cloud->size() << "\n";
    pcl::io::savePCDFileBinary(filename, *cloud);
    std::cout << "[GRID32] PLY exported: " << filename << "\n";
}

	protected:



    uint64_t **_grid;
    float _maxX, _maxY, _maxZ, _minX, _minY, _minZ;
	uint32_t _gridSizeX, _gridSizeY, _gridSizeZ, _gridStepY, _gridStepZ, _gridSize;
    float _cellRes, _oneDivRes;
    uint32_t _cellSizeX, _cellSizeY, _cellSizeZ, _cellStepY, _cellStepZ, _cellSize;
    uint32_t _maxCells, _cellIndex;
    uint64_t *_buffer;
	uint64_t *_dummy;

	uint64_t _garbage;
};

#endif

