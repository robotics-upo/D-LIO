#ifndef __GRID_64_HPP__
#define __GRID_64_HPP__


#include <algorithm>  
#include <bitset>
#include <stdint.h>

class GRID64
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


    GRID64(void)
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

    ~GRID64(void)
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

void exportGridToPCD(const std::string& filename, int subsampling_factor)
{
    using PointT = pcl::PointXYZI;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    const uint32_t step = std::max(1, subsampling_factor);

    for (uint32_t cz = 0; cz < _gridSizeZ; ++cz)
    {
        const float z0 = _minZ + static_cast<float>(cz);
        for (uint32_t cy = 0; cy < _gridSizeY; ++cy)
        {
            const float y0 = _minY + static_cast<float>(cy);
            for (uint32_t cx = 0; cx < _gridSizeX; ++cx)
            {
                const float x0 = _minX + static_cast<float>(cx);

                const uint32_t i = cx + cy * _gridStepY + cz * _gridStepZ;
                uint64_t* cell = _grid[i];
                if (cell == _dummy) continue;

                for (uint32_t vz = 0; vz < _cellSizeZ; vz += step) {
                    for (uint32_t vy = 0; vy < _cellSizeY; vy += step) {
                        for (uint32_t vx = 0; vx < _cellSizeX; vx += step) {
                            const uint32_t j = 1u + vx + vy * _cellStepY + vz * _cellStepZ;

                            const uint64_t mask = cell[j];
                            if (mask != 0ULL) continue; 

                            PointT pt;
                            pt.x = x0 + vx * _cellRes;
                            pt.y = y0 + vy * _cellRes;
                            pt.z = z0 + vz * _cellRes;
                            pt.intensity = 0.0f;
                            cloud->push_back(pt);
                        }
                    }
                }
            }
        }
    }

    if (cloud->empty())
    {
        std::cerr << "[GRID64] Warning: Empty Cloud (no mask==0 found).\n";
        return;
    }

    std::cout << "[GRID64] Total points (mask==0): " << cloud->size() << "\n";
    pcl::io::savePCDFileBinary(filename, *cloud);
    std::cout << "[GRID64] PCD exported: " << filename << "\n";
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

