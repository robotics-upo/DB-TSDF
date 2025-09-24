// VTKMeshExtractor: Extract a surface mesh from a TDF grid using VTK.
// Pipeline: TDF → vtkImageData → smoothing → Marching Cubes → mesh file.

#include "mesh/vtk_mesh_extractor.hpp"
#include <cstring> 

constexpr float KEEP_RATIO = 0.00f;   // Ratio of max voxel hits used as hit threshold

void VTKMeshExtractor::extract(const TDF3D32 &grid,
                               const std::string &filename,
                               float iso_level)
{
  // 1) Allocate VTK volume
  auto image = vtkSmartPointer<vtkImageData>::New();
  int nx = static_cast<int>(grid.getSizeX());
  int ny = static_cast<int>(grid.getSizeY());
  int nz = static_cast<int>(grid.getSizeZ());
  image->SetDimensions(nx, ny, nz);
  image->SetSpacing(grid.getResolution(), grid.getResolution(), grid.getResolution());
  image->SetOrigin (grid.getMinX(),      grid.getMinY(),      grid.getMinZ());
  image->AllocateScalars(VTK_FLOAT, 1);

  // 2) Copy TDF values into VTK buffer
  float *dest = static_cast<float*>(image->GetScalarPointer());
  size_t count = static_cast<size_t>(nx) * ny * nz;
  const float h = grid.getResolution();                      
  const VoxelData *vox  = grid.getVoxelPtr();    

  // 2.a) Compute hit threshold
  uint8_t maxHits = 0;
  for (size_t i = 0; i < count; ++i)
      if (vox[i].hits > maxHits) maxHits = vox[i].hits;

  const uint8_t thrHits =
      (maxHits == 0) ? 255
                    : static_cast<uint8_t>(std::round(maxHits * KEEP_RATIO));

  // 2.b) Fill buffer with signed band values
  const float BAND = 1.1f * h;  
  for (size_t i = 0; i < count; ++i)
  {
      const float dvox = grid.voxelDist(i);   
      const bool enough_hits = (vox[i].hits >= thrHits);
      if (!enough_hits) {
          dest[i] = +BAND;                 
          continue;
      }
      const bool occupied = ((vox[i].s & 0x01) == 0); 
      const bool is_center = (dvox == 0.0f);          
      if (occupied && is_center) {
          dest[i] = -BAND;                   
      } else {
          dest[i] = +BAND;               
      }
  }

  // 2.5) Gaussian smoothing
  auto smoother = vtkSmartPointer<vtkImageGaussianSmooth>::New();
  smoother->SetInputData(image);
  smoother->SetStandardDeviation(1.0);  
  // smoother->SetRadiusFactors(1.5, 1.5, 1.5);
  // smoother->SetDimensionality(3);
  smoother->Update();

  // 3) Marching Cubes extraction
  auto mc = vtkSmartPointer<vtkMarchingCubes>::New();
  mc->SetInputConnection(smoother->GetOutputPort());
  mc->ComputeNormalsOn();
  mc->SetValue(0, iso_level);
  mc->Update();

  // 4) Save mesh as STL or VTP  
  auto ext_pos = filename.find_last_of('.');
  std::string ext = (ext_pos==std::string::npos)
                    ? ""
                    : filename.substr(ext_pos+1);
  if (ext == "stl") {
    // Binary STL
    auto writer = vtkSmartPointer<vtkSTLWriter>::New();
    writer->SetFileName(filename.c_str());
    writer->SetInputData(mc->GetOutput());
    writer->SetFileTypeToBinary();  
    if (!writer->Write()) {
      throw std::runtime_error("VTK STL writer failed to write mesh.");
    }
  }
  else if (ext == "vtp") {
    // VTP XML
    auto writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer->SetFileName(filename.c_str());
    writer->SetInputData(mc->GetOutput());
    if (!writer->Write()) {
      throw std::runtime_error("VTK XML writer failed to write mesh.");
    }
  }
  else {
    throw std::invalid_argument(
      "Unsupported file extension: " + ext + 
      " (only .stl and .vtp are supported)");
  }
}








