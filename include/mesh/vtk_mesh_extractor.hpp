// vtk_mesh_extractor.hpp
#ifndef VTK_MESH_EXTRACTOR_HPP
#define VTK_MESH_EXTRACTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>        
#include <pcl/io/pcd_io.h>
#include <string>
#include <db_tsdf/tsdf3d_32.hpp>

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkMarchingCubes.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkSTLWriter.h>
#include <vtkMultiThreader.h>
#include <vtkImageGaussianSmooth.h>
#include <vtkSmoothPolyDataFilter.h>




/// \brief Static helper to extract .stl mesh from TDF3D64
class VTKMeshExtractor {
public:
  /// \param grid      
  /// \param filename 
  /// \param iso_level
  static void extract(const TDF3D32 &grid,
                      const std::string &filename,
                      float iso_level);
};

#endif // VTK_MESH_EXTRACTOR_HPP
