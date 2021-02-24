/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkAnnotateOBBs.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkPCLConversions.h"

#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkTimerLog.h>
#include <vtkNew.h>
#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkTransformPolyDataFilter.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <cassert>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLConversions);

//----------------------------------------------------------------------------
vtkPCLConversions::vtkPCLConversions()
{
}

//----------------------------------------------------------------------------
vtkPCLConversions::~vtkPCLConversions()
{
}


//----------------------------------------------------------------------------

vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPCDFile(const std::string& filename)
{
  pcl::PCDReader reader;
  pcl::PCLPointCloud2Ptr cloud(new pcl::PCLPointCloud2());

  int version;
  int type;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;

#if PCL_VERSION_COMPARE(<,1,6,0)

  int idx;
  sensor_msgs::PointCloud2Ptr cloud(new pcl::PCLPointCloud2());
  reader.readHeader(filename, *cloud, origin, orientation, version, type, idx);

#elif PCL_VERSION_COMPARE(<,1,7,0)

  unsigned int idx;
  int offset = 0;
  sensor_msgs::PointCloud2Ptr cloud(new pcl::PCLPointCloud2());
  reader.readHeader(filename, *cloud, origin, orientation, version, type, idx, offset);

#else

  unsigned int idx;
  int offset = 0;
  reader.readHeader(filename, *cloud, origin, orientation, version, type, idx, offset);

#endif

  reader.read(filename, *cloud);
  return ConvertPointCloud2ToVtk(cloud, origin, orientation);
}

vtkSmartPointer<vtkPolyData> vtkPCLConversions::ConvertPointCloud2ToVtk(pcl::PCLPointCloud2Ptr& cloud_in)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New(); // OR poly_data->Reset();
  pcl::PCLPointCloud2Ptr cloud = cloud_in;

  // The vast majority of this function comes from pcl/io/vtk_lib_io.h, but because anymal repository version of
  // pcl removes vtk, we cannot use the function that exists there because the headers don't exist.
  // Add Points
  std::size_t x_idx = pcl::getFieldIndex (*cloud, std::string ("x") );
  vtkSmartPointer<vtkPoints> cloud_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cloud_vertices = vtkSmartPointer<vtkCellArray>::New ();

  vtkIdType pid[1];
  for (std::size_t point_idx = 0; point_idx < cloud->width * cloud->height; point_idx ++)
  {
    float point[3];

    int point_offset = (int (point_idx) * cloud->point_step);
    int offset = point_offset + cloud->fields[x_idx].offset;
    memcpy (&point, &cloud->data[offset], sizeof (float)*3);

    pid[0] = cloud_points->InsertNextPoint (point);
    cloud_vertices->InsertNextCell (1, pid);
  }

  //set the points and vertices we created as the geometry and topology of the polydata
  poly_data->SetPoints (cloud_points);
  poly_data->SetVerts (cloud_vertices);

  // Add RGB
  int rgb_idx = pcl::getFieldIndex (*cloud, "rgb");
  if (rgb_idx != -1)
  {
    //std::cout << "Adding rgb" << std::endl;
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();

    colors->SetNumberOfComponents (3);
    colors->SetName ("rgb");

    for (std::size_t point_idx = 0; point_idx < cloud->width * cloud->height; point_idx ++)
    {
      unsigned char bgr[3];

      int point_offset = (int (point_idx) * cloud->point_step);
      int offset = point_offset + cloud->fields[rgb_idx].offset;
      memcpy (&bgr, &cloud->data[offset], sizeof (unsigned char)*3);

      colors->InsertNextTuple3(bgr[2], bgr[1], bgr[0]);
    }

    poly_data->GetCellData()->SetScalars(colors);
  }

  // Add other fields
  for (size_t i = 0; i < cloud->fields.size(); ++i)
  {
    std::string field_name = cloud->fields[i].name;
    int field_idx = pcl::getFieldIndex(*cloud, field_name);
    
    if (field_name == "x" || field_name == "y" || field_name == "z" || field_name == "rgb" )
      continue;
    
    if (field_idx != -1)
    {
      vtkSmartPointer<vtkFloatArray> cloud_data = vtkSmartPointer<vtkFloatArray>::New ();
      cloud_data->SetNumberOfComponents (1);
      cloud_data->SetName(field_name.c_str());

      for (std::size_t point_idx = 0; point_idx < cloud->width * cloud->height; point_idx ++)
      {
        float value;

        int point_offset = (int (point_idx) * cloud->point_step);
        int offset = point_offset + cloud->fields[field_idx].offset;
        memcpy (&value, &cloud->data[offset], sizeof(float));
        cloud_data->InsertNextValue(value);
      }

      poly_data->GetPointData()->AddArray(cloud_data);

     }
  }

  return poly_data;
}


vtkSmartPointer<vtkPolyData> vtkPCLConversions::ConvertPointCloud2ToVtk(pcl::PCLPointCloud2Ptr& cloud,
                                                                        const Eigen::Vector4f& origin,
                                                                        const Eigen::Quaternionf& orientation)
{
  vtkSmartPointer<vtkPolyData> polyData = ConvertPointCloud2ToVtk(cloud);

  // transform polydata
  vtkSmartPointer<vtkTransform> transform = convertTransform(origin, orientation);

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetTransform(transform);
  transformFilter->SetInputData(polyData);
  transformFilter->Update();
  polyData->DeepCopy(transformFilter->GetOutput());

  return polyData;
}

vtkSmartPointer<vtkTransform> vtkPCLConversions::convertTransform(const Eigen::Vector4f &origin, const Eigen::Quaternionf& orientation)
{

  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();

  double translation[3] = {origin(0), origin(1), origin(2)};
  //rotation
  Eigen::AngleAxisf angleAxis(orientation);
  double theta = angleAxis.angle();
  double axis[3] = {angleAxis.axis()(0), angleAxis.axis()(1), angleAxis.axis()(2)};

  t->Identity();
  t->Translate(translation);
  t->RotateWXYZ(theta * 180./M_PI, axis);
  return t;

}

//----------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr vtkPCLConversions::PointCloudFromPolyData(vtkPolyData* polyData)
{
  const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = numberOfPoints;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(numberOfPoints);

  if (!numberOfPoints)
    {
    return cloud;
    }

  vtkFloatArray* floatPoints = vtkFloatArray::SafeDownCast(polyData->GetPoints()->GetData());
  vtkDoubleArray* doublePoints = vtkDoubleArray::SafeDownCast(polyData->GetPoints()->GetData());
  assert(floatPoints || doublePoints);

  if (floatPoints)
    {
    float* data = floatPoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
      cloud->points[i].x = data[i*3];
      cloud->points[i].y = data[i*3+1];
      cloud->points[i].z = data[i*3+2];
      }
    }
  else if (doublePoints)
    {
    double* data = doublePoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
      cloud->points[i].x = data[i*3];
      cloud->points[i].y = data[i*3+1];
      cloud->points[i].z = data[i*3+2];
      }
    }

  return cloud;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> vtkPCLConversions::NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
    {
    ids[i*2] = 1;
    ids[i*2+1] = i;
    }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

//----------------------------------------------------------------------------
void vtkPCLConversions::AddVertexCells(vtkPolyData* polyData)
{
  if (!polyData || !polyData->GetNumberOfPoints())
  {
    return;
  }

  polyData->SetVerts(NewVertexCells(polyData->GetNumberOfPoints()));
}

//----------------------------------------------------------------------------
void vtkPCLConversions::PerformPointCloudConversionBenchmark(vtkPolyData* polyData)
{
  if (!polyData)
    {
    return;
    }

  double start;
  double elapsed;
  unsigned long kilobytes;

  const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();
  std::cout << "Number of input points: " << numberOfPoints << std::endl;

  start = vtkTimerLog::GetUniversalTime();
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud = PointCloudFromPolyData(polyData);
  elapsed = vtkTimerLog::GetUniversalTime() - start;

  std::cout << "Conversion to pcl::PointCloud took " << elapsed << " seconds. "
            << numberOfPoints / elapsed << " points per second." << std::endl;


  start = vtkTimerLog::GetUniversalTime();
  pcl::PCLPointCloud2Ptr tempCloud2(new pcl::PCLPointCloud2());
  toPCLPointCloud2(*tempCloud, *tempCloud2);
  vtkSmartPointer<vtkPolyData> tempPolyData = ConvertPointCloud2ToVtk(tempCloud2);
  elapsed = vtkTimerLog::GetUniversalTime() - start;

  std::cout << "Conversion to vtkPolyData took " << elapsed << " seconds. "
            << numberOfPoints / elapsed << " points per second." << std::endl;


  start = vtkTimerLog::GetUniversalTime();
  vtkSmartPointer<vtkCellArray> tempCells = NewVertexCells(numberOfPoints);
  elapsed = vtkTimerLog::GetUniversalTime() - start;

  std::cout << "Constructing vertex cells took " << elapsed << " seconds. "
            << numberOfPoints / elapsed << " points per second." << std::endl;


  kilobytes = tempPolyData->GetActualMemorySize();
  std::cout << "vtkPolyData uses " << kilobytes/1024.0 << " MB. "
            << kilobytes*1024 / numberOfPoints << " bytes per point." << std::endl;

  kilobytes = tempPolyData->GetPoints()->GetActualMemorySize();
  std::cout << "vtkPolyData's points use " << kilobytes/1024.0 << " MB. "
            << kilobytes*1024 / numberOfPoints << " bytes per point." << std::endl;

  kilobytes = tempPolyData->GetVerts()->GetActualMemorySize();
  std::cout << "vtkPolyData's cells use " << kilobytes/1024.0 << " MB. "
            << kilobytes*1024 / numberOfPoints << " bytes per point." << std::endl;
}

//----------------------------------------------------------------------------
namespace {

vtkSmartPointer<vtkIntArray> NewLabelsArray(vtkIdType length)
{
  vtkSmartPointer<vtkIntArray> labels = vtkSmartPointer<vtkIntArray>::New();
  labels->SetNumberOfComponents(1);
  labels->SetNumberOfTuples(length);
  labels->FillComponent(0, 0);
  return labels;
}

void LabelIndices(const std::vector<int>& indices, vtkIntArray* labels, const int labelValue)
{
  const size_t numberOfIndices = indices.size();
  for (size_t k = 0; k < numberOfIndices; ++k)
    {
    labels->SetValue(indices[k], labelValue);
    }
}

}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(pcl::IndicesConstPtr indices, vtkIdType length)
{
  vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);
  if (indices)
    {
    LabelIndices(*indices, labels, 1);
    }  
  return labels;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(pcl::PointIndices::ConstPtr indices, vtkIdType length)
{
  vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);
  if (indices)
    {
    LabelIndices(indices->indices, labels, 1);
    }  
  return labels;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(const std::vector<pcl::PointIndices>& indices, vtkIdType length)
{
  vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);

  for (size_t i = 0; i < indices.size(); ++i)
    {
    const int labelValue = i + 1;
    LabelIndices(indices[i].indices, labels, labelValue);
    }

  return labels;
}

//----------------------------------------------------------------------------
void vtkPCLConversions::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  //os << indent << "Property: " << endl;
}
