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

/**
 * @brief The PointCloudFieldIterator class is used to iterate through a PCLPointCloud2
 */
class PointCloudFieldIterator
{
public:
  PointCloudFieldIterator(pcl::PCLPointCloud2& cloud, const std::string& field_name)
  {
    for (size_t i = 0; i < cloud.fields.size(); ++i)
    {
      if(cloud.fields[i].name == field_name)
      {
        int offset = cloud.fields[i].offset;
        data_char_ = &(cloud.data.front()) + offset;
        data_ = reinterpret_cast<float*>(data_char_);
        point_step_ = cloud.point_step;
        return;
      }
    }
    throw std::runtime_error("Field " + field_name + " does not exist");

  }

  PointCloudFieldIterator& operator ++()
  {
    data_char_ += point_step_;
    data_ = reinterpret_cast<float*>(data_char_);
  }

  float& operator*()
  {
    return *data_;
  }

private:
  unsigned char* data_char_;
  float* data_;
  int point_step_;
};

vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPCDFile(const std::string& filename)
{
  pcl::PCDReader reader;
  pcl::PCLPointCloud2 cloud;

  int version;
  int type;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;

#if PCL_VERSION_COMPARE(<,1,6,0)

  int idx;
  sensor_msgs::PointCloud2 cloud;
  reader.readHeader(filename, cloud, origin, orientation, version, type, idx);

#elif PCL_VERSION_COMPARE(<,1,7,0)

  unsigned int idx;
  int offset = 0;
  sensor_msgs::PointCloud2 cloud;
  reader.readHeader(filename, cloud, origin, orientation, version, type, idx, offset);

#else

  unsigned int idx;
  int offset = 0;
  reader.readHeader(filename, cloud, origin, orientation, version, type, idx, offset);

#endif

  reader.read(filename, cloud);
  return ConvertPointCloud2ToVtk(cloud, origin, orientation);
}

vtkSmartPointer<vtkPolyData> vtkPCLConversions::ConvertPointCloud2ToVtk(pcl::PCLPointCloud2& cloud)
{
  const size_t numberOfPoints = cloud.height*cloud.width;

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToFloat();
  points->Allocate(numberOfPoints);
  polyData->SetPoints(points);
  polyData->SetVerts(NewVertexCells(numberOfPoints));

  for (size_t i = 0; i < cloud.fields.size(); ++i)
  {
    std::string field_name = cloud.fields[i].name;
    //points (x,y,z) are read from the x field only so y and z can be ignored
    if (field_name == "y" || field_name == "z")
      continue;

    if (field_name == "x")
    {
      PointCloudFieldIterator iter(cloud, field_name);
      for (size_t j = 0; j < numberOfPoints; ++j, ++iter)
      {
        points->InsertNextPoint(&(*iter));
      }
    } else
    {
      vtkSmartPointer<vtkFloatArray> field = vtkSmartPointer<vtkFloatArray>::New();
      field->SetName(field_name.c_str());
      field->SetNumberOfValues(numberOfPoints);
      polyData->GetPointData()->AddArray(field);
      PointCloudFieldIterator iter(cloud, field_name);
      for (size_t j = 0; j < numberOfPoints; ++j, ++iter)
      {
        field->SetValue(j, *iter);
      }
    }

  }

  return polyData;
}


vtkSmartPointer<vtkPolyData> vtkPCLConversions::ConvertPointCloud2ToVtk(pcl::PCLPointCloud2& cloud,
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
  pcl::PCLPointCloud2 tempCloud2;
  toPCLPointCloud2(*tempCloud, tempCloud2);
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
