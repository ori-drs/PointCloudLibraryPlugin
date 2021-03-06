/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkAnnotateOBBs.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkAnnotateOBBs -
// .SECTION Description
//

#ifndef __vtkAnnotateOBBs_h
#define __vtkAnnotateOBBs_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkAnnotateOBBs : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkAnnotateOBBs, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkAnnotateOBBs *New();

  vtkGetMacro(AnnotateLabelZero, bool);
  vtkSetMacro(AnnotateLabelZero, bool);

  int GetNumberOfBoundingBoxes();
  void GetBoundingBoxOrigin(int boxId, double origin[3]);
  void GetBoundingBoxEdge(int boxId, int edgeId, double edge[3]);


protected:

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);

  vtkAnnotateOBBs();
  virtual ~vtkAnnotateOBBs();

  bool AnnotateLabelZero;

private:
  vtkAnnotateOBBs(const vtkAnnotateOBBs&);  // Not implemented.
  void operator=(const vtkAnnotateOBBs&);  // Not implemented.

  class vtkInternal;
  vtkInternal* Internal;
};

#endif


