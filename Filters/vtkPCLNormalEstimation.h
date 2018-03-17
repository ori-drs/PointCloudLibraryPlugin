/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLNormalEstimation.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLNormalEstimation -
// .SECTION Description
//

#ifndef __vtkPCLNormalEstimation_h
#define __vtkPCLNormalEstimation_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkPCLNormalEstimation : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCLNormalEstimation, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTKPCLFILTERS_OVERRIDE;

  static vtkPCLNormalEstimation *New();

  vtkSetMacro(SearchRadius, double);
  vtkGetMacro(SearchRadius, double);


protected:

  double SearchRadius;

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector) VTKPCLFILTERS_OVERRIDE;

  vtkPCLNormalEstimation();
  virtual ~vtkPCLNormalEstimation() VTKPCLFILTERS_OVERRIDE;


  virtual int FillInputPortInformation(int port,
                                       vtkInformation* info) VTKPCLFILTERS_OVERRIDE;

private:
  vtkPCLNormalEstimation(const vtkPCLNormalEstimation&)
      VTKPCLFILTERS_DELETE_FUNCTION;
  void operator=(const vtkPCLNormalEstimation&)
      VTKPCLFILTERS_DELETE_FUNCTION;
};

#endif
