set(pcl_plugin_INCLUDE_DIRS_new "")
foreach(path ${pcl_plugin_INCLUDE_DIRS})
  list(APPEND pcl_plugin_INCLUDE_DIRS_new ${path})
  list(APPEND pcl_plugin_INCLUDE_DIRS_new "${path}/vtkPCLFilters")
endforeach()
set(pcl_plugin_INCLUDE_DIRS ${pcl_plugin_INCLUDE_DIRS_new})
