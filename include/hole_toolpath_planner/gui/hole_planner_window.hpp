#pragma once

#include <memory>
#include <vector>

#include <QMainWindow>
#include <vtkSmartPointer.h>

#include "hole_toolpath_planner/msg/hole_array.hpp"
#include "hole_toolpath_planner/msg/toolpath.hpp"
#include "hole_toolpath_planner/parameters.hpp"
#include "hole_toolpath_planner/srv/detect_holes.hpp"

class QAction;
class QCheckBox;
class QDoubleSpinBox;
class QDockWidget;
class QLabel;
class QPushButton;
class QVTKOpenGLNativeWidget;
class vtkActor;
class vtkAxes;
class vtkAxesActor;
class vtkPolyDataMapper;
class vtkPropAssembly;
class vtkRenderer;
class vtkTubeFilter;

namespace hole_toolpath_planner
{
class HoleDetector;

/**
 * @brief GUI window for loading a mesh, running the hole detector + toolpath generator, and visualizing results.
 */
class HolePlannerWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit HolePlannerWindow(QWidget* parent = nullptr);
  ~HolePlannerWindow() override;

  /**
   * @brief Sets the mesh to render and use during planning.
   */
  void setMeshFile(const QString& file);

  /**
   * @brief Loads planner parameters from a ROS-style YAML params file.
   */
  void loadParameters(const QString& file);

  /**
   * @brief Runs detection + toolpath planning using the current mesh and parameters.
   */
  void plan();

private:
  void rebuildDetector();
  void render();
  void refreshParametersUi();

  void onLoadMesh();
  void onLoadParameters();
  void onSaveHoles();
  void onSaveToolpaths();

  // VTK helpers
  void rebuildHoleActors();
  void rebuildToolpathActors();

  // Data + planner state
  std::string mesh_file_;
  QString params_file_;
  std::shared_ptr<rclcpp::Node> node_;
  PlannerParameters params_;
  std::unique_ptr<HoleDetector> detector_;
  srv::DetectHoles::Request detect_request_;
  msg::HoleArray holes_;
  std::vector<msg::Toolpath> toolpaths_;

  // UI widgets
  QAction* action_run_{};
  QAction* action_load_mesh_{};
  QAction* action_load_params_{};
  QAction* action_save_holes_{};
  QAction* action_save_toolpaths_{};

  QCheckBox* show_mesh_{};
  QCheckBox* show_holes_{};
  QCheckBox* show_toolpaths_{};
  QCheckBox* show_toolpath_lines_{};
  QCheckBox* show_axes_{};
  QCheckBox* generate_toolpaths_{};

  QDoubleSpinBox* min_diameter_{};
  QDoubleSpinBox* max_diameter_{};
  QDoubleSpinBox* min_length_{};
  QCheckBox* watertight_hint_{};

  QDoubleSpinBox* axis_size_{};
  QDoubleSpinBox* origin_size_{};

  QDoubleSpinBox* approach_offset_{};
  QDoubleSpinBox* stepdown_{};
  QDoubleSpinBox* spiral_pitch_{};

  // Rendering
  QVTKOpenGLNativeWidget* render_widget_{};
  vtkSmartPointer<vtkRenderer> renderer_;
  vtkSmartPointer<vtkPolyDataMapper> mesh_mapper_;
  vtkSmartPointer<vtkActor> mesh_actor_;

  vtkSmartPointer<vtkPropAssembly> hole_actor_;
  vtkSmartPointer<vtkPropAssembly> toolpath_actor_;
  vtkSmartPointer<vtkPropAssembly> toolpath_lines_actor_;

  vtkSmartPointer<vtkAxes> axes_;
  vtkSmartPointer<vtkAxesActor> axes_actor_;
  vtkSmartPointer<vtkTubeFilter> tube_filter_;
};

}  // namespace hole_toolpath_planner
