#include "hole_toolpath_planner/gui/hole_planner_window.hpp"

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QCommandLineParser>
#include <QDockWidget>
#include <QFileDialog>
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QToolBar>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkAbstractPolyDataReader.h>
#include <vtkAxes.h>
#include <vtkAxesActor.h>
#include <vtkLeaderActor2D.h>
#include <vtkLineSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkOpenGLActor.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLRenderer.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropAssembly.h>
#include <vtkRenderWindow.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkSTLReader.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkTubeFilter.h>

#include <pcl/io/vtk_lib_io.h>

#include <Eigen/Geometry>
#include <array>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <memory>
#include <exception>

#include "hole_toolpath_planner/hole_detector.hpp"

namespace hole_toolpath_planner
{
namespace
{
Eigen::Isometry3d toEigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (q.norm() > 1e-9)
    iso.linear() = q.normalized().toRotationMatrix();
  return iso;
}

vtkSmartPointer<vtkTransform> toVTK(const Eigen::Isometry3d& mat)
{
  auto t = vtkSmartPointer<vtkTransform>::New();
  t->Translate(mat.translation().data());
  Eigen::AngleAxisd aa(mat.rotation());
  t->RotateWXYZ(aa.angle() * 180.0 / M_PI, aa.axis().data());
  return t;
}

vtkSmartPointer<vtkActor> makeSphere(const Eigen::Vector3d& center, double radius, const std::array<double, 3>& color)
{
  vtkNew<vtkSphereSource> source;
  source->SetCenter(center.x(), center.y(), center.z());
  source->SetRadius(radius);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(source->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(color[0], color[1], color[2]);
  actor->GetProperty()->SetOpacity(0.8);

  return actor;
}

vtkSmartPointer<vtkPropAssembly> createPoseActors(const std::vector<geometry_msgs::msg::Pose>& poses,
                                                  vtkAlgorithmOutput* waypoint_shape_output_port)
{
  auto assembly = vtkSmartPointer<vtkPropAssembly>::New();

  for (const auto& pose : poses)
  {
    auto transform_filter = vtkSmartPointer<vtkTransformFilter>::New();
    transform_filter->SetTransform(toVTK(toEigen(pose)));
    transform_filter->SetInputConnection(waypoint_shape_output_port);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transform_filter->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    assembly->AddPart(actor);
  }

  return assembly;
}

vtkSmartPointer<vtkPropAssembly> createPolylineActors(const std::vector<geometry_msgs::msg::Pose>& poses,
                                                      double radius,
                                                      const std::array<double, 3>& color)
{
  auto assembly = vtkSmartPointer<vtkPropAssembly>::New();
  if (poses.size() < 2)
    return assembly;

  for (std::size_t i = 0; i + 1 < poses.size(); ++i)
  {
    vtkNew<vtkLineSource> line;
    line->SetPoint1(poses[i].position.x, poses[i].position.y, poses[i].position.z);
    line->SetPoint2(poses[i + 1].position.x, poses[i + 1].position.y, poses[i + 1].position.z);

    vtkNew<vtkTubeFilter> tube;
    tube->SetInputConnection(line->GetOutputPort());
    tube->SetRadius(radius);
    tube->SetNumberOfSides(8);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(tube->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    assembly->AddPart(actor);
  }

  return assembly;
}

vtkSmartPointer<vtkPropAssembly> createToolpathActors(const std::vector<msg::Toolpath>& toolpaths,
                                                      vtkAlgorithmOutput* waypoint_shape_output_port)
{
  auto assembly = vtkSmartPointer<vtkPropAssembly>::New();
  for (const auto& tp : toolpaths)
  {
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(tp.poses.size());
    for (const auto& ps : tp.poses)
      poses.push_back(ps.pose);
    assembly->AddPart(createPoseActors(poses, waypoint_shape_output_port));
  }
  return assembly;
}

vtkSmartPointer<vtkPropAssembly> createToolpathLines(const std::vector<msg::Toolpath>& toolpaths,
                                                     double radius,
                                                     const std::array<double, 3>& color)
{
  auto assembly = vtkSmartPointer<vtkPropAssembly>::New();
  for (const auto& tp : toolpaths)
  {
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(tp.poses.size());
    for (const auto& ps : tp.poses)
      poses.push_back(ps.pose);
    assembly->AddPart(createPolylineActors(poses, radius, color));
  }
  return assembly;
}

vtkSmartPointer<vtkPropAssembly> createHoleActors(const msg::HoleArray& holes,
                                                  vtkAlgorithmOutput* waypoint_shape_output_port,
                                                  double sphere_radius)
{
  auto assembly = vtkSmartPointer<vtkPropAssembly>::New();
  for (const auto& hole : holes.holes)
  {
    const Eigen::Vector3d center{ hole.pose.position.x, hole.pose.position.y, hole.pose.position.z };
    assembly->AddPart(makeSphere(center, sphere_radius, { 0.1, 0.7, 0.3 }));

    std::vector<geometry_msgs::msg::Pose> pose_list{ hole.pose };
    auto pose_actor = createPoseActors(pose_list, waypoint_shape_output_port);
    assembly->AddPart(pose_actor);
  }
  return assembly;
}
}  // namespace

HolePlannerWindow::HolePlannerWindow(QWidget* parent)
  : QMainWindow(parent)
  , renderer_(vtkSmartPointer<vtkOpenGLRenderer>::New())
  , mesh_mapper_(vtkSmartPointer<vtkOpenGLPolyDataMapper>::New())
  , mesh_actor_(vtkSmartPointer<vtkOpenGLActor>::New())
  , hole_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , toolpath_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , toolpath_lines_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , axes_(vtkSmartPointer<vtkAxes>::New())
  , axes_actor_(vtkSmartPointer<vtkAxesActor>::New())
  , tube_filter_(vtkSmartPointer<vtkTubeFilter>::New())
{
  setWindowTitle("Hole Toolpath Planner");

  render_widget_ = new QVTKOpenGLNativeWidget(this);
  setCentralWidget(render_widget_);

  // Toolbar
  auto* toolbar = addToolBar("Actions");
  action_load_mesh_ = toolbar->addAction("Load Mesh", this, &HolePlannerWindow::onLoadMesh);
  action_load_params_ = toolbar->addAction("Load Params", this, &HolePlannerWindow::onLoadParameters);
  toolbar->addSeparator();
  action_run_ = toolbar->addAction("Run Planner", this, &HolePlannerWindow::plan);
  toolbar->addSeparator();
  action_save_holes_ = toolbar->addAction("Save Holes", this, &HolePlannerWindow::onSaveHoles);
  action_save_toolpaths_ = toolbar->addAction("Save Toolpaths", this, &HolePlannerWindow::onSaveToolpaths);

  // Dock with controls
  auto* dock = new QDockWidget("Settings", this);
  dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
  auto* dock_widget = new QWidget(dock);
  auto* layout = new QVBoxLayout(dock_widget);
  layout->setContentsMargins(6, 6, 6, 6);

  auto* request_form = new QFormLayout();
  request_form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
  min_diameter_ = new QDoubleSpinBox(dock_widget);
  min_diameter_->setDecimals(4);
  min_diameter_->setRange(0.0, 1.0);
  min_diameter_->setSingleStep(0.001);
  min_diameter_->setValue(0.0);
  max_diameter_ = new QDoubleSpinBox(dock_widget);
  max_diameter_->setDecimals(4);
  max_diameter_->setRange(0.0, 1.0);
  max_diameter_->setSingleStep(0.001);
  max_diameter_->setValue(0.05);
  min_length_ = new QDoubleSpinBox(dock_widget);
  min_length_->setDecimals(4);
  min_length_->setRange(0.0, 1.0);
  min_length_->setSingleStep(0.001);
  min_length_->setValue(0.0);
  watertight_hint_ = new QCheckBox(dock_widget);
  watertight_hint_->setChecked(false);

  request_form->addRow(new QLabel("Min diameter (m)", dock_widget), min_diameter_);
  request_form->addRow(new QLabel("Max diameter (m)", dock_widget), max_diameter_);
  request_form->addRow(new QLabel("Min length (m)", dock_widget), min_length_);
  request_form->addRow(new QLabel("Watertight hint", dock_widget), watertight_hint_);

  auto* view_form = new QFormLayout();
  show_mesh_ = new QCheckBox("Show mesh", dock_widget);
  show_mesh_->setChecked(true);
  show_holes_ = new QCheckBox("Show holes", dock_widget);
  show_holes_->setChecked(true);
  show_toolpaths_ = new QCheckBox("Show poses", dock_widget);
  show_toolpaths_->setChecked(true);
  show_toolpath_lines_ = new QCheckBox("Show lines", dock_widget);
  show_toolpath_lines_->setChecked(true);
  show_axes_ = new QCheckBox("Show world axes", dock_widget);
  show_axes_->setChecked(true);

  axis_size_ = new QDoubleSpinBox(dock_widget);
  axis_size_->setDecimals(3);
  axis_size_->setRange(0.001, 1.0);
  axis_size_->setValue(0.025);
  axis_size_->setSingleStep(0.005);

  origin_size_ = new QDoubleSpinBox(dock_widget);
  origin_size_->setDecimals(3);
  origin_size_->setRange(0.001, 1.0);
  origin_size_->setValue(0.05);
  origin_size_->setSingleStep(0.005);

  view_form->addRow(show_mesh_);
  view_form->addRow(show_holes_);
  view_form->addRow(show_toolpaths_);
  view_form->addRow(show_toolpath_lines_);
  view_form->addRow(show_axes_);
  view_form->addRow(new QLabel("Pose axis size (m)", dock_widget), axis_size_);
  view_form->addRow(new QLabel("World axis size (m)", dock_widget), origin_size_);

  auto* toolpath_form = new QFormLayout();
  generate_toolpaths_ = new QCheckBox("Generate toolpaths", dock_widget);
  generate_toolpaths_->setChecked(true);
  approach_offset_ = new QDoubleSpinBox(dock_widget);
  approach_offset_->setDecimals(3);
  approach_offset_->setRange(0.0, 1.0);
  approach_offset_->setSingleStep(0.001);
  approach_offset_->setValue(0.01);

  stepdown_ = new QDoubleSpinBox(dock_widget);
  stepdown_->setDecimals(3);
  stepdown_->setRange(0.0, 1.0);
  stepdown_->setSingleStep(0.001);
  stepdown_->setValue(0.002);

  spiral_pitch_ = new QDoubleSpinBox(dock_widget);
  spiral_pitch_->setDecimals(3);
  spiral_pitch_->setRange(0.0, 1.0);
  spiral_pitch_->setSingleStep(0.001);
  spiral_pitch_->setValue(0.005);

  toolpath_form->addRow(generate_toolpaths_);
  toolpath_form->addRow(new QLabel("Approach offset (m)", dock_widget), approach_offset_);
  toolpath_form->addRow(new QLabel("Stepdown (m)", dock_widget), stepdown_);
  toolpath_form->addRow(new QLabel("Spiral pitch (m)", dock_widget), spiral_pitch_);

  layout->addWidget(new QLabel("Detection request", dock_widget));
  layout->addLayout(request_form);
  layout->addSpacing(8);
  layout->addWidget(new QLabel("Toolpath", dock_widget));
  layout->addLayout(toolpath_form);
  layout->addSpacing(8);
  layout->addWidget(new QLabel("View", dock_widget));
  layout->addLayout(view_form);
  layout->addStretch(1);

  dock_widget->setLayout(layout);
  dock->setWidget(dock_widget);
  addDockWidget(Qt::LeftDockWidgetArea, dock);

  // VTK scene setup
  renderer_->SetBackground(0.2, 0.2, 0.2);
  mesh_actor_->SetMapper(mesh_mapper_);
  renderer_->AddActor(mesh_actor_);

  renderer_->AddActor(hole_actor_);
  renderer_->AddActor(toolpath_actor_);
  renderer_->AddActor(toolpath_lines_actor_);

  axes_->SetScaleFactor(axis_size_->value());
  tube_filter_->SetInputConnection(axes_->GetOutputPort());
  tube_filter_->SetRadius(axes_->GetScaleFactor() / 10.0);
  tube_filter_->SetNumberOfSides(10);
  tube_filter_->CappingOn();

  axes_actor_->SetTotalLength(origin_size_->value(), origin_size_->value(), origin_size_->value());
  renderer_->AddActor(axes_actor_);

  vtkRenderWindow* window = render_widget_->renderWindow();
  window->AddRenderer(renderer_);
  render_widget_->interactor()->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());

  // Visibility bindings
  connect(show_mesh_, &QCheckBox::toggled, [this](bool checked) {
    mesh_actor_->SetVisibility(checked);
    render();
  });
  connect(show_holes_, &QCheckBox::toggled, [this](bool checked) {
    if (hole_actor_)
      hole_actor_->SetVisibility(checked);
    render();
  });
  connect(show_toolpaths_, &QCheckBox::toggled, [this](bool checked) {
    if (toolpath_actor_)
      toolpath_actor_->SetVisibility(checked);
    render();
  });
  connect(show_toolpath_lines_, &QCheckBox::toggled, [this](bool checked) {
    if (toolpath_lines_actor_)
      toolpath_lines_actor_->SetVisibility(checked);
    render();
  });
  connect(show_axes_, &QCheckBox::toggled, [this](bool checked) {
    axes_actor_->SetVisibility(checked);
    render();
  });
  connect(axis_size_, &QDoubleSpinBox::editingFinished, [this]() {
    axes_->SetScaleFactor(axis_size_->value());
    tube_filter_->SetRadius(axes_->GetScaleFactor() / 10.0);
    rebuildHoleActors();
    rebuildToolpathActors();
    render();
  });
  connect(origin_size_, &QDoubleSpinBox::editingFinished, [this]() {
    axes_actor_->SetTotalLength(origin_size_->value(), origin_size_->value(), origin_size_->value());
    render();
  });

  mesh_actor_->SetVisibility(show_mesh_->isChecked());
  hole_actor_->SetVisibility(show_holes_->isChecked());
  toolpath_actor_->SetVisibility(show_toolpaths_->isChecked());
  toolpath_lines_actor_->SetVisibility(show_toolpath_lines_->isChecked());
  axes_actor_->SetVisibility(show_axes_->isChecked());

  rebuildDetector();
}

HolePlannerWindow::~HolePlannerWindow() = default;

void HolePlannerWindow::render()
{
  render_widget_->renderWindow()->Render();
  render_widget_->renderWindow()->Render();
}

void HolePlannerWindow::rebuildDetector()
{
  rclcpp::NodeOptions options;
  if (!params_file_.isEmpty())
  {
    options.arguments({ "--ros-args", "--params-file", params_file_.toStdString() });
  }

  node_ = std::make_shared<rclcpp::Node>("hole_toolpath_planner_gui", options);
  params_ = declare_and_get_parameters(*node_);
  detector_ = std::make_unique<HoleDetector>(*node_, params_);
  refreshParametersUi();
}

void HolePlannerWindow::refreshParametersUi()
{
  const bool default_generate = params_file_.isEmpty() ? true : params_.toolpath.generate;
  generate_toolpaths_->setChecked(default_generate);
  approach_offset_->setValue(params_.toolpath.approach_offset);
  stepdown_->setValue(params_.toolpath.stepdown);
  spiral_pitch_->setValue(params_.toolpath.spiral_pitch);
}

void HolePlannerWindow::setMeshFile(const QString& file)
{
  vtkSmartPointer<vtkAbstractPolyDataReader> reader;
  if (file.endsWith(".ply"))
    reader = vtkSmartPointer<vtkPLYReader>::New();
  else if (file.endsWith(".stl"))
    reader = vtkSmartPointer<vtkSTLReader>::New();
  else
    return;

  mesh_file_ = file.toStdString();

  reader->SetFileName(mesh_file_.c_str());
  reader->Update();
  mesh_mapper_->SetInputData(reader->GetOutput());
  renderer_->ResetCamera();
  render();
}

void HolePlannerWindow::loadParameters(const QString& file)
{
  params_file_ = file;
  rebuildDetector();
}

void HolePlannerWindow::plan()
{
  if (!detector_)
  {
    QMessageBox::warning(this, "Planner", "Planner not ready; failed to build detector.");
    return;
  }

  if (mesh_file_.empty())
  {
    QMessageBox::warning(this, "Planner", "Load a mesh first.");
    return;
  }

  detect_request_.mesh_path = mesh_file_;
  detect_request_.min_diameter = static_cast<float>(min_diameter_->value());
  detect_request_.max_diameter = static_cast<float>(max_diameter_->value());
  detect_request_.min_length = static_cast<float>(min_length_->value());
  detect_request_.watertight_hint = watertight_hint_->isChecked();

  params_.toolpath.generate = generate_toolpaths_->isChecked();
  params_.toolpath.approach_offset = approach_offset_->value();
  params_.toolpath.stepdown = stepdown_->value();
  params_.toolpath.spiral_pitch = spiral_pitch_->value();
  detector_ = std::make_unique<HoleDetector>(*node_, params_);

  try
  {
    QApplication::setOverrideCursor(Qt::WaitCursor);
    holes_ = detector_->detect(detect_request_);
    const rclcpp::Time stamp(holes_.header.stamp);
    toolpaths_ = detector_->make_toolpaths(holes_, stamp);
    QApplication::restoreOverrideCursor();

    rebuildHoleActors();
    rebuildToolpathActors();
    render();
  }
  catch (const std::exception& ex)
  {
    QApplication::restoreOverrideCursor();
    QMessageBox::warning(this, "Planner error", ex.what());
  }
}

void HolePlannerWindow::rebuildHoleActors()
{
  renderer_->RemoveActor(hole_actor_);
  hole_actor_ = createHoleActors(holes_, tube_filter_->GetOutputPort(), axis_size_->value() * 0.5);
  renderer_->AddActor(hole_actor_);
  hole_actor_->SetVisibility(show_holes_->isChecked());
}

void HolePlannerWindow::rebuildToolpathActors()
{
  renderer_->RemoveActor(toolpath_actor_);
  renderer_->RemoveActor(toolpath_lines_actor_);

  toolpath_actor_ = createToolpathActors(toolpaths_, tube_filter_->GetOutputPort());
  toolpath_lines_actor_ = createToolpathLines(toolpaths_, axis_size_->value() * 0.2, { 0.2, 0.4, 0.9 });

  renderer_->AddActor(toolpath_actor_);
  renderer_->AddActor(toolpath_lines_actor_);

  toolpath_actor_->SetVisibility(show_toolpaths_->isChecked());
  toolpath_lines_actor_->SetVisibility(show_toolpath_lines_->isChecked());
}

void HolePlannerWindow::onLoadMesh()
{
  const QString file = QFileDialog::getOpenFileName(this, "Load mesh file", "", "Mesh files (*.ply *.stl)");
  if (!file.isNull())
    setMeshFile(file);
}

void HolePlannerWindow::onLoadParameters()
{
  const QString file =
      QFileDialog::getOpenFileName(this, "Load ROS params", "", "YAML files (*.yaml *.yml);;All files (*.*)");
  if (!file.isNull())
    loadParameters(file);
}

void HolePlannerWindow::onSaveHoles()
{
  if (holes_.holes.empty())
  {
    QMessageBox::information(this, "Save holes", "No detections to save yet.");
    return;
  }

  const QString file =
      QFileDialog::getSaveFileName(this, "Save holes YAML", "", "YAML files (*.yaml *.yml);;All files (*.*)");
  if (file.isEmpty())
    return;

  std::ofstream out(file.toStdString());
  if (!out)
  {
    QMessageBox::warning(this, "Save holes", "Failed to open file for writing.");
    return;
  }

  out << "holes:\n";
  for (const auto& hole : holes_.holes)
  {
    out << "  - id: " << hole.id << "\n";
    out << "    diameter: " << hole.diameter << "\n";
    out << "    length: " << hole.length << "\n";
    out << "    pose:\n";
    out << "      position: [" << hole.pose.position.x << ", " << hole.pose.position.y << ", "
        << hole.pose.position.z << "]\n";
    out << "      orientation: [" << hole.pose.orientation.w << ", " << hole.pose.orientation.x << ", "
        << hole.pose.orientation.y << ", " << hole.pose.orientation.z << "]\n";
    out << "    axis: [" << hole.axis.x << ", " << hole.axis.y << ", " << hole.axis.z << "]\n";
  }
}

void HolePlannerWindow::onSaveToolpaths()
{
  if (toolpaths_.empty())
  {
    QMessageBox::information(this, "Save toolpaths", "No toolpaths to save yet.");
    return;
  }

  const QString file =
      QFileDialog::getSaveFileName(this, "Save toolpaths YAML", "", "YAML files (*.yaml *.yml);;All files (*.*)");
  if (file.isEmpty())
    return;

  std::ofstream out(file.toStdString());
  if (!out)
  {
    QMessageBox::warning(this, "Save toolpaths", "Failed to open file for writing.");
    return;
  }

  out << "toolpaths:\n";
  for (const auto& tp : toolpaths_)
  {
    out << "  - hole_id: " << tp.hole_id << "\n";
    out << "    strategy: \"" << tp.strategy << "\"\n";
    out << "    stepdown: " << tp.stepdown << "\n";
    out << "    feedrate: " << tp.feedrate << "\n";
    out << "    poses:\n";
    for (const auto& ps : tp.poses)
    {
      out << "      - position: [" << ps.pose.position.x << ", " << ps.pose.position.y << ", " << ps.pose.position.z
          << "]\n";
      out << "        orientation: [" << ps.pose.orientation.w << ", " << ps.pose.orientation.x << ", "
          << ps.pose.orientation.y << ", " << ps.pose.orientation.z << "]\n";
    }
  }
}

}  // namespace hole_toolpath_planner
