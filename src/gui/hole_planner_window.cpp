#include "hole_toolpath_planner/gui/hole_planner_window.hpp"

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QDockWidget>
#include <QFileDialog>
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QToolBar>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkAbstractPolyDataReader.h>
#include <vtkAxesActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkOpenGLActor.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLRenderer.h>
#include <vtkPLYReader.h>
#include <vtkPropAssembly.h>
#include <vtkRenderWindow.h>
#include <vtkProperty.h>
#include <vtkSTLReader.h>
#include <vtkTransform.h>

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

vtkSmartPointer<vtkAxesActor> createHoleAxisActor(const geometry_msgs::msg::Pose& pose, double axis_size)
{
  auto actor = vtkSmartPointer<vtkAxesActor>::New();
  actor->SetTotalLength(axis_size, axis_size, axis_size);
  actor->SetAxisLabels(false);
  actor->GetXAxisShaftProperty()->SetColor(1.0, 0.0, 0.0);
  actor->GetYAxisShaftProperty()->SetColor(0.0, 1.0, 0.0);
  actor->GetZAxisShaftProperty()->SetColor(0.0, 0.0, 1.0);
  actor->SetUserTransform(toVTK(toEigen(pose)));
  return actor;
}
}  // namespace

HolePlannerWindow::HolePlannerWindow(QWidget* parent)
  : QMainWindow(parent)
  , renderer_(vtkSmartPointer<vtkOpenGLRenderer>::New())
  , mesh_mapper_(vtkSmartPointer<vtkOpenGLPolyDataMapper>::New())
  , mesh_actor_(vtkSmartPointer<vtkOpenGLActor>::New())
  , hole_actor_(vtkSmartPointer<vtkPropAssembly>::New())
  , axes_actor_(vtkSmartPointer<vtkAxesActor>::New())
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
  action_save_holes_ = toolbar->addAction("Save Hole Poses", this, &HolePlannerWindow::onSaveHoles);

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

  min_diameter_label_ = new QLabel("Min diameter (m)", dock_widget);
  max_diameter_label_ = new QLabel("Max diameter (m)", dock_widget);
  min_length_label_ = new QLabel("Min length (m)", dock_widget);
  request_form->addRow(min_diameter_label_, min_diameter_);
  request_form->addRow(max_diameter_label_, max_diameter_);
  request_form->addRow(min_length_label_, min_length_);
  request_form->addRow(new QLabel("Watertight hint", dock_widget), watertight_hint_);

  auto* view_form = new QFormLayout();
  show_mesh_ = new QCheckBox("Show mesh", dock_widget);
  show_mesh_->setChecked(true);
  show_holes_ = new QCheckBox("Show holes", dock_widget);
  show_holes_->setChecked(true);
  show_axes_ = new QCheckBox("Show world axes", dock_widget);
  show_axes_->setChecked(true);
  show_table_ = new QCheckBox("Show table", dock_widget);
  show_table_->setChecked(false);
  imperial_units_ = new QCheckBox("Imperial units (in)", dock_widget);
  imperial_units_->setChecked(false);

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
  view_form->addRow(show_axes_);
  view_form->addRow(show_table_);
  axis_size_label_ = new QLabel("Pose axis size (m)", dock_widget);
  origin_size_label_ = new QLabel("World axis size (m)", dock_widget);
  view_form->addRow(axis_size_label_, axis_size_);
  view_form->addRow(origin_size_label_, origin_size_);
  view_form->addRow(imperial_units_);

  layout->addWidget(new QLabel("Detection request", dock_widget));
  layout->addLayout(request_form);
  layout->addSpacing(8);
  layout->addWidget(new QLabel("View", dock_widget));
  layout->addLayout(view_form);
  layout->addStretch(1);

  show_all_holes_ = new QCheckBox("Show all holes", dock_widget);
  show_all_holes_->setChecked(true);
  hole_table_ = new QTableWidget(dock_widget);
  hole_table_->setColumnCount(4);
  hole_table_->setHorizontalHeaderLabels(QStringList() << "Show"
                                                       << "Center (m)"
                                                       << "Diameter (mm)"
                                                       << "Length (mm)");
  hole_table_->setSortingEnabled(false);
  hole_table_->horizontalHeader()->setStretchLastSection(true);
  hole_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  hole_table_->setEditTriggers(QAbstractItemView::AllEditTriggers);
  hole_table_->setVisible(false);
  show_all_holes_->setVisible(false);

  layout->addSpacing(8);
  layout->addWidget(show_all_holes_);
  layout->addWidget(hole_table_);

  dock_widget->setLayout(layout);
  dock->setWidget(dock_widget);
  addDockWidget(Qt::LeftDockWidgetArea, dock);

  // VTK scene setup
  renderer_->SetBackground(0.2, 0.2, 0.2);
  mesh_actor_->SetMapper(mesh_mapper_);
  renderer_->AddActor(mesh_actor_);

  renderer_->AddActor(hole_actor_);
  const double origin_size_m = origin_size_->value();
  axes_actor_->SetTotalLength(origin_size_m, origin_size_m, origin_size_m);
  axes_actor_->SetXAxisLabelText("X");
  axes_actor_->SetYAxisLabelText("Y");
  axes_actor_->SetZAxisLabelText("Z");
  axes_actor_->GetXAxisShaftProperty()->SetColor(1.0, 0.0, 0.0);
  axes_actor_->GetYAxisShaftProperty()->SetColor(0.0, 1.0, 0.0);
  axes_actor_->GetZAxisShaftProperty()->SetColor(0.0, 0.0, 1.0);
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
  connect(show_axes_, &QCheckBox::toggled, [this](bool checked) {
    axes_actor_->SetVisibility(checked);
    render();
  });
  connect(show_table_, &QCheckBox::toggled, [this](bool checked) {
    show_all_holes_->setVisible(checked);
    hole_table_->setVisible(checked);
    show_holes_->setChecked(checked);
    hole_actor_->SetVisibility(checked);
    render();
  });
  connect(hole_table_, &QTableWidget::itemChanged, [this](QTableWidgetItem* item) {
    if (item->column() != 0)
      return;
    updateHoleVisibility(item->row());
  });
  connect(hole_table_, &QTableWidget::cellClicked, [this](int row, int column) {
    if (column != 0)
      return;
    updateHoleVisibility(row);
  });
  connect(show_all_holes_, &QCheckBox::toggled, [this](bool checked) {
    for (int row = 0; row < hole_table_->rowCount(); ++row)
    {
      if (auto* item = hole_table_->item(row, 0))
        item->setCheckState(checked ? Qt::Checked : Qt::Unchecked);
    }
    for (std::size_t i = 0; i < hole_axes_.size(); ++i)
      hole_axes_[i]->SetVisibility(checked);
    hole_actor_->Modified();
    render();
  });
  connect(axis_size_, &QDoubleSpinBox::editingFinished, [this]() {
    rebuildHoleActors();
    render();
  });
  connect(origin_size_, &QDoubleSpinBox::editingFinished, [this]() {
    const double unit_scale = imperial_units_->isChecked() ? 0.0254 : 1.0;
    const double origin_size_m = origin_size_->value() * unit_scale;
    axes_actor_->SetTotalLength(origin_size_m, origin_size_m, origin_size_m);
    render();
  });
  connect(imperial_units_, &QCheckBox::toggled, [this](bool checked) {
    const double scale = checked ? 1.0 / 0.0254 : 0.0254;
    min_diameter_->setValue(min_diameter_->value() * scale);
    max_diameter_->setValue(max_diameter_->value() * scale);
    min_length_->setValue(min_length_->value() * scale);
    axis_size_->setValue(axis_size_->value() * scale);
    origin_size_->setValue(origin_size_->value() * scale);

    min_diameter_label_->setText(checked ? "Min diameter (in)" : "Min diameter (m)");
    max_diameter_label_->setText(checked ? "Max diameter (in)" : "Max diameter (m)");
    min_length_label_->setText(checked ? "Min length (in)" : "Min length (m)");
    axis_size_label_->setText(checked ? "Pose axis size (in)" : "Pose axis size (m)");
    origin_size_label_->setText(checked ? "World axis size (in)" : "World axis size (m)");
    hole_table_->setHorizontalHeaderLabels(QStringList() << "Show"
                                                         << QString("Center (%1)").arg(checked ? "in" : "m")
                                                         << QString("Diameter (%1)").arg(checked ? "in" : "mm")
                                                         << QString("Length (%1)").arg(checked ? "in" : "mm"));

    refreshHoleTable();
    rebuildHoleActors();
    const double unit_scale = checked ? 0.0254 : 1.0;
    const double origin_size_m = origin_size_->value() * unit_scale;
    axes_actor_->SetTotalLength(origin_size_m, origin_size_m, origin_size_m);
    render();
  });

  mesh_actor_->SetVisibility(show_mesh_->isChecked());
  hole_actor_->SetVisibility(show_holes_->isChecked());
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
  (void)params_;
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
  const double unit_scale = imperial_units_->isChecked() ? 0.0254 : 1.0;
  detect_request_.min_diameter = static_cast<float>(min_diameter_->value() * unit_scale);
  detect_request_.max_diameter = static_cast<float>(max_diameter_->value() * unit_scale);
  detect_request_.min_length = static_cast<float>(min_length_->value() * unit_scale);
  detect_request_.watertight_hint = watertight_hint_->isChecked();

  detector_ = std::make_unique<HoleDetector>(*node_, params_);

  try
  {
    QApplication::setOverrideCursor(Qt::WaitCursor);
    holes_ = detector_->detect(detect_request_);
    QApplication::restoreOverrideCursor();

    rebuildHoleActors();
    refreshHoleTable();
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
  hole_actor_ = vtkSmartPointer<vtkPropAssembly>::New();
  hole_axes_.clear();
  const double axis_size_m = axis_size_->value() * (imperial_units_->isChecked() ? 0.0254 : 1.0);
  hole_axes_.reserve(holes_.holes.size());
  for (const auto& hole : holes_.holes)
  {
    auto actor = createHoleAxisActor(hole.pose, axis_size_m);
    hole_actor_->AddPart(actor);
    hole_axes_.push_back(actor);
  }
  renderer_->AddActor(hole_actor_);
  hole_actor_->SetVisibility(show_holes_->isChecked());
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
    QMessageBox::information(this, "Save hole poses", "No detections to save yet.");
    return;
  }

  std::vector<std::size_t> selected;
  selected.reserve(holes_.holes.size());
  if (hole_table_->rowCount() == static_cast<int>(holes_.holes.size()))
  {
    for (int row = 0; row < hole_table_->rowCount(); ++row)
    {
      const auto* item = hole_table_->item(row, 0);
      if (item && item->checkState() == Qt::Checked)
        selected.push_back(static_cast<std::size_t>(row));
    }
  }
  else
  {
    for (std::size_t i = 0; i < holes_.holes.size(); ++i)
      selected.push_back(i);
  }

  if (selected.empty())
  {
    QMessageBox::information(this, "Save hole poses", "No holes selected in the table.");
    return;
  }

  const QString file =
      QFileDialog::getSaveFileName(this, "Save hole poses YAML", "", "YAML files (*.yaml *.yml);;All files (*.*)");
  if (file.isEmpty())
    return;

  std::ofstream out(file.toStdString());
  if (!out)
  {
    QMessageBox::warning(this, "Save hole poses", "Failed to open file for writing.");
    return;
  }

  const int64_t stamp_sec = 0;
  const int64_t stamp_nsec = 0;
  const std::string frame_id =
      params_.logging.frame_id.empty() ? std::string("world") : params_.logging.frame_id;

  for (std::size_t index : selected)
  {
    const auto& hole = holes_.holes[index];
    out << "---\n";
    out << "header:\n";
    out << "  stamp:\n";
    out << "    sec: " << stamp_sec << "\n";
    out << "    nanosec: " << stamp_nsec << "\n";
    out << "  frame_id: " << frame_id << "\n";
    out << "pose:\n";
    out << "  position:\n";
    out << "    x: " << hole.pose.position.x << "\n";
    out << "    y: " << hole.pose.position.y << "\n";
    out << "    z: " << hole.pose.position.z << "\n";
    out << "  orientation:\n";
    out << "    x: " << hole.pose.orientation.x << "\n";
    out << "    y: " << hole.pose.orientation.y << "\n";
    out << "    z: " << hole.pose.orientation.z << "\n";
    out << "    w: " << hole.pose.orientation.w << "\n";
  }
}

void HolePlannerWindow::refreshHoleTable()
{
  hole_table_->blockSignals(true);
  hole_table_->setRowCount(static_cast<int>(holes_.holes.size()));
  for (int row = 0; row < static_cast<int>(holes_.holes.size()); ++row)
  {
    const auto& hole = holes_.holes[static_cast<std::size_t>(row)];

    auto* show_item = new QTableWidgetItem();
    show_item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
    show_item->setCheckState(Qt::Checked);
    hole_table_->setItem(row, 0, show_item);

    const bool imperial = imperial_units_->isChecked();
    const double unit_scale = imperial ? (1.0 / 0.0254) : 1.0;
    const double center_x = hole.pose.position.x * unit_scale;
    const double center_y = hole.pose.position.y * unit_scale;
    const double center_z = hole.pose.position.z * unit_scale;

    const double diameter_mm = hole.diameter * 1000.0;
    const double length_mm = hole.length * 1000.0;
    const double diameter_unit = imperial ? (hole.diameter * unit_scale) : diameter_mm;
    const double length_unit = imperial ? (hole.length * unit_scale) : length_mm;

    hole_table_->setItem(
        row,
        1,
        new QTableWidgetItem(
            QString("[%1] (%2, %3, %4)")
                .arg(hole.id, 2, 10, QChar('0'))
                .arg(center_x, 0, 'f', 4)
                .arg(center_y, 0, 'f', 4)
                .arg(center_z, 0, 'f', 4)));
    hole_table_->setItem(row, 2, new QTableWidgetItem(QString::number(diameter_unit, 'f', imperial ? 4 : 3)));
    hole_table_->setItem(row, 3, new QTableWidgetItem(QString::number(length_unit, 'f', imperial ? 4 : 3)));
  }

  show_all_holes_->blockSignals(true);
  show_all_holes_->setChecked(true);
  show_all_holes_->blockSignals(false);
  hole_table_->blockSignals(false);
}

void HolePlannerWindow::updateHoleVisibility(int row)
{
  if (row < 0 || static_cast<std::size_t>(row) >= hole_axes_.size())
    return;
  const auto* item = hole_table_->item(row, 0);
  if (!item)
    return;
  const bool visible = item->checkState() == Qt::Checked;
  hole_axes_[static_cast<std::size_t>(row)]->SetVisibility(visible);
  hole_actor_->Modified();
  render();
}

}  // namespace hole_toolpath_planner
