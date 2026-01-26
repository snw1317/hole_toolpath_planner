#include "hole_toolpath_planner/gui/hole_planner_window.hpp"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <rclcpp/rclcpp.hpp>

#include <signal.h>

using hole_toolpath_planner::HolePlannerWindow;

void handleSignal(int /*sig*/) { QApplication::instance()->quit(); }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("hole_toolpath_planner_gui");
  QCoreApplication::setOrganizationName("hole_toolpath_planner");

  QCommandLineParser parser;
  parser.setApplicationDescription("Hole toolpath planner GUI");
  parser.addHelpOption();

  QCommandLineOption mesh_opt(QStringList() << "m"
                                            << "mesh",
                              "Mesh file to preload (.ply or .stl)",
                              "mesh");
  QCommandLineOption params_opt(QStringList() << "p"
                                              << "params",
                                "ROS parameters YAML file",
                                "params");
  parser.addOption(mesh_opt);
  parser.addOption(params_opt);
  parser.process(app);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  HolePlannerWindow window;
  const QString mesh_file = parser.value(mesh_opt);
  if (!mesh_file.isEmpty())
    window.setMeshFile(mesh_file);

  const QString params_file = parser.value(params_opt);
  if (!params_file.isEmpty())
    window.loadParameters(params_file);

  window.showMaximized();
  const int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
