
#include "mainwindow.h"


//void init(){
//    ros::spin();
//}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "guiofbluerov2");
    ros::start();
    ros::NodeHandle n_;
    rosHandlerGui rosHandler(n_);
//    std::thread t1(init);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    QApplication app(argc, argv);
    qRegisterMetaType<Eigen::MatrixXd>("Eigen::MatrixXd");
    qRegisterMetaType<std::vector<double>>("std::vector<double>");
    MainWindow mainWindow;

    mainWindow.showMaximized();
    std::thread t2(&MainWindow::threadSendCurrentDesiredPoseRobot, &mainWindow);

    mainWindow.setStyleSheet("background-color: rgb(177,205,186); ");
    //ROS to GUI
    QObject::connect(&rosHandler, &rosHandlerGui::updatePlotPositionVectorROS,
                     &mainWindow, &MainWindow::updateStateForPlotting);
    QObject::connect(&rosHandler, &rosHandlerGui::updateStateOfRobotROS,
                     &mainWindow, &MainWindow::updateStateOfRobot);
    QObject::connect(&rosHandler, &rosHandlerGui::updateSonarImageROS,
                     &mainWindow, &MainWindow::updateSonarImage);
    QObject::connect(&rosHandler, &rosHandlerGui::updateCameraImageROS,
                     &mainWindow, &MainWindow::updateCameraImage);
    //GUI to ROS
    QObject::connect(&mainWindow, &MainWindow::updateDesiredState, &rosHandler, &rosHandlerGui::updateDesiredState);
    QObject::connect(&mainWindow, &MainWindow::resetEKFEstimator, &rosHandler, &rosHandlerGui::resetEKFEstimator);
    QObject::connect(&mainWindow, &MainWindow::updateConfigSonar, &rosHandler, &rosHandlerGui::updateConfigSonar);
    QObject::connect(&mainWindow, &MainWindow::updateAngleCamera, &rosHandler, &rosHandlerGui::updateAngleCamera);
    QObject::connect(&mainWindow, &MainWindow::updateLightIntensity, &rosHandler, &rosHandlerGui::updateLightIntensity);




    return app.exec();
}
