
#include "mainwindow.h"


void init(){
    ros::spin();
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "guiofbluerov2");
    ros::start();
    ros::NodeHandle n_;
    rosHandlerGui rosHandler(n_);
    std::thread t1(init);

    QApplication app(argc, argv);
    MainWindow mainWindow;

    mainWindow.showMaximized();


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
    QObject::connect(&mainWindow, &MainWindow::sendSonarRange, &rosHandler, &rosHandlerGui::setSonarRange);
    QObject::connect(&mainWindow, &MainWindow::updateDesiredState, &rosHandler, &rosHandlerGui::updateDesiredState);


    return app.exec();
}
