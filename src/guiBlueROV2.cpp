
#include "mainwindow.h"
#include <thread>

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
    QObject::connect(&rosHandler, &rosHandlerGui::updatePositionsROS,
                     &mainWindow, &MainWindow::updatePositions);
    QObject::connect(&rosHandler, &rosHandlerGui::updateSonarImageROS,
                     &mainWindow, &MainWindow::updateSonarImage);
    QObject::connect(&rosHandler, &rosHandlerGui::updateCameraImageROS,
                     &mainWindow, &MainWindow::updateCameraImage);
    //GUI to ROS
    QObject::connect(&mainWindow, &MainWindow::sendSonarRange, &rosHandler, &rosHandlerGui::setSonarRange);


    //ros::spin();// as far as i understand not necessary because of app.exec.

    return app.exec();
}
