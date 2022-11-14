
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
//    Q_INIT_RESOURCE(application);
    QApplication app(argc, argv);
//    std::cout << QT_VERSION_STR << std::endl;
//    QCoreApplication::setApplicationVersion(QT_VERSION_STR);

    qRegisterMetaType<Eigen::MatrixXd>("Eigen::MatrixXd");
    qRegisterMetaType<std::vector<double>>("std::vector<double>");
    MainWindow mainWindow;


    std::thread t2(&MainWindow::threadSendCurrentDesiredPoseRobot, &mainWindow);

    mainWindow.setStyleSheet("background-color: rgb(177,205,186); ");
    //ROS to GUI
    QObject::connect(&rosHandler, &rosHandlerGui::updatePlotPositionVectorROS,
                     &mainWindow, &MainWindow::updateStateForPlotting,Qt::BlockingQueuedConnection);
    QObject::connect(&rosHandler, &rosHandlerGui::updateStateOfRobotROS,
                     &mainWindow, &MainWindow::updateStateOfRobot,Qt::AutoConnection);
    QObject::connect(&rosHandler, &rosHandlerGui::updateSonarImageROS,
                     &mainWindow, &MainWindow::updateSonarImage,Qt::BlockingQueuedConnection);
    QObject::connect(&rosHandler, &rosHandlerGui::updateCameraImageROS,
                     &mainWindow, &MainWindow::updateCameraImage,Qt::BlockingQueuedConnection);
    //GUI to ROS
    QObject::connect(&mainWindow, &MainWindow::updateDesiredState, &rosHandler, &rosHandlerGui::updateDesiredState,Qt::AutoConnection);
    QObject::connect(&mainWindow, &MainWindow::resetEKFEstimator, &rosHandler, &rosHandlerGui::resetEKFEstimator,Qt::AutoConnection);
    QObject::connect(&mainWindow, &MainWindow::updateConfigSonar, &rosHandler, &rosHandlerGui::updateConfigSonar,Qt::AutoConnection);
    QObject::connect(&mainWindow, &MainWindow::updateAngleCamera, &rosHandler, &rosHandlerGui::updateAngleCamera,Qt::AutoConnection);
    QObject::connect(&mainWindow, &MainWindow::updateLightIntensity, &rosHandler, &rosHandlerGui::updateLightIntensity,Qt::AutoConnection);



    mainWindow.showMaximized();
    return app.exec();
}
