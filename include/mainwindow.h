//
// Created by tim-linux on 14.12.21.
//

#include "rosHandlerGui.h"

#ifndef BLUEROV2COMMON_MAINWINDOW_H
#define BLUEROV2COMMON_MAINWINDOW_H

class MainWindow : public QMainWindow {
Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr) {
        sonarRange = 2;
        sonarStepSize = 1;
        QScreen *screen = QGuiApplication::primaryScreen();
        QRect screenGeometry = screen->geometry();
        int screenHeight = screenGeometry.height();
        int screenWidth = screenGeometry.width();

        int sizeOfSlider = 315;
        int xposRangeSonar = screenWidth - 1.5 * sizeOfSlider;
        int yposRangeSonar = 0 + 100;
        this->desiredHeight = 0;
        this->desiredRoll = 0;
        this->desiredPitch = 0;
        this->desiredYaw = 0;
        this->desiredXMovement = 0;
        this->desiredYMovement = 0;
        this->holdPositionStatus = false;

        this->initializationSonarWindows(xposRangeSonar, yposRangeSonar, screenWidth, sizeOfSlider);
        this->initializationCurrentPosition(screenWidth);
        this->initializationSonarImage(screenWidth);
        this->initializationCameraImage(screenWidth);
        this->initializationSliderCameraLight(sizeOfSlider);
        this->initializationGamepad(screenWidth);


    }

public:
    void handleSonarSlider(int sonarRange);

    void handleSonarSliderReleased();

    void handleSonarStepSlider(int sonarRange);

    void handleSonarStepReleased();

    void handleEKFReset();

    void handleHoldPosition();

    void handleControlWithController();

    void handleLightSlider(int lightIntensity);

    void handleLightSliderReleased();

    void handleCameraAngleSlider(int cameraAngle);

    void handleCameraAngleSliderReleased();

    //Gamepad Handling Functions
    void handleHeight(double changeOfHeight);

    void handleRoll(double changeOfRoll);

    void handlePitch(double changeOfPitch);

    void handleYaw(double changeOfYaw);

    void handleLocalXMovement(double changeOfX);

    void handleLocalYMovement(double changeOfY);

    void changeHoldPositionStatus(bool holdPosition);

    void threadSendCurrentDesiredPoseRobot() {
        ros::Rate loop_rate(30);

        while (ros::ok()) {

            this->updateRightX(this->m_gamepad->axisRightX());
            this->updateRightY(this->m_gamepad->axisRightY());
            this->updateLeftX(this->m_gamepad->axisLeftX());
            this->updateXButton(this->m_gamepad->buttonA());
            this->updateSquareButton(this->m_gamepad->buttonY());
            this->updateCircleButton(this->m_gamepad->buttonB());
            this->updateTriangleButton(this->m_gamepad->buttonX());
            this->updateR1Button(this->m_gamepad->buttonR1());
            this->updateR2Button(this->m_gamepad->buttonR2());


            //std::cout << "Sending data from Gui To ROS: " << test<< std::endl;


            emit this->updateDesiredState(this->desiredHeight, this->desiredRoll, this->desiredPitch, this->desiredYaw,
                                          this->desiredXMovement, this->desiredYMovement, this->holdPositionStatus);
            ros::spinOnce();
            loop_rate.sleep();
        }


    }

public slots:

    void
    updateStateForPlotting(std::vector<double> xPositionRobot, std::vector<double> yPositionRobot,
                           std::vector<double> yawPositionRobot);

    void updateStateOfRobot(double xPos, double yPos, double zPos, double roll, double pitch, double yaw,
                            Eigen::MatrixXd covariance);//covariance is just 6 values

    void updateSonarImage(QPixmap sonarImage);

    void updateCameraImage(QPixmap cameraImage);

    void updateRightX(double value);

    void updateRightY(double value);

    void updateLeftX(double value);

    void updateLeftY(double value);

    void updateXButton(bool pressed);

    void updateSquareButton(bool pressed);

    void updateCircleButton(bool pressed);

    void updateTriangleButton(bool pressed);

    void updateR1Button(bool pressed);

    void updateR2Button(double pressedValue);

    void updateL1Button(bool pressed);

    void updateL2Button(bool pressed);

public:
signals:

    void sendSonarRange(double range);

    void updateDesiredState(double desiredHeight, double desiredRoll, double desiredPitch, double desiredYaw,
                            double desiredXMovement, double desiredYMovement, bool holdPosition);

private:
    QLabel *distanceToBottom, *depth, *plotOfPosition, *sonarLabel, *sonarTicks;
    QLabel *currentSonarRange, *sonarStepSizeLabel, *currentSonarStepSize, *sonarImageLabel;
    QLabel *lightLabel, *cameraImageLabel, *lightTicks, *currentLightIntensity, *cameraAngleLabel;
    QLabel *currentCameraAngle, *cameraAngleTicks;
    QLabel *currentXThrustLabel, *currentYThrustLabel, *currentHeightDesiredLabel, *currentDesiredRollLabel, *currentDesiredPitchLabel, *currentDesiredYawLabel;
    QPushButton *resetEKF, *holdPos;
    QSlider *rangeSonarSlider, *angularStepSizeSlider, *lightSlider, *cameraAngleSlider;
    int sonarRange, sonarStepSize, lightIntensity, cameraAngle;
    QCustomPlot *customPlot;
    QVector<double> xPositionRobot, yPositionRobot, yawPositionRobot;
    QPixmap *sonarImage, *cameraImage;
    QGamepad *m_gamepad;
    bool connectedGamepad;
    std::atomic<double> desiredHeight, desiredRoll, desiredPitch, desiredYaw, desiredXMovement, desiredYMovement;

    std::atomic<bool> holdPositionStatus;

    //current state Robot:
    std::atomic<double> currentHeight, currentRoll, currentPitch, currentYaw, currentXPos, currentYPos;


private:
    void initializationSonarWindows(int xposRangeSonar, int yposRangeSonar, int screenWidth, int sizeOfSlider) {
        //range sonar
        rangeSonarSlider = new QSlider(Qt::Horizontal, this);

        rangeSonarSlider->setFocusPolicy(Qt::StrongFocus);
        //rangeSonarSlider->setTickPosition(QSlider::TicksBelow);
        //rangeSonarSlider->setTickInterval(5);
        rangeSonarSlider->setMaximum(60);
        rangeSonarSlider->setMinimum(2);
        rangeSonarSlider->setGeometry(QRect(QPoint(xposRangeSonar, yposRangeSonar), QSize(sizeOfSlider, 20)));
        sonarLabel = new QLabel("Sonar Range:", this);
        sonarLabel->setGeometry(QRect(QPoint(screenWidth - 1.3 * sizeOfSlider, yposRangeSonar - 50), QSize(200, 50)));
        sonarTicks = new QLabel("2                                       30                                       60",
                                this);
        sonarTicks->setGeometry(QRect(QPoint(xposRangeSonar - 3, yposRangeSonar + 25), QSize(sizeOfSlider, 15)));
        currentSonarRange = new QLabel("0", this);
        currentSonarRange->setGeometry(
                QRect(QPoint(screenWidth - 0.8 * sizeOfSlider, yposRangeSonar - 50), QSize(200, 50)));
        connect(rangeSonarSlider, &QSlider::valueChanged, this, &MainWindow::handleSonarSlider);
        connect(rangeSonarSlider, &QSlider::sliderReleased, this, &MainWindow::handleSonarSliderReleased);

        //angular Step size
        yposRangeSonar = yposRangeSonar + 150;
        angularStepSizeSlider = new QSlider(Qt::Horizontal, this);
        angularStepSizeSlider->setFocusPolicy(Qt::StrongFocus);
        angularStepSizeSlider->setMaximum(10);
        angularStepSizeSlider->setMinimum(1);
        angularStepSizeSlider->setGeometry(QRect(QPoint(xposRangeSonar, yposRangeSonar), QSize(sizeOfSlider, 20)));
        sonarStepSizeLabel = new QLabel("Step Size:", this);
        sonarStepSizeLabel->setGeometry(
                QRect(QPoint(screenWidth - 1.3 * sizeOfSlider, yposRangeSonar - 50), QSize(200, 50)));
        sonarTicks = new QLabel("1                                       5                                       10",
                                this);
        sonarTicks->setGeometry(QRect(QPoint(xposRangeSonar - 3, yposRangeSonar + 25), QSize(sizeOfSlider, 15)));
        currentSonarStepSize = new QLabel("0", this);
        currentSonarStepSize->setGeometry(
                QRect(QPoint(screenWidth - 0.8 * sizeOfSlider, yposRangeSonar - 50), QSize(200, 50)));
        connect(angularStepSizeSlider, &QSlider::valueChanged, this, &MainWindow::handleSonarStepSlider);
        connect(angularStepSizeSlider, &QSlider::sliderReleased, this, &MainWindow::handleSonarStepReleased);
    }

    void initializationCurrentPosition(int screenWidth) {
        int sizePlot = 400;
        int distanceFromLeftCorner = 50;
        int sizeButtons = 120;
        resetEKF = new QPushButton("Reset EKF", this);
        // set size and location of the button
        resetEKF->setGeometry(QRect(QPoint(distanceFromLeftCorner, 500), QSize(sizeButtons, 40)));
        connect(resetEKF, &QPushButton::released, this, &MainWindow::handleEKFReset);
        this->holdPos = new QPushButton("hold Pos", this);
        // set size and location of the button
        this->holdPos->setGeometry(
                QRect(QPoint(distanceFromLeftCorner + sizePlot - sizeButtons, 500), QSize(sizeButtons, 40)));
        connect(this->holdPos, &QPushButton::released, this, &MainWindow::handleHoldPosition);

//        controlByHuman = new QPushButton("Direct Control", this);
        // set size and location of the button
//        controlByHuman->setGeometry(
//                QRect(QPoint(distanceFromLeftCorner + sizePlot - sizeButtons, 500), QSize(sizeButtons, 40)));
//        connect(controlByHuman, &QPushButton::released, this, &MainWindow::handleControlWithController);


        this->customPlot = new QCustomPlot(this);
        this->customPlot->setGeometry(QRect(QPoint(distanceFromLeftCorner, 550), QSize(sizePlot, sizePlot)));
// create graph and assign data to it:
        this->customPlot->addGraph();
        this->customPlot->graph(0)->setData(this->xPositionRobot, this->yPositionRobot);
// give the axes some labels:
        this->customPlot->xAxis->setLabel("x");
        this->customPlot->yAxis->setLabel("y");
// set axes ranges, so we see all data:
        double xMin = *std::min_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
        double xMax = *std::max_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
        double yMin = *std::min_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
        double yMax = *std::max_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
//        this->customPlot->xAxis->setRange(xMin-1, xMax+1);
//        this->customPlot->yAxis->setRange(yMin-1, yMax+1);
        this->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
        this->customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 2));
        this->customPlot->replot();

    }

    void initializationSonarImage(int screenWidth) {
        int sizeSonarImage = 500;
        this->sonarImageLabel = new QLabel("exampleImage", this);
        this->sonarImageLabel->setGeometry(
                QRect(QPoint(screenWidth - sizeSonarImage - 50, 500), QSize(sizeSonarImage, sizeSonarImage)));

        this->sonarImage = new QPixmap("/home/tim-linux/Pictures/file_example_JPG_100kB.jpg");
        this->sonarImageLabel->setPixmap(
                sonarImage->scaled(sonarImageLabel->width(), sonarImageLabel->height(), Qt::KeepAspectRatio));
    }

    void initializationCameraImage(int screenWidth) {
        int sizeCameraImage = 500;
        this->cameraImageLabel = new QLabel("exampleImage", this);
        this->cameraImageLabel->setGeometry(
                QRect(QPoint(screenWidth / 2 - sizeCameraImage / 2, 500), QSize(sizeCameraImage, sizeCameraImage)));

        this->cameraImage = new QPixmap("/home/tim-linux/Pictures/file_example_JPG_100kB.jpg");
        this->cameraImageLabel->setPixmap(
                cameraImage->scaled(cameraImageLabel->width(), cameraImageLabel->height(), Qt::KeepAspectRatio));
    }

    void initializationSliderCameraLight(int sizeOfSlider) {
        int positionGeneral = 100;
        //lights
        this->lightSlider = new QSlider(Qt::Horizontal, this);
        this->lightSlider->setFocusPolicy(Qt::StrongFocus);
        this->lightSlider->setTickInterval(1);
        this->lightSlider->setMaximum(10);
        this->lightSlider->setMinimum(0);
        this->lightSlider->setGeometry(QRect(QPoint(50, positionGeneral), QSize(sizeOfSlider, 20)));
        this->lightLabel = new QLabel("Light Intensity:", this);
        this->lightLabel->setGeometry(QRect(QPoint(50, positionGeneral - 30), QSize(100, 20)));
        this->currentLightIntensity = new QLabel("0", this);
        this->currentLightIntensity->setGeometry(QRect(QPoint(50 + 154, positionGeneral - 30), QSize(100, 20)));
        this->lightTicks = new QLabel(
                "0                                           5                                   10", this);
        this->lightTicks->setGeometry(QRect(QPoint(50 - 3, positionGeneral + 30), QSize(sizeOfSlider, 15)));
        connect(this->lightSlider, &QSlider::valueChanged, this, &MainWindow::handleLightSlider);
        connect(this->lightSlider, &QSlider::sliderReleased, this, &MainWindow::handleLightSliderReleased);
        //camera angle
        positionGeneral = positionGeneral + 100;
        this->cameraAngleSlider = new QSlider(Qt::Horizontal, this);
        this->cameraAngleSlider->setFocusPolicy(Qt::StrongFocus);
        this->cameraAngleSlider->setTickInterval(10);
        this->cameraAngleSlider->setMaximum(180);
        this->cameraAngleSlider->setMinimum(0);
        this->cameraAngleSlider->setGeometry(QRect(QPoint(50, positionGeneral), QSize(sizeOfSlider, 20)));
        this->cameraAngleLabel = new QLabel("camera Angle:", this);
        this->cameraAngleLabel->setGeometry(QRect(QPoint(50, positionGeneral - 30), QSize(100, 20)));
        this->currentCameraAngle = new QLabel("0", this);
        this->currentCameraAngle->setGeometry(QRect(QPoint(50 + 151, positionGeneral - 30), QSize(100, 20)));
        this->cameraAngleTicks = new QLabel(
                "0                                          90                                180", this);
        this->cameraAngleTicks->setGeometry(QRect(QPoint(50 - 3, positionGeneral + 30), QSize(sizeOfSlider, 15)));
        connect(this->cameraAngleSlider, &QSlider::valueChanged, this, &MainWindow::handleCameraAngleSlider);
        connect(this->cameraAngleSlider, &QSlider::sliderReleased, this, &MainWindow::handleCameraAngleSliderReleased);
    }

    void initializationGamepad(int screenWidth) {
        auto gamepads = QGamepadManager::instance()->connectedGamepads();
        if (gamepads.isEmpty()) {
            qDebug() << "Did not find any connected gamepads";
            this->connectedGamepad = false;
        }else{
            this->connectedGamepad = true;
        }

        this->m_gamepad = new QGamepad(*gamepads.begin(), this);
//        connect(this->m_gamepad, &QGamepad::axisLeftXChanged, this, &MainWindow::updateLeftX);
//        connect(this->m_gamepad, &QGamepad::axisLeftYChanged, this, &MainWindow::updateLeftY);
//        connect(this->m_gamepad, &QGamepad::axisRightXChanged, this, &MainWindow::updateRightX);
//        connect(this->m_gamepad, &QGamepad::axisRightYChanged, this, &MainWindow::updateRightY);
//        connect(this->m_gamepad, &QGamepad::buttonAChanged, this, &MainWindow::updateXButton);
//        connect(this->m_gamepad, &QGamepad::buttonBChanged, this, &MainWindow::updateCircleButton);
//        connect(this->m_gamepad, &QGamepad::buttonXChanged, this, &MainWindow::updateTriangleButton);
//        connect(this->m_gamepad, &QGamepad::buttonYChanged, this, &MainWindow::updateSquareButton);
//        connect(this->m_gamepad, &QGamepad::buttonL1Changed, this, &MainWindow::updateL1Button);
//        connect(this->m_gamepad, &QGamepad::buttonR1Changed, this, &MainWindow::updateR1Button);
//        connect(this->m_gamepad, &QGamepad::buttonL2Changed, this, &MainWindow::updateL2Button);
//        connect(this->m_gamepad, &QGamepad::buttonR2Changed, this, &MainWindow::updateR2Button);
        //currently not used:
//        connect(this->m_gamepad, &QGamepad::buttonSelectChanged, this, );
//        connect(this->m_gamepad, &QGamepad::buttonStartChanged, this, );
//        connect(this->m_gamepad, &QGamepad::buttonGuideChanged, this, );
        int xPositionOfLabels = 200;
        int yPositionOfLabels = 350;
        currentXThrustLabel = new QLabel("Thrust X: ", this);
        currentXThrustLabel->setGeometry(QRect(QPoint(xPositionOfLabels - 150, yPositionOfLabels), QSize(100, 50)));

        currentYThrustLabel = new QLabel("Thrust Y: ", this);
        currentYThrustLabel->setGeometry(QRect(QPoint(xPositionOfLabels, yPositionOfLabels), QSize(100, 50)));

        currentHeightDesiredLabel = new QLabel("Height : ", this);
        currentHeightDesiredLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels + 150, yPositionOfLabels), QSize(100, 50)));

        currentDesiredRollLabel = new QLabel("Roll: ", this);
        currentDesiredRollLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels - 150, yPositionOfLabels + 50), QSize(100, 50)));

        currentDesiredPitchLabel = new QLabel("Pitch: ", this);
        currentDesiredPitchLabel->setGeometry(QRect(QPoint(xPositionOfLabels, yPositionOfLabels + 50), QSize(100, 50)));

        currentDesiredYawLabel = new QLabel("Yaw: ", this);
        currentDesiredYawLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels + 150, yPositionOfLabels + 50), QSize(100, 50)));


    }

    static QVector<double> keepEveryNthElementWithAverage(std::vector<double> array, int nthElement){
        QVector<double> output;
        double av = 0;

//        if(nthElement>1){
//            std::cout << "Starting For Loop: "<< nthElement << std::endl;
//        }
        int howOften=1;
        for (int i = 0; i < array.size(); i++)
        {
            av += array[i];

            if (i % nthElement == 0)
            {

                output.append(av / ((double) howOften));
                av = 0; // reset sum for next average
                howOften = 0;
            }
            howOften++;
        }
        return output;
    }

};

#endif //BLUEROV2COMMON_MAINWINDOW_H
