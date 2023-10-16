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
        this->sonarRange = 2;
        this->sonarStepSize = 1;
        this->frequencyRangeValue = 750;
        this->numberOfSamplesValue = 1200;
        this->strengthXYMovement = 0.5;

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
        this->initializationSliderXYStrength(screenWidth, sizeOfSlider);


    }

public:
    void handleSonarSlider(int sonarRange);

    void handleSonarSliderReleased();

    void handleSonarStepSlider(int sonarRange);

    void handleSonarStepReleased();

    void handleFrequencyRangeSlider(int sonarRange);

    void handleFrequencyRangeReleased();

    void handleNumberOfSamplesSlider(int sonarRange);

    void handleNumberOfSamplesReleased();


    void handleEKFReset();

    void handleHoldPosition();

    void handleControlWithController();

    void handleLightSlider(int lightIntensity);

    void handleLightSliderReleased();

    void handleCameraAngleSlider(int cameraAngle);

    void handleCameraAngleSliderReleased();

    void handleStrengthXYMovementSlider(int strength);

    //Gamepad Handling Functions
    void handleHeight(double changeOfHeight);

    void handleRoll(double changeOfRoll);

    void handlePitch(double changeOfPitch);

    void handleYaw(double changeOfYaw);

    void handleLocalXMovement(double changeOfX);

    void handleLocalYMovement(double changeOfY);

    void changeHoldPositionStatus(bool holdPosition);

    void handleResetEKFGraph();

    void threadSendCurrentDesiredPoseRobot() {
        rclcpp::Rate loop_rate(30);
//        ros::Rate loop_rate(30);

        while (rclcpp::ok()) {

//            this->updateRightX(this->m_gamepad->axisRightX());
//            this->updateRightY(this->m_gamepad->axisRightY());
//            this->updateLeftX(this->m_gamepad->axisLeftX());
//            this->updateXButton(this->m_gamepad->buttonA());
//            this->updateSquareButton(this->m_gamepad->buttonY());
//            this->updateCircleButton(this->m_gamepad->buttonB());
//            this->updateTriangleButton(this->m_gamepad->buttonX());
//            this->updateR1Button(this->m_gamepad->buttonR1());
//            this->updateR2Button(this->m_gamepad->buttonR2());


            //std::cout << "Sending data from Gui To ROS: " << test<< std::endl;


            emit this->updateDesiredState(this->desiredHeight, this->desiredRoll, this->desiredPitch, this->desiredYaw,
                                          this->desiredXMovement, this->desiredYMovement, this->holdPositionStatus);
//            rclcpp::spin();
//            rclcpp::spin_some();
            loop_rate.sleep();
        }


    }

    void threadUpdateCurrentDesiredPoseRobot() {
        rclcpp::Rate loop_rate(10);

//        ros::Rate loop_rate(10);

        while (rclcpp::ok()) {

            this->updateRightX(this->m_gamepad->axisRightX());
            this->updateRightY(this->m_gamepad->axisRightY());
            this->updateLeftX(this->m_gamepad->axisLeftX());
            this->updateXButton(this->m_gamepad->buttonA());
            this->updateSquareButton(this->m_gamepad->buttonY());
            this->updateCircleButton(this->m_gamepad->buttonB());
            this->updateTriangleButton(this->m_gamepad->buttonX());
            this->updateR1Button(this->m_gamepad->buttonR1());
            this->updateR2Button(this->m_gamepad->buttonR2());

//            ros::spinOnce();
            loop_rate.sleep();
        }


    }

public slots:

    void
    updateStateForPlotting(std::vector<double> xPositionRobot, std::vector<double> yPositionRobot,
                           std::vector<double> yawPositionRobot);

    void updateStateOfRobot(double xPos, double yPos, double zPos, double roll, double pitch, double yaw,
                            Eigen::MatrixXd covariance);//covariance is just 6 values
    void updateCameraImage(QPixmap cameraImage);

    void updateSonarImage(QPixmap sonarImage);

    void updateDVLState(double distance1, double distance2, double distance3, double distance4);

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

    void updateDesiredState(double desiredHeight, double desiredRoll, double desiredPitch, double desiredYaw,
                            double desiredXMovement, double desiredYMovement, bool holdPosition);

    void resetEKFEstimator(bool resetOnlyGraph);

    void updateConfigSonar(int stepSize, int rangeSonar,int frequencyRange,int numberOfSamples);

    void updateAngleCamera(int angleCamera);

    void updateLightIntensity(int intensityLight);

private:
    QLabel *distanceToBottom, *depth, *plotOfPosition, *sonarLabel, *sonarTicks, *movementStrengthLabel;
    QLabel *currentSonarRange, *sonarStepSizeLabel, *currentSonarStepSize, *sonarImageLabel, *currentStrengthXYMovement;
    QLabel *lightLabel, *cameraImageLabel, *lightTicks, *currentLightIntensity, *cameraAngleLabel;
    QLabel *currentCameraAngle, *cameraAngleTicks;
    QLabel *currentXThrustLabel, *currentYThrustLabel, *currentHeightDesiredLabel, *currentDesiredRollLabel, *currentDesiredPitchLabel, *currentDesiredYawLabel, *desiredPositionLabel;
    QLabel *currentPositionZLabel, *currentPositionLabel, *currentPositionYawLabel, *currentPositionXLabel, *currentPositionRollLabel, *currentPositionPitchLabel, *currentPositionYLabel;
    QLabel *currentDistanceToBottom, *currentDistanceDVL1, *currentDistanceDVL2, *currentDistanceDVL3, *currentDistanceDVL4;
    QLabel *numberOfSamplesLabel,*numberOfSamplesTicks,*currentNumberOfSamples;
    QLabel *frequencyRangeLabel,*frequencyRangeTicks,*currentfrequencyRange;
    QPushButton *resetEKF, *holdPos, *resetGraphEKF;
    QSlider *rangeSonarSlider, *angularStepSizeSlider, *lightSlider, *cameraAngleSlider, *strengthXYMovementSlider,*numberOfSamplesSlider,*frequencyRangeSlider;
    int sonarRange, sonarStepSize, lightIntensity, cameraAngleDesired,frequencyRangeValue,numberOfSamplesValue;
    QCustomPlot *customPlot;
    QVector<double> xPositionRobot, yPositionRobot, yawPositionRobot;
    QPixmap *sonarImage, *cameraImage;
    QGamepad *m_gamepad;
    bool connectedGamepad;
    std::atomic<double> desiredHeight, desiredRoll, desiredPitch, desiredYaw, desiredXMovement, desiredYMovement;

    std::atomic<bool> holdPositionStatus;

    //current state Robot:
    std::atomic<double> currentHeight, currentRoll, currentPitch, currentYaw, currentXPos, currentYPos;

    std::atomic<double> strengthXYMovement;
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
        this->rangeSonarSlider->setSliderPosition(30);

        //angular Step size
        yposRangeSonar = yposRangeSonar + 100;
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
        this->angularStepSizeSlider->setSliderPosition(5);



        // set number of samples for each sonar ray
        yposRangeSonar = yposRangeSonar + 100;
        numberOfSamplesSlider = new QSlider(Qt::Horizontal, this);
        numberOfSamplesSlider->setFocusPolicy(Qt::StrongFocus);
        numberOfSamplesSlider->setMaximum(2000);
        numberOfSamplesSlider->setMinimum(200);
        numberOfSamplesSlider->setGeometry(QRect(QPoint(xposRangeSonar, yposRangeSonar), QSize(sizeOfSlider, 20)));
        numberOfSamplesLabel = new QLabel("Number Of Samples:", this);
        numberOfSamplesLabel->setGeometry(
                QRect(QPoint(screenWidth - 1.3 * sizeOfSlider, yposRangeSonar - 50), QSize(200, 50)));
        numberOfSamplesTicks = new QLabel("200                         1000                                  2000",
                                this);
        numberOfSamplesTicks->setGeometry(QRect(QPoint(xposRangeSonar - 3, yposRangeSonar + 25), QSize(sizeOfSlider, 15)));
        currentNumberOfSamples = new QLabel("1000", this);
        currentNumberOfSamples->setGeometry(
                QRect(QPoint(screenWidth - 0.8 * sizeOfSlider, yposRangeSonar - 50), QSize(200, 50)));
        connect(numberOfSamplesSlider, &QSlider::valueChanged, this, &MainWindow::handleNumberOfSamplesSlider);
        connect(numberOfSamplesSlider, &QSlider::sliderReleased, this, &MainWindow::handleNumberOfSamplesReleased);
        this->numberOfSamplesSlider->setSliderPosition(1000);

        // set transmit frequency
        yposRangeSonar = yposRangeSonar + 100;
        frequencyRangeSlider = new QSlider(Qt::Horizontal, this);
        frequencyRangeSlider->setFocusPolicy(Qt::StrongFocus);
        frequencyRangeSlider->setMaximum(1000);
        frequencyRangeSlider->setMinimum(500);
        frequencyRangeSlider->setGeometry(QRect(QPoint(xposRangeSonar, yposRangeSonar), QSize(sizeOfSlider, 20)));
        frequencyRangeLabel = new QLabel("Frequency Range:", this);
        frequencyRangeLabel->setGeometry(
                QRect(QPoint(screenWidth - 1.3 * sizeOfSlider, yposRangeSonar - 50), QSize(200, 50)));
        frequencyRangeTicks = new QLabel("500                              750                              1000",
                                this);
        frequencyRangeTicks->setGeometry(QRect(QPoint(xposRangeSonar - 3, yposRangeSonar + 25), QSize(sizeOfSlider, 15)));
        currentfrequencyRange = new QLabel("750", this);
        currentfrequencyRange->setGeometry(
                QRect(QPoint(screenWidth - 0.8 * sizeOfSlider, yposRangeSonar - 50), QSize(200, 50)));
        connect(frequencyRangeSlider, &QSlider::valueChanged, this, &MainWindow::handleFrequencyRangeSlider);
        connect(frequencyRangeSlider, &QSlider::sliderReleased, this, &MainWindow::handleFrequencyRangeReleased);
        this->frequencyRangeSlider->setSliderPosition(750);
        //

        //send initial values to sonar
//        this->handleSonarSliderReleased();
    }

    void initializationSliderXYStrength(int screenWidth, int sizeOfSlider) {
        //strength Slider
        this->strengthXYMovementSlider = new QSlider(Qt::Horizontal, this);

        this->strengthXYMovementSlider->setFocusPolicy(Qt::StrongFocus);
        //rangeSonarSlider->setTickPosition(QSlider::TicksBelow);
        this->strengthXYMovementSlider->setTickInterval(1);
        this->strengthXYMovementSlider->setMaximum(10);
        this->strengthXYMovementSlider->setMinimum(1);
        this->strengthXYMovementSlider->setGeometry(
                QRect(QPoint(screenWidth / 2 - sizeOfSlider, 70), QSize(sizeOfSlider, 20)));
        movementStrengthLabel = new QLabel("Strength Of XY Movement:", this);
        movementStrengthLabel->setGeometry(QRect(QPoint(screenWidth / 2 - 1.3 * sizeOfSlider, 10), QSize(200, 50)));
        currentStrengthXYMovement = new QLabel("0", this);
        currentStrengthXYMovement->setGeometry(
                QRect(QPoint(screenWidth / 2 - 0.5 * sizeOfSlider, 10), QSize(200, 50)));
        connect(this->strengthXYMovementSlider, &QSlider::valueChanged, this,
                &MainWindow::handleStrengthXYMovementSlider);
//        connect(this->strengthXYMovementSlider, &QSlider::sliderReleased, this, &MainWindow::handleSonarSliderReleased);
        this->strengthXYMovementSlider->setSliderPosition(5);
    }


    void initializationCurrentPosition(int screenWidth) {
        int sizePlot = 400;
        int distanceFromLeftCorner = 50;
        int sizeButtons = 120;
        resetEKF = new QPushButton("Reset EKF", this);
        // set size and location of the button
        resetEKF->setGeometry(QRect(QPoint(distanceFromLeftCorner, 500), QSize(sizeButtons, 40)));

        //disabling EKF reset. Could also be solved by supressing the send mavros vision position for some time. Would be done in the EKF system
//        connect(resetEKF, &QPushButton::released, this, &MainWindow::handleEKFReset);
        this->holdPos = new QPushButton("hold Pos", this);
        // set size and location of the button
        this->holdPos->setGeometry(
                QRect(QPoint(distanceFromLeftCorner + sizePlot - sizeButtons, 500), QSize(sizeButtons, 40)));
        connect(this->holdPos, &QPushButton::released, this, &MainWindow::handleHoldPosition);

        resetGraphEKF = new QPushButton("Reset Graph", this);
//         set size and location of the button
        resetGraphEKF->setGeometry(
                QRect(QPoint(distanceFromLeftCorner + sizePlot / 2 - sizeButtons / 2, 500), QSize(sizeButtons, 40)));
        connect(resetGraphEKF, &QPushButton::released, this, &MainWindow::handleResetEKFGraph);


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
        this->lightSlider->setSliderPosition(0);
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
        this->cameraAngleSlider->setSliderPosition(90);

        //send initial values to robot
//        this->handleCameraAngleSliderReleased();
//        this->handleLightSliderReleased();
    }

    void initializationGamepad(int screenWidth) {
        auto gamepads = QGamepadManager::instance()->connectedGamepads();
        if (gamepads.isEmpty()) {
            qDebug() << "Did not find any connected gamepads";
            this->connectedGamepad = false;
        } else {
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
        int yPositionOfLabels = 300;
        desiredPositionLabel = new QLabel("Desired State: ", this);
        desiredPositionLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels - 150, yPositionOfLabels - 40), QSize(200, 45)));

        currentXThrustLabel = new QLabel("Thrust X: ", this);
        currentXThrustLabel->setGeometry(QRect(QPoint(xPositionOfLabels - 150, yPositionOfLabels - 5), QSize(100, 45)));

        currentYThrustLabel = new QLabel("Thrust Y: ", this);
        currentYThrustLabel->setGeometry(QRect(QPoint(xPositionOfLabels, yPositionOfLabels - 5), QSize(100, 45)));

        currentHeightDesiredLabel = new QLabel("Height: 0.00", this);
        currentHeightDesiredLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels + 150, yPositionOfLabels - 5), QSize(100, 45)));

        currentDesiredRollLabel = new QLabel("Roll: 0.00", this);
        currentDesiredRollLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels - 150, yPositionOfLabels + 30), QSize(100, 45)));

        currentDesiredPitchLabel = new QLabel("Pitch: 0.00", this);
        currentDesiredPitchLabel->setGeometry(QRect(QPoint(xPositionOfLabels, yPositionOfLabels + 30), QSize(100, 45)));

        currentDesiredYawLabel = new QLabel("Yaw: 0.00", this);
        currentDesiredYawLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels + 150, yPositionOfLabels + 30), QSize(100, 45)));

        currentPositionLabel = new QLabel("Position : ", this);
        currentPositionLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels - 150, yPositionOfLabels + 85), QSize(150, 45)));

        currentPositionXLabel = new QLabel("X: ", this);
        currentPositionXLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels - 150, yPositionOfLabels + 115), QSize(150, 45)));

        currentPositionYLabel = new QLabel("Y: ", this);
        currentPositionYLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels, yPositionOfLabels + 115), QSize(150, 45)));

        currentPositionZLabel = new QLabel("Height: ", this);
        currentPositionZLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels + 150, yPositionOfLabels + 115), QSize(150, 45)));

        currentPositionRollLabel = new QLabel("Roll: ", this);
        currentPositionRollLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels - 150, yPositionOfLabels + 150), QSize(150, 45)));

        currentPositionPitchLabel = new QLabel("Pitch: ", this);
        currentPositionPitchLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels, yPositionOfLabels + 150), QSize(150, 45)));

        currentPositionYawLabel = new QLabel("Yaw: ", this);
        currentPositionYawLabel->setGeometry(
                QRect(QPoint(xPositionOfLabels + 150, yPositionOfLabels + 150), QSize(150, 45)));

        currentDistanceToBottom = new QLabel("DVL distances: ", this);
        currentDistanceToBottom->setGeometry(
                QRect(QPoint(screenWidth / 2 - screenWidth / 5 -screenWidth/25, yPositionOfLabels - 40), QSize(200, 45)));

        currentDistanceDVL1 = new QLabel("Distance 1: ", this);
        currentDistanceDVL1->setGeometry(
                QRect(QPoint(screenWidth / 2 - screenWidth / 5 -screenWidth/25, yPositionOfLabels ), QSize(200, 45)));
        currentDistanceDVL2 = new QLabel("Distance 2: ", this);
        currentDistanceDVL2->setGeometry(
                QRect(QPoint(screenWidth / 2 - screenWidth / 5 -screenWidth/25 + screenWidth/12, yPositionOfLabels ), QSize(200, 45)));
        currentDistanceDVL3 = new QLabel("Distance 3: ", this);
        currentDistanceDVL3->setGeometry(
                QRect(QPoint(screenWidth / 2 - screenWidth / 5 -screenWidth/25, yPositionOfLabels +40), QSize(200, 45)));
        currentDistanceDVL4 = new QLabel("Distance 4: ", this);
        currentDistanceDVL4->setGeometry(
                QRect(QPoint(screenWidth / 2  - screenWidth / 5 -screenWidth/25 + screenWidth/12, yPositionOfLabels +40), QSize(200, 45)));


    }

    static QVector<double>
    keepEveryNthElementWithAverage(std::vector<double> array, int nthElement, int holdLastPositions) {
        QVector<double> output;
        double av = 0;

//        if(nthElement>1){
//            std::cout << "Starting For Loop: "<< nthElement << std::endl;
//        }
        int howOften = 1;
        for (int i = 0; i < array.size() - holdLastPositions; i++) {
            av += array[i];

            if (i % nthElement == 0) {

                output.append(av / ((double) howOften));
                av = 0; // reset sum for next average
                howOften = 0;
            }
            howOften++;
        }
        for (int i = array.size() - holdLastPositions; i < array.size(); i++) {
            output.append(array[i]);
        }

        return output;
    }

public:
    void updateDesiredPosition(){
        rclcpp::sleep_for(std::chrono::seconds(200000000));// should be 0.2 seconds
//        rclcpp::Duration(0.2).sleep();
        double tmpNumber;
        tmpNumber = this->currentYaw;
        this->desiredYaw = tmpNumber;

        tmpNumber = this->currentHeight;
        this->desiredHeight = tmpNumber;
        QString xstr = "Height: " + QString::number(this->desiredHeight, 'f', 2);
        this->currentHeightDesiredLabel->setText(xstr);

    }

};

#endif //BLUEROV2COMMON_MAINWINDOW_H
