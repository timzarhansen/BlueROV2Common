//
// Created by tim-linux on 14.12.21.
//
#include "mainwindow.h"


void MainWindow::updateStateForPlotting(QVector<double> xPositionRobot, QVector<double> yPositionRobot,
                                        QVector<double> yawPositionRobot) {
    this->xPositionRobot = xPositionRobot;
    this->yPositionRobot = yPositionRobot;
    this->yawPositionRobot = yawPositionRobot;
    double xMin = *std::min_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
    double xMax = *std::max_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
    double yMin = *std::min_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
    double yMax = *std::max_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
    this->customPlot->xAxis->setRange(xMin - 1, xMax + 1);
    this->customPlot->yAxis->setRange(yMin - 1, yMax + 1);
    //std::cout << this->xPositionRobot.size() << std::endl;
    //this->customPlot->clearGraphs();
    this->customPlot->graph(0)->setData(this->xPositionRobot, this->yPositionRobot);
    this->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
    this->customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 2));
    this->customPlot->replot();

}

void MainWindow::updateSonarImage(QPixmap sonarImage) {
    this->sonarImageLabel->setPixmap(
            sonarImage.scaled(sonarImageLabel->width(), sonarImageLabel->height(), Qt::KeepAspectRatio));
}

void MainWindow::updateCameraImage(QPixmap cameraImage) {
    this->cameraImageLabel->setPixmap(
            cameraImage.scaled(sonarImageLabel->width(), sonarImageLabel->height(), Qt::KeepAspectRatio));
}

void MainWindow::handleSonarSlider(int sonarRange) {

    QString xstr = QString::number(sonarRange);
    this->sonarRange = sonarRange;
    this->currentSonarRange->setText(xstr);

}

void MainWindow::handleSonarSliderReleased() {
    //send current distance to the Sonar
    std::cout << "sonarRange send to Sonar: " << this->sonarRange << std::endl;
}

void MainWindow::handleSonarStepSlider(int angularStepSize) {
    QString xstr = QString::number(angularStepSize);
    this->sonarStepSize = angularStepSize;
    this->currentSonarStepSize->setText(xstr);

}

void MainWindow::handleSonarStepReleased() {
    //send current distance to the Sonar
    std::cout << "sonarStepSize send to Sonar: " << this->sonarStepSize << std::endl;

}

void MainWindow::handleEKFReset() {
    std::cout << "send reset to EKF" << std::endl;
}

void MainWindow::handleHoldPosition() {
    std::cout << "send hold Position" << std::endl;

}

void MainWindow::handleControlWithController() {
    std::cout << "send control Robot with Controller" << std::endl;
}

void MainWindow::handleLightSlider(int lightIntensity) {
    QString xstr = QString::number(lightIntensity);
    this->lightIntensity = lightIntensity;
    this->currentLightIntensity->setText(xstr);

}

void MainWindow::handleLightSliderReleased() {
    std::cout << "send Light to Robot: " << this->lightIntensity << std::endl;

}

void MainWindow::handleCameraAngleSlider(int cameraAngle) {
    QString xstr = QString::number(cameraAngle);
    this->cameraAngle = cameraAngle;
    this->currentCameraAngle->setText(xstr);
}

void MainWindow::handleCameraAngleSliderReleased() {
    std::cout << "send Camera Angle to Robot: " << this->cameraAngle << std::endl;

}

//move x body axis
void MainWindow::updateRightX(double value) {
    std::cout << "Right X: " << value << std::endl;
    this->desiredXMovement = 0.2 * value;
}

//move y body axis
void MainWindow::updateRightY(double value) {
    std::cout << "Right Y: " << value << std::endl;
    this->desiredYMovement = 0.2 * value;
}

//yaw rotation
void MainWindow::updateLeftX(double value) {
    std::cout << "Left X: " << value << std::endl;

    this->desiredYaw =this->desiredYaw+0.01 * value;
    //make sure to hold yaw in range of +- pi
    if (this->desiredYaw > M_PI) {
        this->desiredYaw = this->desiredYaw-2 * M_PI;
    }
    if (this->desiredYaw < -M_PI) {
        this->desiredYaw = this->desiredYaw+2 * M_PI;
    }
}

void MainWindow::updateLeftY(double value) { std::cout << "Left Y: " << value << std::endl; }//nothing

//roll +
void MainWindow::updateXButton(bool pressed) {
    std::cout << "X button Pressed: " << pressed << std::endl;
    this->desiredRoll = this->desiredRoll+0.01;
    if (this->desiredRoll > M_PI) {
        this->desiredRoll =this->desiredRoll- 2 * M_PI;
    }
}

//pitch +
void MainWindow::updateSquareButton(bool pressed) {
    std::cout << "Square button Pressed: " << pressed << std::endl;
    this->desiredPitch =this->desiredPitch+ 0.01;
    if (this->desiredPitch > M_PI / 2) {
        this->desiredPitch = M_PI / 2;
    }
}

//roll -
void MainWindow::updateCircleButton(bool pressed) {
    std::cout << "Circle button Pressed: " << pressed << std::endl;
    this->desiredRoll = this->desiredRoll-0.01;
    if (this->desiredRoll < -M_PI) {
        this->desiredRoll = this->desiredRoll+2 * M_PI;
    }
}

//pitch -
void MainWindow::updateTriangleButton(bool pressed) {
    std::cout << "Triangle button Pressed: " << pressed << std::endl;
    this->desiredPitch =this->desiredPitch- 0.01;
    if (this->desiredPitch < -M_PI / 2) {
        this->desiredPitch = -M_PI / 2;
    }
}

//height +
void MainWindow::updateR1Button(bool pressed) {
    std::cout << "R1 button Pressed: " << pressed << std::endl;
    if (abs(this->desiredHeight - this->currentHeight) < 1.0f) {
        this->desiredHeight = this->desiredHeight+0.1;
    }
}

//height -
void MainWindow::updateR2Button(bool pressed) {
    std::cout << "R2 button Pressed: " << pressed << std::endl;
    if (abs(this->desiredHeight - this->currentHeight) < 1.0f) {
        this->desiredHeight =this->desiredHeight- 0.1;
    }
}

void MainWindow::updateL1Button(bool pressed) { std::cout << "L1 button Pressed: " << pressed << std::endl; }//nothing
void MainWindow::updateL2Button(bool pressed) { std::cout << "L2 button Pressed: " << pressed << std::endl; }//nothing


void MainWindow::updateStateOfRobot(double xPos, double yPos, double zPos, double roll, double pitch, double yaw,
                                    Eigen::MatrixXd covariance) {
    this->currentHeight = zPos;
    this->currentRoll = roll;
    this->currentPitch = pitch;
    this->currentYaw = yaw;
    this->currentXPos = xPos;
    this->currentYPos = yPos;
}
