//
// Created by tim-linux on 14.12.21.
//
#include "mainwindow.h"

#include <utility>


void MainWindow::updateStateForPlotting(std::vector<double> xPositionRobot, std::vector<double> yPositionRobot,
                                        std::vector<double> yawPositionRobot) {

    if(xPositionRobot.size()>300){

        int everyNthElement= xPositionRobot.size()/300;
        this->xPositionRobot = this->keepEveryNthElementWithAverage(xPositionRobot,everyNthElement,50);
        this->yPositionRobot = this->keepEveryNthElementWithAverage(yPositionRobot,everyNthElement,50);
        this->yawPositionRobot = this->keepEveryNthElementWithAverage(yawPositionRobot,everyNthElement,50);
    }else{
        this->xPositionRobot = this->keepEveryNthElementWithAverage(xPositionRobot,1,0);
        this->yPositionRobot = this->keepEveryNthElementWithAverage(yPositionRobot,1,0);
        this->yawPositionRobot = this->keepEveryNthElementWithAverage(yawPositionRobot,1,0);
    }


    //std::nth_element(this->xPositionRobot.begin(),this->xPositionRobot.begin()+5,this->xPositionRobot.end());
//    std::cout << this->xPositionRobot.size() << std::endl;
    double xMin = *std::min_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
    double xMax = *std::max_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
    double yMin = *std::min_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
    double yMax = *std::max_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
    this->customPlot->xAxis->setRange(yMin - 1, yMax + 1);
    this->customPlot->yAxis->setRange(xMin - 1, xMax + 1);
    //std::cout << this->xPositionRobot.size() << std::endl;
    //this->customPlot->clearGraphs();
    this->customPlot->graph(0)->setData(this->yPositionRobot, this->xPositionRobot);
    this->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
    this->customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 2));
    this->customPlot->xAxis->setLabel("y");
    this->customPlot->yAxis->setLabel("x");
//    std::cout << "test" << std::endl;
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
//    std::cout << "sonarRange send to Sonar: " << this->sonarRange << std::endl;
    emit this->updateConfigSonar(this->sonarStepSize, this->sonarRange);
}

void MainWindow::handleSonarStepSlider(int angularStepSize) {
    QString xstr = QString::number(angularStepSize);
    this->sonarStepSize = angularStepSize;
    this->currentSonarStepSize->setText(xstr);

}

void MainWindow::handleSonarStepReleased() {
    //send current distance to the Sonar
//    std::cout << "sonarStepSize send to Sonar: " << this->sonarStepSize << std::endl;
    emit this->updateConfigSonar(this->sonarStepSize, this->sonarRange);
}

void MainWindow::handleEKFReset() {
    std::cout << "send reset to EKF" << std::endl;
    emit this->resetEKFEstimator(false);

}

void MainWindow::handleResetEKFGraph() {

    emit this->resetEKFEstimator(true);
}

void MainWindow::handleHoldPosition() {
    std::cout << "send hold Position" << std::endl;
    this->holdPositionStatus= not this->holdPositionStatus;

    QPalette pal = this->holdPos->palette();
    if(this->holdPositionStatus){
        pal.setColor(QPalette::Button, QColor(Qt::green));
    }else{
        pal.setColor(QPalette::Button, QColor(Qt::red));

        double tmpNumber = this->currentYaw;
        this->desiredYaw = tmpNumber;
    }

    this->holdPos->setAutoFillBackground(true);
    this->holdPos->setFlat(true);
    this->holdPos->setPalette(pal);
    this->holdPos->update();

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
//    std::cout << "send Light to Robot: " << this->lightIntensity << std::endl;
    emit this->updateLightIntensity(this->lightIntensity);
}

void MainWindow::handleCameraAngleSlider(int cameraAngle) {
    QString xstr = QString::number(cameraAngle);
    this->cameraAngleDesired = cameraAngle;
    this->currentCameraAngle->setText(xstr);
}

void MainWindow::handleCameraAngleSliderReleased() {
//    std::cout << "send Camera Angle to Robot: " << this->cameraAngleDesired << std::endl;
    emit this->updateAngleCamera(this->cameraAngleDesired);
}

//move x body axis
void MainWindow::updateRightX(double value) {
//    std::cout << "Right X: " << value << std::endl;
    this->desiredYMovement = 0.5 * value;
    QString xstr = "Thrust Y: " + QString::number(this->desiredYMovement, 'f', 2);
    this->currentYThrustLabel->setText(xstr);
}

//move y body axis
void MainWindow::updateRightY(double value) {
//    std::cout << "Right Y: " << value << std::endl;
    this->desiredXMovement = -0.5 * value;
    QString xstr = "Thrust X: " + QString::number(this->desiredXMovement, 'f', 2);
    this->currentXThrustLabel->setText(xstr);
}

//yaw rotation
void MainWindow::updateLeftX(double value) {
//    std::cout << "Left X: " << value << std::endl;
    if(abs(value)>0.05){
        this->desiredYaw = this->desiredYaw + 0.01 * value;
    }
    //make sure to hold yaw in range of +- pi
    if (this->desiredYaw > M_PI) {
        this->desiredYaw = this->desiredYaw - 2 * M_PI;
    }
    if (this->desiredYaw < -M_PI) {
        this->desiredYaw = this->desiredYaw + 2 * M_PI;
    }
    QString xstr = "Yaw: " + QString::number(this->desiredYaw * 180 / M_PI, 'f', 2);
    this->currentDesiredYawLabel->setText(xstr);
}

void MainWindow::updateLeftY(double value) { std::cout << "Left Y: " << value << std::endl; }//nothing

//roll +
void MainWindow::updateXButton(bool pressed) {
//    std::cout << "X button Pressed: " << pressed << std::endl;
    if (pressed) {
        this->desiredRoll = this->desiredRoll + 0.01;
        if (this->desiredRoll > M_PI) {
            this->desiredRoll = this->desiredRoll - 2 * M_PI;
        }

        QString xstr = "Roll: " + QString::number(this->desiredRoll * 180 / M_PI, 'f', 2);
        this->currentDesiredRollLabel->setText(xstr);
    }

}

//pitch +
void MainWindow::updateSquareButton(bool pressed) {
//    std::cout << "Square button Pressed: " << pressed << std::endl;
    if (pressed) {
        this->desiredPitch = this->desiredPitch + 0.01;

        if (this->desiredPitch > M_PI / 2) {
            this->desiredPitch = M_PI / 2;
        }
        QString xstr = "Pitch: " + QString::number(this->desiredPitch * 180 / M_PI, 'f', 2);
        this->currentDesiredPitchLabel->setText(xstr);
    }
}

//roll -
void MainWindow::updateCircleButton(bool pressed) {
//    std::cout << "Circle button Pressed: " << pressed << std::endl;
    if (pressed) {
        this->desiredRoll = this->desiredRoll - 0.01;

        if (this->desiredRoll < -M_PI) {
            this->desiredRoll = this->desiredRoll + 2 * M_PI;
        }
        QString xstr = "Roll: " + QString::number(this->desiredRoll * 180 / M_PI, 'f', 2);
        this->currentDesiredRollLabel->setText(xstr);
    }
}

//pitch -
void MainWindow::updateTriangleButton(bool pressed) {
//    std::cout << "Triangle button Pressed: " << pressed << std::endl;
    if (pressed) {
        this->desiredPitch = this->desiredPitch - 0.01;

        if (this->desiredPitch < -M_PI / 2) {
            this->desiredPitch = -M_PI / 2;
        }
        QString xstr = "Pitch: " + QString::number(this->desiredPitch * 180 / M_PI, 'f', 2);
        this->currentDesiredPitchLabel->setText(xstr);
    }
}

//height +
void MainWindow::updateR1Button(bool pressed) {
//    std::cout << "R1 button Pressed: " << pressed << std::endl;

    if (pressed) {
        if (abs(this->desiredHeight - 0.02 - this->currentHeight) < 1.0f) {
            this->desiredHeight = this->desiredHeight - 0.02;
        }
        QString xstr = "Height: " + QString::number(this->desiredHeight, 'f', 2);
        this->currentHeightDesiredLabel->setText(xstr);
    }
}

//height -
void MainWindow::updateR2Button(double pressedValue) {
//    std::cout << "R2 button Pressed: " << pressedValue << std::endl;
    if (pressedValue>0.2) {
        if (abs(this->desiredHeight + 0.02 - this->currentHeight) < 1.0f) {
            this->desiredHeight = this->desiredHeight + 0.02;
        }
        QString xstr = "Height: " + QString::number(this->desiredHeight, 'f', 2);
        this->currentHeightDesiredLabel->setText(xstr);
    }
}

void MainWindow::updateL1Button(bool pressed) { std::cout << "L1 button Pressed: " << pressed << std::endl; }//nothing
void MainWindow::updateL2Button(bool pressed) { std::cout << "L2 button Pressed: " << pressed << std::endl; }//nothing


void MainWindow::updateStateOfRobot(double xPos, double yPos, double zPos, double roll, double pitch, double yaw,
                                    Eigen::MatrixXd covariance) {
    this->currentHeight = zPos;
    QString xstr = "Height: " + QString::number(this->currentHeight, 'f', 2);
    this->currentPositionZLabel->setText(xstr);

    this->currentRoll = roll;
    this->currentPitch = pitch;
    this->currentYaw = yaw;
    xstr = "Yaw: " + QString::number(this->currentYaw*180/M_PI, 'f', 2);
    this->currentPositionYawLabel->setText(xstr);
    this->currentXPos = xPos;
    this->currentYPos = yPos;
}
