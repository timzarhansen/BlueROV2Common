//
// Created by tim-linux on 14.12.21.
//
#include "mainwindow.h"


void MainWindow::updatePositions(QVector<double> xPositionRobot, QVector<double> yPositionRobot,
                                 QVector<double> yawPositionRobot) {
    this->xPositionRobot = xPositionRobot;
    this->yPositionRobot = yPositionRobot;
    this->yawPositionRobot = yawPositionRobot;
    double xMin = *std::min_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
    double xMax = *std::max_element(this->xPositionRobot.constBegin(), this->xPositionRobot.constEnd());
    double yMin = *std::min_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
    double yMax = *std::max_element(this->yPositionRobot.constBegin(), this->yPositionRobot.constEnd());
    this->customPlot->xAxis->setRange(xMin-1, xMax+1);
    this->customPlot->yAxis->setRange(yMin-1, yMax+1);
    //std::cout << this->xPositionRobot.size() << std::endl;
    //this->customPlot->clearGraphs();
    this->customPlot->graph(0)->setData(this->xPositionRobot, this->yPositionRobot);
    this->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
    this->customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 2));
    this->customPlot->replot();

}

void MainWindow::updateSonarImage(QPixmap sonarImage){
    this->sonarImageLabel->setPixmap(sonarImage.scaled(sonarImageLabel->width(), sonarImageLabel->height(), Qt::KeepAspectRatio));
}
void MainWindow::updateCameraImage(QPixmap cameraImage){
    this->cameraImageLabel->setPixmap(cameraImage.scaled(sonarImageLabel->width(), sonarImageLabel->height(), Qt::KeepAspectRatio));
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

void MainWindow::handleHoldPosition(){
    std::cout << "send hold Position" << std::endl;

}

void MainWindow::handleControlWithController(){
    std::cout << "send control Robot with Controller"<< std::endl;
}

void MainWindow::handleLightSlider(int lightIntensity){
    QString xstr = QString::number(lightIntensity);
    this->lightIntensity = lightIntensity;
    this->currentLightIntensity->setText(xstr);

}
void MainWindow::handleLightSliderReleased(){
    std::cout << "send Light to Robot: "<< this->lightIntensity << std::endl;

}

void MainWindow::handleCameraAngleSlider(int cameraAngle){
    QString xstr = QString::number(cameraAngle);
    this->cameraAngle = cameraAngle;
    this->currentCameraAngle->setText(xstr);
}

void MainWindow::handleCameraAngleSliderReleased(){
    std::cout << "send Camera Angle to Robot: "<< this->cameraAngle <<  std::endl;

}