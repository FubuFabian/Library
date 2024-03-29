/* 
 * File:   ProbeCalibrationWidget.cpp
 * Author: Zian Fanti
 * 
 * Created on 23 de diciembre de 2011, 0:38
 */

#include "ProbeCalibrationWidget.h"
#include "SinglePointUltrasoundCalibration.h"

#include <QErrorMessage>
#include <QString>
#include <QFile>
#include <QTextStream>

#include <vtkExtractVOI.h>

#include <vnl/vnl_quaternion.h>
#include <vnl/vnl_vector_fixed.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vnl/vnl_double_2.h>

using namespace std;

ProbeCalibrationWidget::ProbeCalibrationWidget(QWidget* parent) : QWidget(parent)
{
    this->setupUi(this);
    
    QStringList header;
    header << "Frame" << "X" << "Y";
    tableWidget->setHorizontalHeaderLabels(header);
}


ProbeCalibrationWidget::~ProbeCalibrationWidget()
{
    this->image = NULL;
    delete this;
}


void ProbeCalibrationWidget::crop()
{
    int depth = 0;
    vtkSmartPointer<vtkImageData> cropImage;
    std::vector< vtkSmartPointer<vtkImageData> > cropStack;
    
	std::cout<<std::endl;
    if (this->radioButton_4->isChecked())
    {
        depth = 4;
        if (workWithStack)
        {
            cropStack.reserve(imageStack.size());
            for (uint i = 0; i < imageStack.size(); i++)
            {
				std::cout<<"Cropping image"<<i+1<<std::endl;
                cropStack.push_back(this->cropProbeImage(imageStack.at(i), depth));
            }
        }
        else
        {
            cropImage = this->cropProbeImage(this->image, depth);
        }
    }
    else if (this->radioButton_5->isChecked())
    {
        depth = 5;
        if (workWithStack)
			for (uint i = 0; i < imageStack.size(); i++){
				std::cout<<"Cropping image"<<i+1<<std::endl;
                imageStack.at(i) = this->cropProbeImage(imageStack.at(i), depth);
			}
        else
            cropImage = this->cropProbeImage(this->image, depth);
    }
    else if (this->radioButton_6->isChecked())
    {
        depth = 6;
        if (workWithStack)
			for (uint i = 0; i < imageStack.size(); i++){
				std::cout<<"Cropping image"<<i+1<<std::endl;
                this->cropProbeImage(imageStack.at(i), depth);
			}
        else
            cropImage = this->cropProbeImage(this->image, depth);
    }
    else if (this->radioButton_8->isChecked())
    {
        depth = 8;
		
        if (workWithStack)
			for (uint i = 0; i < imageStack.size(); i++){
				std::cout<<"Cropping image"<<i+1<<std::endl;
                this->cropProbeImage(imageStack.at(i), depth);
			}
        else
            cropImage = this->cropProbeImage(this->image, depth);
    }
    else
    {
        QErrorMessage errorMessage(this);
        errorMessage.showMessage("<b>No Depth selected</b> <br /> Select one depth before crop");
        errorMessage.exec();
        return;
    }
    
    // show croped images
    //    MainWindow* w = dynamic_cast<MainWindow*> (this->parentWidget());    
    if (0 != mainWindow)
    {
        if (workWithStack)
        {
            mainWindow->getDisplayWidget()->setAndDisplayMultipleImages(cropStack);
            
            QString str = "crop image stack with depth = ";
            QString num;
            num.setNum(depth);
            mainWindow->addLogText(str + num);
        }
        else
        {
            mainWindow->getDisplayWidget()->setAndDisplayImage(cropImage);
            
            QString str = "crop image with depth = ";
            QString num;
            num.setNum(depth);
            mainWindow->addLogText(str + num);
            
            cropImage = NULL;
        }
    }
}


void ProbeCalibrationWidget::setImage(vtkSmartPointer<vtkImageData> image)
{
    this->workWithStack = false;
    this->image = image;
}


void ProbeCalibrationWidget::
setImageStack(std::vector<vtkSmartPointer<vtkImageData> > imagestack)
{
    this->workWithStack = true;
    this->imageStack = imagestack;
    this->coords.set_size(imageStack.size(), 2);
}


vtkSmartPointer<vtkImageData> ProbeCalibrationWidget::
cropProbeImage(vtkSmartPointer<vtkImageData> image, int depthType)
{
	
    vtkSmartPointer<vtkExtractVOI> extractVOI = vtkSmartPointer<vtkExtractVOI>::New();
    
    
    int* dim = image->GetDimensions();
    
    switch (depthType)
    {
        case 4:
        {
            extractVOI->SetInput(image);
            extractVOI->SetVOI(91, 478, dim[1] - 446, dim[1] - 49, 0, 0);
            extractVOI->Update();
            return extractVOI->GetOutput();
        }
        case 5:
        {
            extractVOI->SetInput(image);
            extractVOI->SetVOI(136, 435, dim[1] - 446, dim[1] - 49, 0, 0);
            extractVOI->Update();
            return extractVOI->GetOutput();
        }
        case 6:
        {
            extractVOI->SetInput(image);
            extractVOI->SetVOI(155, 414, dim[1] - 446, dim[1] - 49, 0, 0);
            extractVOI->Update();
            return extractVOI->GetOutput();
        }
        case 8:
        {
            extractVOI->SetInput(image);
            extractVOI->SetVOI(188, 380, dim[1] - 446, dim[1] - 49, 0, 0);
            extractVOI->Update();
            return extractVOI->GetOutput();
        }
        default:
        {
            //                MainWindow* w = dynamic_cast<MainWindow*> (this->parentWidget());
            //                if (0 != w)
            if (mainWindow != 0)
            {
                //                        w->addLogText("depth not found, nothing made");
                mainWindow->addLogText("depth not found, nothing made");
            }
            return image;
        }
    }
}


void ProbeCalibrationWidget::setMainWindow(MainWindow* mainwindow)
{
    this->mainWindow = mainwindow;
}


bool setCoordsSize = false;


void ProbeCalibrationWidget::getCoordinates()
{
	std::cout<<std::endl;
    int x = mainWindow->getDisplayWidget()->getXPicked();
    int y = mainWindow->getDisplayWidget()->getYPicked();
    
    QString str;
    
    if (workWithStack)
    {
        this->coords.set_size(imageStack.size(), 2);
        
        
        tableWidget->setRowCount(imageStack.size());
        int row = mainWindow->getDisplayWidget()->getImageDisplayedIndex();
        tableWidget->setItem(row, 0, new QTableWidgetItem(str.setNum(row)));
        tableWidget->setItem(row, 1, new QTableWidgetItem(str.setNum(x)));
        tableWidget->setItem(row, 2, new QTableWidgetItem(str.setNum(y)));
       
		std::cout<<"Picked Pixel for Image "<<row<< ": "<<x<<","<<y<<std::endl;

		coords[row][0] = x;
        coords[row][1] = y;
    }
    else
    {
        tableWidget->setRowCount(1);
        tableWidget->setItem(0, 0, new QTableWidgetItem(str.setNum(1)));
        tableWidget->setItem(0, 1, new QTableWidgetItem(str.setNum(x)));
        tableWidget->setItem(0, 2, new QTableWidgetItem(str.setNum(y)));
        std::cout << "PC -> (x = " << x << ", y = " << y << ")" << std::endl;
    }
}


void ProbeCalibrationWidget::loadRotationsFile()
{
    QString rotationFilename = QFileDialog::getOpenFileName(this, tr("Load Rotations File"), 
		QDir::currentPath(), tr("Txt (*.txt *.doc)"));

	std::cout<<std::endl;
    std::cout<<"Loading Rotations File"<<std::endl;

    if (!rotationFilename.isEmpty())
    {

        QFile file(rotationFilename);
        if (!file.open(QIODevice::ReadOnly)){
            std::cout<<"Could not open rotations File"<<std::endl;
             return;
         }
        
        QTextStream stream(&file);
        QString line;
        
        // declare a rotation matrix 
        this->rotations.set_size(imageStack.size(), 4);
        
        int idx = 0;
        while (!stream.atEnd())
        {
            line = stream.readLine();
            QStringList lineList = line.split(" ");
            
            for (int j = 0; j < lineList.size(); j++)
            {
                rotations.put(idx, j, lineList.at(j).toDouble());                 
            }
            
            idx++;            
            //std::cout << line.toAscii().data() << std::endl;
        }
        file.close();        
        rotationsLabel->setText("Rotations file are loaded");
    }
}


void ProbeCalibrationWidget::loadTranslationsFile()
{
    QString translationFilename = QFileDialog::getOpenFileName(this, tr("Load Translations File"), 
		QDir::currentPath(),tr("Txt (*.txt *.doc)"));
    
	std::cout<<std::endl;
	std::cout<<"Loading Translations File"<<std::endl;

    if (!translationFilename.isEmpty())
    {
        
        QFile file(translationFilename);
        if (!file.open(QIODevice::ReadOnly)){
           std::cout<<"Could not open translations File"<<std::endl;
            return;
        }
        
        QTextStream stream(&file);
        QString line;
        
        // declare a translation matrix 
        this->translations.set_size(imageStack.size(), 3); 
        
        int idx = 0;
        while (!stream.atEnd())
        {
            line = stream.readLine();
            QStringList lineList = line.split(" ");
            
            for (int j = 0; j < lineList.size(); j++)
            {
                translations.put(idx, j, lineList.at(j).toDouble());        
            }
            idx++;   
            //std::cout << line.toAscii().data() << std::endl;
        }
        file.close(); // when your done.        
        translationsLabel->setText("Translations file are loaded");
    }
}


void ProbeCalibrationWidget::calibrate()
{

    SinglePointUltrasoundCalibration * calibrator = new SinglePointUltrasoundCalibration;
	calibrator->setLSQR();
	calibrator->setQuaternions(rotations);
	calibrator->setTranslations(translations);
	calibrator->setSinglePointCoords(coords);

	vnl_vector<double> * estimatedCalibrationParameters = new vnl_vector<double>;
	estimatedCalibrationParameters->set_size(11);
	if(calibrator->calibrate(estimatedCalibrationParameters)){
		std::cout<<"Succesfull Calibration "<<estimatedCalibrationParameters->get(0)<<","<<estimatedCalibrationParameters->get(1)<<","<<estimatedCalibrationParameters->get(2)<<std::endl;
		vnl_vector<double> estimatedGets = calibrator->getEstimatedCalibrationParameters();
		std::cout<<"Succesfull Calibration "<<estimatedGets.get(0)<<","<<estimatedGets.get(1)<<","<<estimatedGets.get(2)<<std::endl;
	}


    
}


void ProbeCalibrationWidget::saveCalibration()
{

    QString saveCalibrationDirectory = QFileDialog::getExistingDirectory(
                this, tr("Choose Directory to Save Calibration"), QDir::currentPath(), 
				QFileDialog::ShowDirsOnly );

    QString filename = "/EstimatedCalibrationParameters.txt";

    QString saveCalibrationFile = saveCalibrationDirectory.append(filename);

    QFile file(saveCalibrationFile);

    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)){
        std::cout<<"Could not open File to save the estimated calibration parameters"<<std::endl;
        return;
    }

	std::cout<<std::endl;
	std::cout<<"Writing Estimated US Calibration Parameters"<<std::endl;

	QTextStream out(&file);

	out<<calibrationParameters[3]<<"\n";
	out<<calibrationParameters[4]<<"\n";
	out<<calibrationParameters[5]<<"\n";
	out<<calibrationParameters[6]<<"\n";
	out<<calibrationParameters[7]<<"\n";
	out<<calibrationParameters[8]<<"\n";
	out<<calibrationParameters[9]<<"\n";
	out<<calibrationParameters[10]<<"\n";

	file.close();

}







