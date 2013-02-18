#include "SinglePointUltrasoundCalibration.h"

SinglePointUltrasoundCalibration::SinglePointUltrasoundCalibration()
{
    LSQRMethod = true;
}

SinglePointUltrasoundCalibration::~SinglePointUltrasoundCalibration()
{

}

bool SinglePointUltrasoundCalibration::calibrate()
{
	if(quaternions.empty())
	{
		std::cout<<"No quaternions loaded"<<std::endl;
		return false;
	}

	if(translations.empty())
	{
		std::cout<<"No translations loaded"<<std::endl;
		return false;
	}

	if(singlePointCoords.empty())
	{
		std::cout<<"No singlePointCoords loaded"<<std::endl;
		return false;
	}

	unsigned int rowsQuaternions = quaternions.rows();
	unsigned int rowsTranslations = translations.rows();
	unsigned int rowsSinglePointCoords = singlePointCoords.rows();

	if(rowsQuaternions!=rowsTranslations || rowsTranslations!=rowsSinglePointCoords || rowsQuaternions!=rowsSinglePointCoords)
	{
		std::cout<<"The number of rows in quaternions, translations and singlePointCoords do not agree"<<std::endl;
		return false;
	}

	if(LSQRMethod)
		return calibrateLSQR();
	else
		return calibrateVNL();

}

bool SinglePointUltrasoundCalibration::calibrate(vnl_vector<double>  * estimatedCalibrationParameters)
{
	if(quaternions.empty())
	{
		std::cout<<"No quaternions loaded"<<std::endl;
		return false;
	}

	if(translations.empty())
	{
		std::cout<<"No translations loaded"<<std::endl;
		return false;
	}

	if(quaternions.empty())
	{
		std::cout<<"No singlePointCoords loaded"<<std::endl;
		return false;
	}

	unsigned int rowsQuaternions = quaternions.rows();
	unsigned int rowsTranslations = translations.rows();
	unsigned int rowsSinglePointCoords = singlePointCoords.rows();

	if(rowsQuaternions!=rowsTranslations || rowsTranslations!=rowsSinglePointCoords || rowsQuaternions!=rowsSinglePointCoords)
	{
		std::cout<<"The number of rows in quaternions, translations and singlePointCoords do not agree"<<std::endl;
		return false;
	}

	if(LSQRMethod)
		return calibrateLSQR(estimatedCalibrationParameters);
	else
		return calibrateVNL(estimatedCalibrationParameters);
}

void SinglePointUltrasoundCalibration::setLSQR()
{
    LSQRMethod = true;
}

void SinglePointUltrasoundCalibration::setVNL()
{
    LSQRMethod = false;
}

void SinglePointUltrasoundCalibration::setQuaternions(vnl_matrix<double> quaternions)
{
	
	if(quaternions.cols()!=4){
		std::cout<<"Quaternions must be a (4,n) vnl_matrix"<<std::endl;
		return;
	}else
		this->quaternions = quaternions;
}

void SinglePointUltrasoundCalibration::setTranslations(vnl_matrix<double> translations)
{
	if(translations.cols()!=3){
		std::cout<<"Translations must be a (3,n) vnl_matrix"<<std::endl;
		return;
	}else
		this->translations = translations;
}

void SinglePointUltrasoundCalibration::setSinglePointCoords(vnl_matrix<double> singlePointCoords)
{
	if(singlePointCoords.cols()!=2){
		std::cout<<"SinglePointCoords must be a (2,n) vnl_matrix"<<std::endl;
		return;
	}else
		this->singlePointCoords = singlePointCoords;
}

void SinglePointUltrasoundCalibration::setInputData(vnl_matrix<double> quaternions,
                                               vnl_matrix<double> translations,
                                               vnl_matrix<double> singlePointCoords)
{
	if(quaternions.cols()!=4){
		std::cout<<"Quaternions must be a (4,n) vnl_matrix"<<std::endl;
		return;
	}else
		this->quaternions = quaternions;

	if(quaternions.cols()!=4){
		std::cout<<"Translations must be a (3,n) vnl_matrix"<<std::endl;
		return;
	}else
		this->translations = translations;

	if(quaternions.cols()!=4){
		std::cout<<"SinglePointCoords must be a (2,n) vnl_matrix"<<std::endl;
		return;
	}else
		this->singlePointCoords = singlePointCoords;

}

bool SinglePointUltrasoundCalibration::getCalibrationMethod()
{
    return LSQRMethod;
}

vnl_vector<double> SinglePointUltrasoundCalibration::getEstimatedCalibrationParameters()
{
    return estimatedCalibrationParameters;
}

bool SinglePointUltrasoundCalibration::calibrateLSQR()
{
	typedef lsqrRecipes::SingleUnknownPointTargetUSCalibrationParametersEstimator::DataType DataType;

	std::vector<lsqrRecipes::Frame> transformations;
	std::vector<lsqrRecipes::Point2D> singlePoints;
	std::vector< DataType > data;

	for (unsigned int i = 0; i < quaternions.rows(); i++) {            
         
        vnl_quaternion<double> quaternion(quaternions[i][1], quaternions[i][2], 
			quaternions[i][3], quaternions[i][0]);

        vnl_matrix<double> rotation = quaternion.rotation_matrix_transpose();
        rotation = rotation.transpose();

		lsqrRecipes::Frame f;

		f.setRotationMatrix(rotation);
		f.setTranslation(translations.get_row(i));
		transformations.push_back(f);

		lsqrRecipes::Point2D point;
	
		point[0]=singlePointCoords[i][0];
		point[1]=singlePointCoords[i][1];

		singlePoints.push_back(point);
    }

	DataType dataElement;

	std::cout<<"Calculating calibration parameters"<<std::endl<<std::endl;
	size_t n = transformations.size();

	for(int i=0; i<n; i++) {
		dataElement.T2 = transformations[i];
		dataElement.q = singlePoints[i];
		data.push_back(dataElement);
	}

	double maxDistanceBetweenPoints = 1.0;
	lsqrRecipes::SingleUnknownPointTargetUSCalibrationParametersEstimator 
		usCalibration(maxDistanceBetweenPoints);

  
	std::vector<double> estimatedLSQRCalibrationParameters;

	usCalibration.setLeastSquaresType(lsqrRecipes::SingleUnknownPointTargetUSCalibrationParametersEstimator::ANALYTIC);
	usCalibration.leastSquaresEstimate(data, estimatedLSQRCalibrationParameters);

	estimatedCalibrationParameters.set_size(11);

	if(estimatedLSQRCalibrationParameters.size() == 0){
		std::cout<<"DEGENERATE CONFIGURATION\n\n\n";
		return false;
	}
	else {
		
		estimatedCalibrationParameters[0] = estimatedLSQRCalibrationParameters[0];
		estimatedCalibrationParameters[1] = estimatedLSQRCalibrationParameters[1];
		estimatedCalibrationParameters[2] = estimatedLSQRCalibrationParameters[2];
		estimatedCalibrationParameters[3] = estimatedLSQRCalibrationParameters[3];
		estimatedCalibrationParameters[4] = estimatedLSQRCalibrationParameters[4];
		estimatedCalibrationParameters[5] = estimatedLSQRCalibrationParameters[5];
		estimatedCalibrationParameters[6] = estimatedLSQRCalibrationParameters[6];
		estimatedCalibrationParameters[7] = estimatedLSQRCalibrationParameters[7];
		estimatedCalibrationParameters[8] = estimatedLSQRCalibrationParameters[8];
		estimatedCalibrationParameters[9] = estimatedLSQRCalibrationParameters[9];
		estimatedCalibrationParameters[10] = estimatedLSQRCalibrationParameters[10];
		
		std::cout<<"t1[x,y,z]:\n";
		std::cout<<"\t["<<estimatedLSQRCalibrationParameters[0]<<", "<<estimatedLSQRCalibrationParameters[1];
		std::cout<<", "<<estimatedLSQRCalibrationParameters[2]<<"]\n";
		std::cout<<"t3[x,y,z]:\n";
		std::cout<<"\t["<<estimatedLSQRCalibrationParameters[3]<<", "<<estimatedLSQRCalibrationParameters[4];
		std::cout<<", "<<estimatedLSQRCalibrationParameters[5]<<"]\n";
		std::cout<<"omega[z,y,x]:\n";
		std::cout<<"\t["<<estimatedLSQRCalibrationParameters[6]<<", "<<estimatedLSQRCalibrationParameters[7];
		std::cout<<", "<<estimatedLSQRCalibrationParameters[8]<<"]\n";
		std::cout<<"m[x,y]:\n";
		std::cout<<"\t["<<estimatedLSQRCalibrationParameters[9];
		std::cout<<", "<<estimatedLSQRCalibrationParameters[10]<<"]\n";
	}	

	return true;
}

bool SinglePointUltrasoundCalibration::calibrateLSQR(vnl_vector<double> * estimatedCalibrationParameters)
{
	typedef lsqrRecipes::SingleUnknownPointTargetUSCalibrationParametersEstimator::DataType DataType;

	std::vector<lsqrRecipes::Frame> transformations;
	std::vector<lsqrRecipes::Point2D> singlePoints;
	std::vector< DataType > data;

	for (unsigned int i = 0; i < quaternions.rows(); i++) {            
         
        vnl_quaternion<double> quaternion(quaternions[i][1], quaternions[i][2], 
			quaternions[i][3], quaternions[i][0]);

        vnl_matrix<double> rotation = quaternion.rotation_matrix_transpose();
        rotation = rotation.transpose();

		lsqrRecipes::Frame f;

		f.setRotationMatrix(rotation);
		f.setTranslation(translations.get_row(i));
		transformations.push_back(f);

		lsqrRecipes::Point2D point;
	
		point[0]=singlePointCoords[i][0];
		point[1]=singlePointCoords[i][1];

		singlePoints.push_back(point);
    }

	DataType dataElement;

	std::cout<<"Calculating calibration parameters"<<std::endl<<std::endl;
	size_t n = transformations.size();

	for(int i=0; i<n; i++) {
		dataElement.T2 = transformations[i];
		dataElement.q = singlePoints[i];
		data.push_back(dataElement);
	}

	double maxDistanceBetweenPoints = 1.0;
	lsqrRecipes::SingleUnknownPointTargetUSCalibrationParametersEstimator 
		usCalibration(maxDistanceBetweenPoints);

  
	std::vector<double> estimatedLSQRCalibrationParameters;
	

	usCalibration.setLeastSquaresType(lsqrRecipes::SingleUnknownPointTargetUSCalibrationParametersEstimator::ANALYTIC);
	usCalibration.leastSquaresEstimate(data, estimatedLSQRCalibrationParameters);

	this->estimatedCalibrationParameters.set_size(11);

	if(estimatedLSQRCalibrationParameters.size() == 0){
		std::cout<<"DEGENERATE CONFIGURATION\n\n\n";
		return false;
	}
	else {
		
		estimatedCalibrationParameters->put(0,estimatedLSQRCalibrationParameters[0]);
		estimatedCalibrationParameters->put(1,estimatedLSQRCalibrationParameters[1]);
		estimatedCalibrationParameters->put(2,estimatedLSQRCalibrationParameters[2]);
		estimatedCalibrationParameters->put(3,estimatedLSQRCalibrationParameters[3]);
		estimatedCalibrationParameters->put(4,estimatedLSQRCalibrationParameters[4]);
		estimatedCalibrationParameters->put(5,estimatedLSQRCalibrationParameters[5]);
		estimatedCalibrationParameters->put(6,estimatedLSQRCalibrationParameters[6]);
		estimatedCalibrationParameters->put(7,estimatedLSQRCalibrationParameters[7]);
		estimatedCalibrationParameters->put(8,estimatedLSQRCalibrationParameters[8]);
		estimatedCalibrationParameters->put(9,estimatedLSQRCalibrationParameters[9]);
		estimatedCalibrationParameters->put(10,estimatedLSQRCalibrationParameters[10]);

		this->estimatedCalibrationParameters = *estimatedCalibrationParameters;

		std::cout<<"t1[x,y,z]:\n";
		std::cout<<"\t["<<estimatedLSQRCalibrationParameters[0]<<", "<<estimatedLSQRCalibrationParameters[1];
		std::cout<<", "<<estimatedLSQRCalibrationParameters[2]<<"]\n";
		std::cout<<"t3[x,y,z]:\n";
		std::cout<<"\t["<<estimatedLSQRCalibrationParameters[3]<<", "<<estimatedLSQRCalibrationParameters[4];
		std::cout<<", "<<estimatedLSQRCalibrationParameters[5]<<"]\n";
		std::cout<<"omega[z,y,x]:\n";
		std::cout<<"\t["<<estimatedLSQRCalibrationParameters[6]<<", "<<estimatedLSQRCalibrationParameters[7];
		std::cout<<", "<<estimatedLSQRCalibrationParameters[8]<<"]\n";
		std::cout<<"m[x,y]:\n";
		std::cout<<"\t["<<estimatedLSQRCalibrationParameters[9];
		std::cout<<", "<<estimatedLSQRCalibrationParameters[10]<<"]\n";
	}	

	return true;
}

bool SinglePointUltrasoundCalibration::calibrateVNL()
{
return true;
}

bool SinglePointUltrasoundCalibration::calibrateVNL(vnl_vector<double> * estimatedCalibrationParameters)
{
return true;
}




