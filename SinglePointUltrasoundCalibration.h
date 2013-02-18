#ifndef SINGLEPOINTULTRASOUNDCALIBRATION_H
#define SINGLEPOINTULTRASOUNDCALIBRATION_H

#include "SinglePointTargetUSCalibrationParametersEstimator.h"
#include "RANSAC.h"

#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_quaternion.h>

class SinglePointUltrasoundCalibration
{

public:
	
	/**
	Constructor
	**/
	SinglePointUltrasoundCalibration();

	/**
	Destructor
	**/
	~SinglePointUltrasoundCalibration();

	/**
	Calibrates an ultrasound probe using LSQR or VNL methods
	[out] bool, true if the calibration process was succesfull
	**/
	bool calibrate();

	/**
	Calibrates an ultrasound probe using LSQR or VNL methods
	[in] vnl_vector pointer to the vector where the estimated parameters
		 are storaged.
	[out] bool, true if the calibration process was succesfull
	**/
	bool calibrate(vnl_vector<double> *);

	/**
	Set LSQR as the calibration method
	**/
	void setLSQR();

	/**
	Set VNL as the calibration method
	**/
	void setVNL();

	/**
	Set the quaternions of the rotation of each image
	[in] (4,n) vnl_matrix, each row has the quaternion for one image
	**/
	void setQuaternions(vnl_matrix<double>);

	/**
	Set the translations of each image
	[in] (3,n) vnl_matrix, each row has the translation for one image
	**/
	void setTranslations(vnl_matrix<double>);

	/**
	Set coords of the single point of each image
	[in] (2,n) vnl_matrix, each row has the coords for one image
	**/
	void setSinglePointCoords(vnl_matrix<double>);

	/**
	Set the data of the quaternions, trnslations and single points coords
	for each image.
	**/
	void setInputData(vnl_matrix<double>, vnl_matrix<double>, vnl_matrix<double>);

	/**
	Returns the calibration used calibration methods
	[out] bool, true if the method is LSQR, false if VNL is used 
	**/
	bool getCalibrationMethod();

	/**
	Returns the estimated calibration parameters
	[out] (11) vnl_vector with the eleven estimated parameters
		  (0-2) Translation from the single point phantom to the tracker
		  (3-5) Translation from the image plane to the sensor
		  (6-8) Rotation from the image plane to the sensor
		  (9-10) Scale factor
	**/
	vnl_vector<double> getEstimatedCalibrationParameters();

private:

	/**
	Calibrates an ultrasound probe using LSQR
	[out] bool, true if the calibration process was succesfull
	**/
	bool calibrateLSQR();

	/**
	Calibrates an ultrasound probe using LSQR
	[in] vnl_vector pointer to the vector where the estimated parameters
		 are storaged.
	[out] bool, true if the calibration process was succesfull
	**/
	bool calibrateLSQR(vnl_vector<double> *);

	/**
	Calibrates an ultrasound probe using VNL
	[out] bool, true if the calibration process was succesfull
	**/
	bool calibrateVNL();

	/**
	Calibrates an ultrasound probe using VNL
	[in] vnl_vector pointer to the vector where the estimated parameters
		 are storaged.
	[out] bool, true if the calibration process was succesfull
	**/
	bool calibrateVNL(vnl_vector<double> *);

	/**
	flag that indicates that LSQR is being used
	**/
	bool LSQRMethod;

	/**
	(4,n) vnl_matrix, each row has the quaternion for one image
	**/
	vnl_matrix<double> quaternions;

	/**
	(3,n) vnl_matrix, each row has the translation for one image
	**/
	vnl_matrix<double> translations;

	/**
	(2,n) vnl_matrix, each row has the coords for one image
	**/
	vnl_matrix<double> singlePointCoords;

	/**
	(11) vnl_vector with the eleven estimated parameters
		  (0-2) Translation from the single point phantom to the tracker
		  (3-5) Translation from the image plane to the sensor
		  (6-8) Rotation from the image plane to the sensor
		  (9-10) Scale factor
	**/
	vnl_vector<double> estimatedCalibrationParameters;

};
#endif // SINGLEPOINTULTRASOUNDCALIBRATION_H
