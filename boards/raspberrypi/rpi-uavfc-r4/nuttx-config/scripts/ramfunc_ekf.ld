/* EKF functions selected from the PC profile (2026-11-02, build 9d120fe) by samples per byte: budget 49152, used 49078 B, 90 functions */
*(.text._ZN3Ekf17measurementUpdateERN6matrix6VectorIfLj24EEERKS2_ff)  /* Ekf::measurementUpdate(matrix::Vector<float, 24u>&, matrix::Vector<float, 24u> const&, float, float)  338 B  5.7% */
*(.text._ZN3Ekf17constrainStateVarERKN9estimator6IdxDofEff)  /* Ekf::constrainStateVar(estimator::IdxDof const&, float, float)  144 B  0.3% */
*(.text._ZN3Ekf26fuseDirectStateMeasurementEfffib)  /* Ekf::fuseDirectStateMeasurement(float, float, float, int, bool)  316 B  0.5% */
*(.text._ZN6matrix6MatrixIfLj4ELj1EEaSERKS1_.isra.0)  /* matrix::Matrix<float, 4u, 1u>::operator=(matrix::Matrix<float, 4u, 1u> const&) [clone .isra.0]  22 B  0.0% */
*(.text._ZN3Ekf20controlGravityFusionERKN9estimator9imuSampleE)  /* Ekf::controlGravityFusion(estimator::imuSample const&)  1292 B  1.9% */
*(.text._ZN3Ekf27constrainStateVarLimitRatioERKN9estimator6IdxDofEfff)  /* Ekf::constrainStateVarLimitRatio(estimator::IdxDof const&, float, float, float)  148 B  0.2% */
*(.text._ZN6matrix9constrainIfLj3ELj1EEENS_6MatrixIT_XT0_EXT1_EEERKS3_S2_S2_)  /* matrix::Matrix<float, 3u, 1u> matrix::constrain<float, 3u, 1u>(matrix::Matrix<float, 3u, 1u> const&, float, float)  156 B  0.2% */
*(.text._ZN3Ekf23constrainStateVariancesEv)  /* Ekf::constrainStateVariances()  328 B  0.4% */
*(.text._ZNK6matrix6VectorIfLj4EE3dotERKNS_6MatrixIfLj4ELj1EEE)  /* matrix::Vector<float, 4u>::dot(matrix::Matrix<float, 4u, 1u> const&) const  32 B  0.0% */
*(.text._ZN6matrix9AxisAngleIfEC2ERKNS_10QuaternionIfEE)  /* matrix::AxisAngle<float>::AxisAngle(matrix::Quaternion<float> const&)  164 B  0.2% */
*(.text._ZN3Ekf19controlHeightFusionERKN9estimator9imuSampleE)  /* Ekf::controlHeightFusion(estimator::imuSample const&)  64 B  0.1% */
*(.text._ZNK6matrix10QuaternionIfE19rotateVectorInverseERKNS_7Vector3IfEE)  /* matrix::Quaternion<float>::rotateVectorInverse(matrix::Vector3<float> const&) const  220 B  0.2% */
*(.text._ZN6matrix6MatrixIfLj4ELj1EEC2ERKS1_)  /* matrix::Matrix<float, 4u, 1u>::Matrix(matrix::Matrix<float, 4u, 1u> const&)  18 B  0.0% */
*(.text._ZNK3Ekf13getRotVarBodyEv)  /* Ekf::getRotVarBody() const  360 B  0.4% */
*(.text._ZN15OutputPredictor37getAngularVelocityAndResetAccumulatorEv)  /* OutputPredictor::getAngularVelocityAndResetAccumulator()  104 B  0.1% */
*(.text._ZNK18EstimatorInterface11getPositionEv)  /* EstimatorInterface::getPosition() const  376 B  0.3% */
*(.text._ZNK6matrix7Vector3IfEplES1_)  /* matrix::Vector3<float>::operator+(matrix::Vector3<float>) const  64 B  0.1% */
*(.text._ZN6matrix6MatrixIfLj4ELj1EE11setIdentityEv)  /* matrix::Matrix<float, 4u, 1u>::setIdentity()  16 B  0.0% */
*(.text._ZN10EKFGSF_yaw5resetEv)  /* EKFGSF_yaw::reset()  22 B  0.0% */
*(.text._ZN4EKF213PublishStatusERKy)  /* EKF2::PublishStatus(unsigned long long const&)  648 B  0.5% */
*(.text._ZN6matrix6VectorIfLj4EE9normalizeEv)  /* matrix::Vector<float, 4u>::normalize()  46 B  0.0% */
*(.text._ZNK3Ekf30clearInhibitedStateKalmanGainsERN6matrix6VectorIfLj24EEE)  /* Ekf::clearInhibitedStateKalmanGains(matrix::Vector<float, 24u>&) const  108 B  0.1% */
*(.text._ZN4EKF215PublishOdometryERKyRKN9estimator9imuSampleE)  /* EKF2::PublishOdometry(unsigned long long const&, estimator::imuSample const&)  288 B  0.2% */
*(.text._ZN15OutputPredictor37applyCorrectionToVerticalOutputBufferEff)  /* OutputPredictor::applyCorrectionToVerticalOutputBuffer(float, float)  146 B  0.1% */
*(.text._ZN6matrix10QuaternionIfEC2ERKNS_9AxisAngleIfEE)  /* matrix::Quaternion<float>::Quaternion(matrix::AxisAngle<float> const&)  152 B  0.1% */
*(.text._ZN4EKF213PublishStatesERKy)  /* EKF2::PublishStates(unsigned long long const&)  126 B  0.1% */
*(.text._ZN15OutputPredictor35resetVelocityDerivativeAccumulationEv)  /* OutputPredictor::resetVelocityDerivativeAccumulation()  14 B  0.0% */
*(.text._ZNK3Ekf36estimateInertialNavFallingLikelihoodEv)  /* Ekf::estimateInertialNavFallingLikelihood() const  360 B  0.2% */
*(.text._ZN11AlphaFilterIN6matrix7Vector2IfEEE17updateCalculationERKS2_)  /* AlphaFilter<matrix::Vector2<float> >::updateCalculation(matrix::Vector2<float> const&)  46 B  0.0% */
*(.text._ZN4EKF215PublishAttitudeERKy.part.0.isra.0)  /* EKF2::PublishAttitude(unsigned long long const&) [clone .part.0] [clone .isra.0]  104 B  0.1% */
*(.text._ZN3Ekf4fuseERKN6matrix6VectorIfLj24EEEf)  /* Ekf::fuse(matrix::Vector<float, 24u> const&, float)  1338 B  0.8% */
*(.text._ZNK15OutputPredictor21getVelocityDerivativeEv)  /* OutputPredictor::getVelocityDerivative() const  60 B  0.0% */
*(.text._ZN15OutputPredictor29applyCorrectionToOutputBufferERKN6matrix7Vector3IfEES4_)  /* OutputPredictor::applyCorrectionToOutputBuffer(matrix::Vector3<float> const&, matrix::Vector3<float> const&)  74 B  0.0% */
*(.text._ZNK18EstimatorInterface23isNorthEastAidingActiveEv)  /* EstimatorInterface::isNorthEastAidingActive() const  58 B  0.0% */
*(.text._ZN15OutputPredictor19correctOutputStatesEyRKN6matrix10QuaternionIfEERKNS0_7Vector3IfEERK9LatLonAltS8_S8_)  /* OutputPredictor::correctOutputStates(unsigned long long, matrix::Quaternion<float> const&, matrix::Vector3<float> const&, LatLonAlt const&, matrix::Vector3<float> const&, matrix::Vector3<float> const&)  988 B  0.5% */
*(.text._ZN4EKF227PublishInnovationTestRatiosERKy)  /* EKF2::PublishInnovationTestRatios(unsigned long long const&)  292 B  0.2% */
*(.text._ZN15OutputPredictor21calculateOutputStatesEyRKN6matrix7Vector3IfEEfS4_f)  /* OutputPredictor::calculateOutputStates(unsigned long long, matrix::Vector3<float> const&, float, matrix::Vector3<float> const&, float)  812 B  0.4% */
*(.text._ZN14ImuDownSampler6updateERKN9estimator9imuSampleE)  /* ImuDownSampler::update(estimator::imuSample const&)  956 B  0.5% */
*(.text._ZNK3Ekf19get_ekf_soln_statusEv)  /* Ekf::get_ekf_soln_status() const  436 B  0.2% */
*(.text._ZNK3Ekf12getRotVarNedEv)  /* Ekf::getRotVarNed() const  84 B  0.0% */
*(.text._ZN4EKF226PublishInnovationVariancesERKy)  /* EKF2::PublishInnovationVariances(unsigned long long const&)  292 B  0.2% */
*(.text._ZN4EKF217UpdateCalibrationERKyRNS_19InFlightCalibrationERKN6matrix7Vector3IfEES8_fbb)  /* EKF2::UpdateCalibration(unsigned long long const&, EKF2::InFlightCalibration&, matrix::Vector3<float> const&, matrix::Vector3<float> const&, float, bool, bool)  400 B  0.2% */
*(.text._ZN3Ekf6updateEv.part.0)  /* Ekf::update() [clone .part.0]  612 B  0.3% */
*(.text._ZN18EstimatorInterface10setIMUDataERKN9estimator9imuSampleE)  /* EstimatorInterface::setIMUData(estimator::imuSample const&)  528 B  0.3% */
*(.text._ZN4EKF218PublishInnovationsERKy)  /* EKF2::PublishInnovations(unsigned long long const&)  302 B  0.2% */
*(.text._ZN3Ekf14stopGnssFusionEv)  /* Ekf::stopGnssFusion()  142 B  0.1% */
*(.text._ZN4EKF220PublishFusionControlERKy.part.0)  /* EKF2::PublishFusionControl(unsigned long long const&) [clone .part.0]  380 B  0.2% */
*(.text._ZN3Ekf18controlFusionModesERKN9estimator9imuSampleE)  /* Ekf::controlFusionModes(estimator::imuSample const&)  684 B  0.3% */
*(.text._ZN15OutputPredictor12outputSampleaSERKS0_.isra.0)  /* OutputPredictor::outputSample::operator=(OutputPredictor::outputSample const&) [clone .isra.0]  46 B  0.0% */
*(.text._ZNK3Ekf21updateAidSourceStatusI24estimator_aid_source3d_sN6matrix7Vector3IfEES4_EEvRT_RKyRKT1_RKT0_SE_SE_f.isra.0)  /* void Ekf::updateAidSourceStatus<estimator_aid_source3d_s, matrix::Vector3<float>, matrix::Vector3<float> >(estimator_aid_source3d_s&, unsigned long long const&, matrix::Vector3<float> const&, matrix::Vector3<float> const&, matrix::Vector3<float> const&, matrix::Vector3<float> const&, float) const [clone .isra.0]  608 B  0.3% */
*(.text._ZNK6matrix10QuaternionIfE8inversedEv)  /* matrix::Quaternion<float>::inversed() const  94 B  0.0% */
*(.text._ZN3Ekf17predictCovarianceERKN9estimator9imuSampleE)  /* Ekf::predictCovariance(estimator::imuSample const&)  16780 B  6.8% */
*(.text._ZN4EKF220PublishLocalPositionERKy)  /* EKF2::PublishLocalPosition(unsigned long long const&)  1404 B  0.6% */
*(.text._ZN4EKF221UpdateGyroCalibrationERKy)  /* EKF2::UpdateGyroCalibration(unsigned long long const&)  134 B  0.1% */
*(.text._ZN3Ekf12predictStateERKN9estimator9imuSampleE)  /* Ekf::predictState(estimator::imuSample const&)  1548 B  0.6% */
*(.text._ZN14ImuDownSampler5resetEv)  /* ImuDownSampler::reset()  208 B  0.1% */
*(.text._ZN3Ekf31checkVerticalAccelerationHealthERKN9estimator9imuSampleE)  /* Ekf::checkVerticalAccelerationHealth(estimator::imuSample const&)  376 B  0.1% */
*(.text._ZN3Ekf20updateIMUBiasInhibitERKN9estimator9imuSampleE)  /* Ekf::updateIMUBiasInhibit(estimator::imuSample const&)  520 B  0.2% */
*(.text._ZN4EKF222UpdateAccelCalibrationERKy)  /* EKF2::UpdateAccelCalibration(unsigned long long const&)  164 B  0.1% */
*(.text._ZNK3Ekf21get_ekf_lpos_accuracyEPfS0_)  /* Ekf::get_ekf_lpos_accuracy(float*, float*) const  160 B  0.1% */
*(.text._ZNK3Ekf40getHorizontalVelocityInnovationTestRatioEv)  /* Ekf::getHorizontalVelocityInnovationTestRatio() const  304 B  0.1% */
*(.text._ZNK3Ekf38getVerticalVelocityInnovationTestRatioEv)  /* Ekf::getVerticalVelocityInnovationTestRatio() const  144 B  0.0% */
*(.text._ZN3Ekf33updateVerticalDeadReckoningStatusEv)  /* Ekf::updateVerticalDeadReckoningStatus()  214 B  0.1% */
*(.text._ZN4EKF220UpdateMagCalibrationERKy)  /* EKF2::UpdateMagCalibration(unsigned long long const&)  324 B  0.1% */
*(.text._ZThn16_N4EKF23RunEv)  /* non-virtual thunk to EKF2::Run()  8 B  0.0% */
*(.text._ZNK3Ekf29getHeadingInnovationTestRatioEv)  /* Ekf::getHeadingInnovationTestRatio() const  168 B  0.0% */
*(.text._ZNK3Ekf40getHorizontalPositionInnovationTestRatioEv)  /* Ekf::getHorizontalPositionInnovationTestRatio() const  236 B  0.1% */
*(.text._ZN4EKF215PublishBaroBiasERKy)  /* EKF2::PublishBaroBias(unsigned long long const&)  148 B  0.0% */
*(.text._ZN4EKF217PublishSensorBiasERKy)  /* EKF2::PublishSensorBias(unsigned long long const&)  796 B  0.2% */
*(.text._ZNK3Ekf20getHeadingInnovRatioEv)  /* Ekf::getHeadingInnovRatio() const  112 B  0.0% */
*(.text._ZNK3Ekf18getHeadingInnovVarEv)  /* Ekf::getHeadingInnovVar() const  112 B  0.0% */
*(.text._ZN3Ekf24controlTerrainFakeFusionEv)  /* Ekf::controlTerrainFakeFusion()  264 B  0.1% */
*(.text._ZN4EKF217UpdateRangeSampleER17ekf2_timestamps_s)  /* EKF2::UpdateRangeSample(ekf2_timestamps_s&)  516 B  0.1% */
*(.text._ZN4EKF23RunEv)  /* EKF2::Run()  3124 B  0.7% */
*(.text._ZNK3Ekf38getVerticalPositionInnovationTestRatioEv)  /* Ekf::getVerticalPositionInnovationTestRatio() const  208 B  0.0% */
*(.text._ZN4EKF218UpdateAuxVelSampleER17ekf2_timestamps_s.part.0)  /* EKF2::UpdateAuxVelSample(ekf2_timestamps_s&) [clone .part.0]  88 B  0.0% */
*(.text._ZN18ZeroVelocityUpdate6updateER3EkfRKN9estimator9imuSampleE)  /* ZeroVelocityUpdate::update(Ekf&, estimator::imuSample const&)  216 B  0.0% */
*(.text._ZN4EKF222PublishAidSourceStatusERKy)  /* EKF2::PublishAidSourceStatus(unsigned long long const&)  2106 B  0.4% */
*(.text._ZN4EKF219UpdateGnssYawSampleEv)  /* EKF2::UpdateGnssYawSample()  132 B  0.0% */
*(.text._ZN4EKF216UpdateBaroSampleER17ekf2_timestamps_s)  /* EKF2::UpdateBaroSample(ekf2_timestamps_s&)  232 B  0.0% */
*(.text._ZN14ZeroGyroUpdate6updateER3EkfRKN9estimator9imuSampleE)  /* ZeroGyroUpdate::update(Ekf&, estimator::imuSample const&)  312 B  0.1% */
*(.text._ZNK3Ekf15getHeadingInnovEv)  /* Ekf::getHeadingInnov() const  110 B  0.0% */
*(.text._ZN4EKF217PublishEventFlagsERKy)  /* EKF2::PublishEventFlags(unsigned long long const&)  420 B  0.1% */
*(.text._ZN4EKF219PublishWindEstimateERKy)  /* EKF2::PublishWindEstimate(unsigned long long const&)  140 B  0.0% */
*(.text._ZNK3Ekf39getHeightAboveGroundInnovationTestRatioEv)  /* Ekf::getHeightAboveGroundInnovationTestRatio() const  128 B  0.0% */
*(.text._ZN3Ekf35updateHorizontalDeadReckoningstatusEv)  /* Ekf::updateHorizontalDeadReckoningstatus()  704 B  0.1% */
*(.text._ZN3Ekf23updateYawManualValidityEv)  /* Ekf::updateYawManualValidity()  120 B  0.0% */
*(.text._ZNK10EKFGSF_yaw10getLogDataEPfS0_S0_S0_S0_S0_)  /* EKFGSF_yaw::getLogData(float*, float*, float*, float*, float*, float*) const  78 B  0.0% */
*(.text._ZN4EKF220UpdateAirspeedSampleER17ekf2_timestamps_s)  /* EKF2::UpdateAirspeedSample(ekf2_timestamps_s&)  576 B  0.1% */
*(.text._ZN3Ekf21updateTerrainValidityEv)  /* Ekf::updateTerrainValidity()  336 B  0.0% */
