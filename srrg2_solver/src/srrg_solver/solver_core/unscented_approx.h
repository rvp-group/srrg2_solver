// file to be included explicitly.
// sucks big times


inline ErrorVectorType evaluateTotalPerturbation(const TotalPerturbationVectorType& v) {

  typename ErrorFunctionType::TotalJacobianMatrixType J_saved = ErrorFunctionType::_J;
  ErrorVectorType e_saved = ErrorFunctionType::_e;

  TotalPerturbationVectorType xx;
  xx.setZero();

  ErrorVectorType err;
  err.setZero();
  for (int i = 0; i < NumVariables; ++i) {
    variable(i)->push();
  }
  int current_offset = 0;
  for (int i = 0; i < NumVariables; ++i) {
    variable(i)->applyPerturbationRaw(&v[current_offset]);
    current_offset += variable(i)->perturbationDim();
  }
  ErrorFunctionType::compute();
  err = ErrorFunctionType::_e;
  for (int i = 0; i < NumVariables; ++i) {
    variable(i)->pop();
  }
  ErrorFunctionType::_J = J_saved;
  ErrorFunctionType::_e = e_saved;
  return err;
}

virtual void computeUnscented() {

  if (!isActive()) {
    return;
  }

  ErrorFunctionType::compute();
  if (!ErrorFunctionType::isValid()) {
    return;
  }

  _chi = this->error().transpose() * _information_matrix * this->error(); // something more fancy with
  _status = FactorStats::Status::Inlier;
  // unscented stuff
  TotalCrossCorrelationMatrixType Sigma_xz = TotalCrossCorrelationMatrixType::Zero();
  InformationMatrixType Omega_zz = InformationMatrixType::Zero();
  InformationMatrixType Sigma_zz = InformationMatrixType::Zero();

  const int dim = TotalPerturbationDim;
  const int num_points = 2 * dim + 1;

  // we need to figure out the coefficients for the sigma points
  const float alpha = 1e-3;
  const float beta = 2.;
  const float lambda = alpha * alpha * dim;
  const float wi = 1. / (2. * (dim + lambda));

  SigmaPoint sigma_points_x[num_points];
  ErrorVectorType sigma_points_z[num_points];

  sigma_points_x[0].x.setZero();
  sigma_points_x[0].wi = lambda / (dim + lambda);
  sigma_points_x[0].wp = lambda / (dim + lambda) + (1. - alpha * alpha + beta);
  sigma_points_z[0] = evaluateTotalPerturbation(sigma_points_x[0].x);

  if (!isValid()) {
    return;
  }

  // ok, we fake the cholesky decomposition
  Eigen::Matrix<float, TotalPerturbationDim, TotalPerturbationDim> chol_decomp;
  chol_decomp.setIdentity();
  chol_decomp *= 1e-2;
  int k = 1;
  for (int i = 0; i < dim; ++i) {
    sigma_points_x[k].wi = wi;
    sigma_points_x[k].wp = wi;
    sigma_points_x[k].x = chol_decomp.col(i);
    sigma_points_z[k] = evaluateTotalPerturbation(sigma_points_x[k].x);
    ++k;
    if (!isValid()) {
      return;
    }

    sigma_points_x[k].wi = wi;
    sigma_points_x[k].wp = wi;
    sigma_points_x[k].x = -chol_decomp.col(i);
    sigma_points_z[k] = evaluateTotalPerturbation(sigma_points_x[k].x);
    ++k;
    if (!isValid()) {
      return;
    }
  }

  // here we have sigma points for the state and the measurements
  // we can proceed computing the cross correlations


  Sigma_zz = _information_matrix.inverse();
  Sigma_xz.setZero();
  ErrorFunctionType::_e.setZero();
  for (int i = 0; i < num_points; ++i) {
    ErrorFunctionType::_e += sigma_points_x[i].wi * sigma_points_z[i];
  }
  for (int i = 0; i < num_points; ++i) {
    ErrorVectorType delta = sigma_points_z[i] - ErrorFunctionType::_e;
    Sigma_xz += sigma_points_x[i].wp * sigma_points_x[i].x * delta.transpose();
    Sigma_zz += sigma_points_x[i].wp * delta * delta.transpose();
  }
  Omega_zz = Sigma_zz.inverse();

  TotalPerturbationVectorType dx = Sigma_xz * Omega_zz * ErrorFunctionType::_e;
  int pos = 0;
  for (int r = 0; r < NumVariables; ++r) {
    MatrixBlockBase* b = _b_blocks[r];
    if (!b) {
      continue;
    }
    for (int d = 0; d < b->rows(); ++d, ++pos) {
      b->storage()[d] -= dx[pos];
    }
  }
};
