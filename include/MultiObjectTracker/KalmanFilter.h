#include "types.h"

namespace kf {

class KalmanFilter {
 public:
  KalmanFilter(float dt, float initCov, Vector<2> paramsCov) {
    m_matP = Matrix<4, 4>::Identity() * initCov;

    // process noise covariance
    m_matQ << 0.25F * std::pow(dt, 4), 0., 0.5F * std::pow(dt, 3), 0., 0., 0.25F * std::pow(dt, 4),
        0., 0.5F * std::pow(dt, 3), 0.5F * std::pow(dt, 3), 0., std::pow(dt, 2), 0., 0.,
        0.5F * std::pow(dt, 3), 0., std::pow(dt, 2);

    // measurement noise matrix
    m_matR << paramsCov(0), 0., 0., paramsCov(1);

    // measurement mapping matrix
    m_matH << 1.0F, 0., 0., 0., 0., 1.0F, 0., 0.;

    // state transition matrix
    m_matF << 1.0F, 0.0F, dt, 0.0F, 0.0F, 1.0F, 0.0F, dt, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        1.0F;
  }

  Vector<4>& vecX() { return m_stateVector; }
  const Vector<4> vecX() const { return m_stateVector; }

  Matrix<4, 4>& matP() { return m_matP; }
  const Matrix<4, 4>& matP() const { return m_matP; }

  Matrix<4, 4>& matQ() { return m_matQ; }
  const Matrix<4, 4>& matQ() const { return m_matQ; }

  Matrix<2, 2>& matR() { return m_matR; }
  const Matrix<2, 2>& matR() const { return m_matR; }

  Matrix<2, 4>& matH() { return m_matH; }
  const Matrix<2, 4>& matH() const { return m_matH; }

  Vector<4> predict() {
    m_stateVector = m_matF * m_stateVector;
    m_matP = m_matF * m_matP * m_matF.transpose() + m_matQ;
    return m_stateVector;
  }

  void correct(const Vector<2>& vecZ) {
    const Matrix<4, 4> matI{Matrix<4, 4>::Identity()};  // Identity matrix
    const Matrix<2, 2> matSk{
        m_matH * m_matP * m_matH.transpose() + m_matR};  // Innovation covariance
    const Matrix<4, 2> matKk{m_matP * m_matH.transpose() * matSk.inverse()};  // Kalman Gain

    m_stateVector = m_stateVector + matKk * (vecZ - (m_matH * m_stateVector));
    m_matP = (matI - matKk * m_matH) * m_matP;
  }

 private:
  Vector<4> m_stateVector{Vector<4>::Zero()};
  Matrix<4, 4> m_matP;
  Matrix<4, 4> m_matQ;
  Matrix<2, 2> m_matR{Matrix<2, 2>::Zero()};
  Matrix<2, 4> m_matH{Matrix<2, 4>::Zero()};
  Matrix<4, 4> m_matF;
};
}  // namespace kf