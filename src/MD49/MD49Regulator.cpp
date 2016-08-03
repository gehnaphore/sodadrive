/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "MD49Regulator.h"

#include <limits>

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

MD49Regulator::MD49Regulator(double const kP_1, double const kI_1,
                             double const kP_2, double const kI_2,
                             double const dt_s)
: _speed_1_regulator(kP_1, kI_1, dt_s, std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max()),
  _speed_2_regulator(kP_2, kI_2, dt_s, std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max()),
  _speed_1_m_per_s_target_value(0.0),
  _speed_2_m_per_s_target_value(0.0),
  _speed_1(0),
  _speed_2(0)
{

}

void MD49Regulator::setSpeed1TargetValue(double const speed_1_m_per_s)
{
  _speed_1_m_per_s_target_value = speed_1_m_per_s;
}

void MD49Regulator::setSpeed2TargetValue(double const speed_2_m_per_s)
{
  _speed_2_m_per_s_target_value = speed_2_m_per_s;
}

void MD49Regulator::updateWithActualValue(double const speed_1_m_per_s,
                                          double const speed_2_m_per_s)
{
  _speed_1 = static_cast<int8_t>(_speed_1_regulator.calc(_speed_1_m_per_s_target_value, speed_1_m_per_s));
  _speed_2 = static_cast<int8_t>(_speed_2_regulator.calc(_speed_2_m_per_s_target_value, speed_2_m_per_s));
}

int8_t MD49Regulator::getSpeed1() const
{
  return _speed_1;
}

int8_t MD49Regulator::getSpeed2() const
{
  return _speed_2;
}
