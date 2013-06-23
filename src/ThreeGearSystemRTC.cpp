// -*- C++ -*-
/*!
 * @file  ThreeGearSystemRTC.cpp
 * @brief getting information from 3GearSystem
 * @date $Date$
 *
 * $Id$
 */

#include "ThreeGearSystemRTC.h"


// Module specification
// <rtc-template block="module_spec">
static const char* threegearsystemrtc_spec[] =
  {
    "implementation_id", "ThreeGearSystemRTC",
    "type_name",         "ThreeGearSystemRTC",
    "description",       "getting information from 3GearSystem",
    "version",           "1.0.0",
    "vendor",            "ogata-lab",
    "category",          "ThreeGearSyste",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ThreeGearSystemRTC::ThreeGearSystemRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_handPositionOut("handPosistion", m_handPosition),
    m_handRotationOut("handRotation", m_handRotation)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
ThreeGearSystemRTC::~ThreeGearSystemRTC()
{
}



RTC::ReturnCode_t ThreeGearSystemRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("handPosistion", m_handPositionOut);
  addOutPort("handRotation", m_handRotationOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ThreeGearSystemRTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThreeGearSystemRTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThreeGearSystemRTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t ThreeGearSystemRTC::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ThreeGearSystemRTC::onDeactivated(RTC::UniqueId ec_id)
{

  return RTC::RTC_OK;
}


RTC::ReturnCode_t ThreeGearSystemRTC::onExecute(RTC::UniqueId ec_id)
{

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ThreeGearSystemRTC::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThreeGearSystemRTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThreeGearSystemRTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThreeGearSystemRTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThreeGearSystemRTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void ThreeGearSystemRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(threegearsystemrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<ThreeGearSystemRTC>,
                             RTC::Delete<ThreeGearSystemRTC>);
  }
  
};


