#include "TxManage.h"
#include "NVStorage.h"

extern void DebugReportFrame(const char* hdr, const CProtocol&, const char* ftr);

// CTxManage is used to send a data frame to the blue wire
//
// As the blue wire is bidirectional, we need to only allow our transmit data
// to reach the blue wire when we actually want to send data.
// At all other times we are listening to the blue wire, receiving any async data
//
// This requires external circuitry to toggle the Tx/Rx modes.
// A "Tx Gating" signal is used.
//   when high, transmit data is sent to the blue wire
//   when low, transmit data is blocked (Hi-Z)
//
// Ideally the circuit also prevents feeding back our own Tx data into the Rx pin
// but the main software loop handles this situation by only accepting Rx data when expected.
//
//  Timing diagram
//                                 ____________________
//   Tx Gate  ____________________|                    |___________________________
//            _____________________________________________________________________
//   Tx Data                          |||||||||||||||

CTxManage::CTxManage(int TxGatePin, HardwareSerial& serial) : 
  m_BlueWireSerial(serial),
  m_TxFrame(CProtocol::CtrlMode)
{
  m_bOnReq = false;
  m_bOffReq = false;
  m_bTxPending = false;
  m_nStartTime = 0;
  m_nTxGatePin = TxGatePin;
}

void CTxManage::begin()
{
  pinMode(m_nTxGatePin, OUTPUT);
  digitalWrite(m_nTxGatePin, LOW);
}

void
CTxManage::queueOnRequest()
{
  m_bOnReq = true;
}

void 
CTxManage::queueOffRequest()
{
  m_bOffReq = true;
}

void 
CTxManage::PrepareFrame(const CProtocol& basisFrame, bool isBTCmaster)
{
  // copy supplied frame, typically this will be the values an OEM controller delivered
  // which means we parrot that data by default.
  // When parroting, we must especially avoid ping ponging "set temperature"!
  // Otherwise we are supplied with the default params for standalone mode, which we 
  // then instil the NV parameters
  m_TxFrame = basisFrame;  

  // ALWAYS install on/off commands if required
  m_TxFrame.resetCommand();   // no command upon blue wire initially, unless a request is pending
  if(m_bOnReq) {
    m_bOnReq = false;
    m_TxFrame.onCommand();
  }
  if(m_bOffReq) {
    m_bOffReq = false;
    m_TxFrame.offCommand();
  }

  // 0x78 prevents the controller showing bum information when we parrot the OEM controller
  // heater is happy either way, the OEM controller has set the max/min stuff already
  if(isBTCmaster) {
    m_TxFrame.setActiveMode();   // this allows heater to svae the tuning params to EEPROM
    m_TxFrame.setFan_Min(pNVStorage->getFmin());
    m_TxFrame.setFan_Max(pNVStorage->getFmax());
    m_TxFrame.setPump_Min(pNVStorage->getPmin());
    m_TxFrame.setPump_Max(pNVStorage->getPmax());
    m_TxFrame.setThermostatMode(pNVStorage->getThermostatMode());
    m_TxFrame.setTemperature_Desired(pNVStorage->getTemperature());
  }
  else {
    m_TxFrame.setPassiveMode();    // this prevents the tuning parameters being saved by heater
  }

  // ensure CRC valid
  m_TxFrame.setCRC();
}

void
CTxManage::Start(unsigned long timenow)
{
  if(timenow == 0)  // avoid a black hole if millis() has wrapped to zero
    timenow++;

  m_nStartTime = timenow;
  m_bTxPending = true;
}

// generate a Tx Gate, then send the TxFrame to the Blue wire
// Note the serial data is ISR driven, we need to hold off
// for a while to let teh buffewred dat clear before closing the Tx Gate.
bool
CTxManage::CheckTx(unsigned long timenow)
{
  if(m_nStartTime) {

    long diff = timenow - m_nStartTime;

    if(diff > m_nStartDelay) {
      // begin front porch of Tx gating pulse
      digitalWrite(m_nTxGatePin, HIGH);
    }
    if(m_bTxPending && (diff > (m_nStartDelay + m_nFrontPorch))) {
      // front porch expired, perform serial transmission
      // Tx gate remains held high
      m_bTxPending = false;
      m_BlueWireSerial.write(m_TxFrame.Data, 24);  // write native binary values
    }
    if(diff > (m_nStartDelay + m_nFrameTime)) {
      // conclude Tx gating after (emperical) delay
      digitalWrite(m_nTxGatePin, LOW);
      m_nStartTime = 0;                          // cancel, we are DONE
    }
  }
  return m_nStartTime == 0;   // returns true when done
}

